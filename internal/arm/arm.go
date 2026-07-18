// Package arm is the commander's core state machine: current joint
// angles, mode (idle/live/replay), and the teach/replay recording loop.
//
// Design note (Option A, command-based teach — see project decisions):
// what gets recorded is the ANGLE WE COMMANDED, not a sensed position.
// This works identically for the mg995 joints (no feedback exists) and
// the SC15 joints (feedback exists but isn't used for recording, only
// for the stats panel / future hybrid drag-teach). Arm is the single
// source of truth for "current angle" on the Go side — the firmware
// does not need to be polled to know where the arm thinks it is.
//
// Units: all internal angle state is in tenths of a degree (deg10),
// matching the wire protocol exactly (protocol.MoveAllFrame etc take
// deg10 directly, no conversion at the send boundary). The only place
// degrees-vs-deg10 conversion happens is at the edges: Stats() for the
// JSON API/UI, and CSV import/export (internal/web/csv.go).
package arm

import (
	"context"
	"encoding/json"
	"fmt"
	"log"
	"sync"
	"time"

	"github.com/xet-labs/roboarm-commander/internal/protocol"
	"github.com/xet-labs/roboarm-commander/internal/uart"
)

type Mode string

const (
	ModeIdle   Mode = "idle"
	ModeLive   Mode = "live"
	ModeReplay Mode = "replay"
)

// Step is one recorded (or replayed) keyframe. Angles are deg10.
type Step struct {
	TMs    int64    `json:"t_ms"` // milliseconds since recording start
	Angles [4]int16 `json:"angles"`
}

const (
	AngleMinDeg10  int16 = 0
	AngleMaxDeg10  int16 = 1800
	HomeAngleDeg10 int16 = 900

	recordIntervalMs = 100 // teach-mode sample rate

	// jogFlushInterval paces how often accumulated jog deltas actually
	// hit the wire, decoupling it from however fast the Xbox bridge's
	// UDP packets arrive (which can be 100Hz+). Without this, jogging
	// floods the firmware's 16-deep command queue and every jog packet
	// beyond the queue's drain rate comes back as ERR_QUEUE_FULL.
	jogFlushInterval = 40 * time.Millisecond
	// jogMoveTimeMs must be a bit longer than jogFlushInterval so
	// consecutive flushed moves overlap smoothly (PwmJoint on the
	// firmware side software-interpolates over this duration; too
	// short and it snaps, too long and jogging feels laggy).
	jogMoveTimeMs uint16 = 60

	// homeMoveTimeMs: Home() is a deliberate, not-time-critical move —
	// give it more time than a jog flush so it doesn't slew the mg995
	// joints unrealistically fast from wherever they currently are.
	homeMoveTimeMs uint16 = 1200
)

type Stats struct {
	Mode          Mode       `json:"mode"`
	AnglesDeg     [4]float64 `json:"angles"` // human-readable degrees, NOT deg10 — UI convenience
	Connected     bool       `json:"connected"`
	LatencyMs     int        `json:"latency_ms"`
	Recording     bool       `json:"recording"`
	RecordedSteps int        `json:"recorded_steps"`
	Replaying     bool       `json:"replaying"`
}

type Arm struct {
	mu sync.Mutex

	link *uart.Link
	mode Mode

	angles [4]int16 // deg10
	dirty  bool     // true if angles changed since last jog flush

	recording bool
	recStart  time.Time
	recBuf    []Step
	lastRec   time.Time

	replayCancel context.CancelFunc
	replaying    bool

	connected bool
	latencyMs int

	// last claw state actually sent to the wire, so ClawSet can skip
	// redundant sends — see ClawSet below.
	clawSent     bool
	lastClawMode byte
	lastClawDuty byte
}

func New(link *uart.Link) *Arm {
	a := &Arm{
		link:   link,
		mode:   ModeIdle,
		angles: [4]int16{HomeAngleDeg10, HomeAngleDeg10, HomeAngleDeg10, HomeAngleDeg10},
	}
	go a.statePoller()
	go a.jogFlusher()
	return a
}

// --- Mode -------------------------------------------------------------

func (a *Arm) SetMode(m Mode) error {
	a.mu.Lock()
	defer a.mu.Unlock()

	if a.mode == ModeReplay && m != ModeReplay && a.replayCancel != nil {
		a.replayCancel()
		a.replaying = false
	}
	if a.mode == ModeLive && a.recording {
		// leaving live mode implicitly stops recording without saving —
		// caller should call StopRecording explicitly if they want the
		// data kept. This just prevents a dangling recorder.
		a.recording = false
	}

	a.mode = m
	return nil
}

func (a *Arm) Mode() Mode {
	a.mu.Lock()
	defer a.mu.Unlock()
	return a.mode
}

// --- Live jog -----------------------------------------------------------

// Jog applies a signed delta (deg10) to each joint. Only updates
// in-memory target state and marks it dirty — actual UART sends happen
// on jogFlusher's fixed cadence, not per-call, so a burst of jog calls
// (e.g. a high-report-rate controller) coalesces into one wire frame
// per flush tick instead of flooding the firmware's command queue.
func (a *Arm) Jog(deltaBase, deltaShoulder, deltaElbow, deltaWrist int16) {
	a.mu.Lock()
	if a.mode != ModeLive {
		a.mu.Unlock()
		return
	}
	deltas := [4]int16{deltaBase, deltaShoulder, deltaElbow, deltaWrist}

	changed := false
	for i, d := range deltas {
		if d == 0 {
			continue
		}
		next := clamp16(a.angles[i]+d, AngleMinDeg10, AngleMaxDeg10)
		if next == a.angles[i] {
			continue
		}
		a.angles[i] = next
		changed = true
	}
	if changed {
		a.dirty = true
	}
	a.mu.Unlock()

	if changed {
		a.maybeRecord()
	}
}

// SetJointDeg sets joint i (0=Base 1=Shoulder 2=Elbow 3=Wrist, matching
// protocol.JointBase..JointWrist) to an absolute angle in plain degrees
// — this is the dashboard slider/typed-value entry point, as opposed
// to Jog's relative deltas from the xbox bridge. Deliberately reuses
// Jog's exact architecture: only in-memory target state is updated
// here (gated to Live mode, same as Jog), and the existing jogFlusher
// paces the actual wire send. That matters because a dragged slider
// can fire many change events per second — without this, each drag
// event would otherwise need its own send and could flood the
// firmware's 16-deep command queue exactly like ungoverned jog packets
// would.
//
// The AngleMinDeg10/AngleMaxDeg10 clamp here is a loose global
// backstop only — the ESP32 firmware clamps for real per-joint via its
// own kJointLimits (config.h) on receipt, so an out-of-range typed
// value can't reach hardware; this just keeps a.angles sane in the
// meantime.
func (a *Arm) SetJointDeg(joint int, deg float64) error {
	if joint < 0 || joint >= protocol.JointCount {
		return fmt.Errorf("arm: joint must be 0-%d", protocol.JointCount-1)
	}
	deg10 := clamp16(int16(deg*10.0), AngleMinDeg10, AngleMaxDeg10)

	a.mu.Lock()
	if a.mode != ModeLive {
		a.mu.Unlock()
		return fmt.Errorf("arm: setting joint angle requires live mode (current: %s)", a.mode)
	}
	changed := a.angles[joint] != deg10
	if changed {
		a.angles[joint] = deg10
		a.dirty = true
	}
	a.mu.Unlock()

	if changed {
		a.maybeRecord()
	}
	return nil
}

// jogFlusher sends the current target angles as one MoveAll frame,
// at most once per jogFlushInterval, only while dirty and in Live mode.
func (a *Arm) jogFlusher() {
	ticker := time.NewTicker(jogFlushInterval)
	defer ticker.Stop()

	for range ticker.C {
		a.mu.Lock()
		if !a.dirty || a.mode != ModeLive {
			a.mu.Unlock()
			continue
		}
		snapshot := a.angles
		a.dirty = false
		a.mu.Unlock()

		if err := a.link.Send(protocol.MoveAllFrame(snapshot, jogMoveTimeMs)); err != nil {
			log.Printf("[arm] jog flush send failed: %v", err)
		}
	}
}

func (a *Arm) maybeRecord() {
	a.mu.Lock()
	defer a.mu.Unlock()
	if !a.recording {
		return
	}
	now := time.Now()
	if now.Sub(a.lastRec) < time.Duration(recordIntervalMs)*time.Millisecond {
		return
	}
	a.lastRec = now
	a.recBuf = append(a.recBuf, Step{
		TMs:    now.Sub(a.recStart).Milliseconds(),
		Angles: a.angles,
	})
}

// --- Teach recording ------------------------------------------------

func (a *Arm) StartRecording() error {
	a.mu.Lock()
	defer a.mu.Unlock()
	if a.mode != ModeLive {
		return fmt.Errorf("arm: recording requires live mode (current: %s)", a.mode)
	}
	a.recording = true
	a.recStart = time.Now()
	a.lastRec = time.Time{}
	a.recBuf = []Step{{TMs: 0, Angles: a.angles}} // anchor with the starting pose
	return nil
}

// StopRecording ends the current recording and returns the captured
// steps. Caller (web handler) is responsible for persisting via store.
func (a *Arm) StopRecording() []Step {
	a.mu.Lock()
	defer a.mu.Unlock()
	a.recording = false
	out := a.recBuf
	a.recBuf = nil
	return out
}

// --- Replay -------------------------------------------------------------

// Replay plays back steps asynchronously. Returns immediately; playback
// runs in a goroutine and can be cancelled via Stop() or SetMode(idle/live).
func (a *Arm) Replay(steps []Step) error {
	a.mu.Lock()
	if a.mode != ModeReplay {
		a.mu.Unlock()
		return fmt.Errorf("arm: replay requires replay mode (current: %s)", a.mode)
	}
	if len(steps) == 0 {
		a.mu.Unlock()
		return fmt.Errorf("arm: empty replay profile")
	}
	ctx, cancel := context.WithCancel(context.Background())
	a.replayCancel = cancel
	a.replaying = true
	a.mu.Unlock()

	go a.runReplay(ctx, steps)
	return nil
}

func (a *Arm) runReplay(ctx context.Context, steps []Step) {
	defer func() {
		a.mu.Lock()
		a.replaying = false
		a.mu.Unlock()
	}()

	start := time.Now()

	for i, step := range steps {
		target := start.Add(time.Duration(step.TMs) * time.Millisecond)
		wait := time.Until(target)
		if wait > 0 {
			select {
			case <-ctx.Done():
				return
			case <-time.After(wait):
			}
		}

		// Time-govern this move by how long until the NEXT recorded
		// step, so mg995 joints (software-interpolated on the firmware
		// side) glide between keyframes instead of snapping. Firmware
		// treats timeMs=0 as "as fast as possible" — avoid that for
		// anything but the last step.
		moveTimeMs := uint16(150) // sane default / last-step fallback
		if i+1 < len(steps) {
			gap := steps[i+1].TMs - step.TMs
			if gap > 0 && gap < 60000 { // sanity bound, ignore absurd gaps
				moveTimeMs = uint16(gap)
			}
		}

		a.mu.Lock()
		a.angles = step.Angles
		a.mu.Unlock()

		if err := a.link.Send(protocol.MoveAllFrame(step.Angles, moveTimeMs)); err != nil {
			log.Printf("[arm] replay send failed at step %d: %v", i, err)
		}
	}
}

func (a *Arm) StopReplay() {
	a.mu.Lock()
	defer a.mu.Unlock()
	if a.replayCancel != nil {
		a.replayCancel()
	}
	a.replaying = false
}

// --- Immediate actions ------------------------------------------------

// Home sends the arm to its home pose immediately, regardless of mode
// (deliberately NOT gated like Jog — the dashboard's "start / home arm"
// button is meant to work any time, matching the wireframe). If a
// replay is in progress, Home will fight it; press Stop first.
func (a *Arm) Home() {
	target := [4]int16{HomeAngleDeg10, HomeAngleDeg10, HomeAngleDeg10, HomeAngleDeg10}

	a.mu.Lock()
	a.angles = target
	a.dirty = false // this send supersedes any pending jog flush
	a.mu.Unlock()

	if err := a.link.Send(protocol.MoveAllFrame(target, homeMoveTimeMs)); err != nil {
		log.Printf("[arm] home send failed: %v", err)
	}
}

func (a *Arm) EmergencyStop() {
	a.mu.Lock()
	a.replaying = false
	a.dirty = false
	if a.replayCancel != nil {
		a.replayCancel()
	}
	a.mu.Unlock()
	_ = a.link.Send(protocol.StopFrame())
}

// ClawSet: direction >0 closes, <0 opens, 0 stops — mirrors the xbox
// package's RT/LT convention. Internally mapped to the firmware's
// mode enum (0=stop 1=close 2=open).
//
// Debounced like Jog/SetJointDeg: the xbox bridge calls this on every
// UDP packet (continuously, ~50Hz, including the neutral/no-trigger
// case which used to resolve to ClawSet(0,0) every single tick), and
// the web dashboard's slider can fire many events per second too.
// Without a skip-if-unchanged check here, that unconditionally hits
// the wire far faster than the firmware's 16-deep command queue can
// drain (it shares that queue with jog moves), flooding it with
// redundant "still stopped"/"still closing" frames — which is what
// was actually behind the "command queue full" / "frame channel
// full" spam, not just contention with the debug CLI's demo mode.
func (a *Arm) ClawSet(direction int8, duty byte) {
	var mode byte
	switch {
	case direction > 0:
		mode = 1
	case direction < 0:
		mode = 2
	default:
		mode = 0
	}

	a.mu.Lock()
	if a.clawSent && mode == a.lastClawMode && duty == a.lastClawDuty {
		a.mu.Unlock()
		return
	}
	a.clawSent = true
	a.lastClawMode = mode
	a.lastClawDuty = duty
	a.mu.Unlock()

	if err := a.link.Send(protocol.ClawSetFrame(mode, duty)); err != nil {
		log.Printf("[arm] claw send failed: %v", err)
	}
}

// --- Stats / state polling ---------------------------------------------

// statePoller periodically requests CmdGetState and measures round-trip
// latency for the stats panel. Runs for the lifetime of the Arm. This
// is a health/latency check only — Arm's own a.angles (commanded, not
// sensed) remains the source of truth for "where is the arm", per the
// Option A design note at the top of this file.
func (a *Arm) statePoller() {
	ticker := time.NewTicker(1 * time.Second)
	defer ticker.Stop()

	for range ticker.C {
		sent := time.Now()
		if err := a.link.Send(protocol.GetStateFrame()); err != nil {
			a.mu.Lock()
			a.connected = false
			a.mu.Unlock()
			continue
		}

		select {
		case f := <-a.link.Frames():
			if f.Cmd != protocol.RspState {
				continue // some other frame arrived first; next tick will retry
			}
			if _, err := protocol.DecodeState(f); err != nil {
				continue
			}
			a.mu.Lock()
			a.connected = true
			a.latencyMs = int(time.Since(sent).Milliseconds())
			a.mu.Unlock()
		case <-time.After(500 * time.Millisecond):
			a.mu.Lock()
			a.connected = false
			a.mu.Unlock()
		}
	}
}

func (a *Arm) Stats() Stats {
	a.mu.Lock()
	defer a.mu.Unlock()
	var deg [4]float64
	for i, v := range a.angles {
		deg[i] = float64(v) / 10.0
	}
	return Stats{
		Mode:          a.mode,
		AnglesDeg:     deg,
		Connected:     a.connected,
		LatencyMs:     a.latencyMs,
		Recording:     a.recording,
		RecordedSteps: len(a.recBuf),
		Replaying:     a.replaying,
	}
}

func clamp16(v, lo, hi int16) int16 {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}

// MarshalSteps / UnmarshalSteps — JSON helpers used by store for the
// SQLite TEXT column.
func MarshalSteps(steps []Step) (string, error) {
	b, err := json.Marshal(steps)
	return string(b), err
}

func UnmarshalSteps(s string) ([]Step, error) {
	var steps []Step
	err := json.Unmarshal([]byte(s), &steps)
	return steps, err
}
