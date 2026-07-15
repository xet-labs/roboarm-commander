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
package arm

import (
	"context"
	"encoding/json"
	"fmt"
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

// Step is one recorded (or replayed) keyframe.
type Step struct {
	TMs    int64  `json:"t_ms"` // milliseconds since recording start
	Angles [4]int `json:"angles"`
}

const (
	AngleMin = 0
	AngleMax = 180
	HomeAngle = 90

	recordIntervalMs = 100 // teach-mode sample rate; matches original dataset's implied cadence
)

type Stats struct {
	Mode          Mode   `json:"mode"`
	Angles        [4]int `json:"angles"`
	Connected     bool   `json:"connected"`
	LatencyMs     int    `json:"latency_ms"`
	Recording     bool   `json:"recording"`
	RecordedSteps int    `json:"recorded_steps"`
	Replaying     bool   `json:"replaying"`
}

type Arm struct {
	mu sync.Mutex

	link *uart.Link
	mode Mode

	angles [4]int

	recording bool
	recStart  time.Time
	recBuf    []Step
	lastRec   time.Time

	replayCancel context.CancelFunc
	replaying    bool

	connected bool
	latencyMs int
}

func New(link *uart.Link) *Arm {
	a := &Arm{
		link:   link,
		mode:   ModeIdle,
		angles: [4]int{HomeAngle, HomeAngle, HomeAngle, HomeAngle},
	}
	go a.statePoller()
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

// Jog applies a signed delta (degrees) to each joint and sends only the
// joints that actually changed. Called from the xbox bridge listener at
// whatever cadence the pad reports state (see internal/xbox).
func (a *Arm) Jog(deltaBase, deltaShoulder, deltaElbow, deltaWrist int) {
	a.mu.Lock()
	if a.mode != ModeLive {
		a.mu.Unlock()
		return
	}
	deltas := [4]int{deltaBase, deltaShoulder, deltaElbow, deltaWrist}
	jointIDs := [4]byte{protocol.JointBase, protocol.JointShoulder, protocol.JointElbow, protocol.JointWrist}

	changed := false
	for i, d := range deltas {
		if d == 0 {
			continue
		}
		next := clamp(a.angles[i]+d, AngleMin, AngleMax)
		if next == a.angles[i] {
			continue
		}
		a.angles[i] = next
		changed = true
		if err := a.link.Send(protocol.MoveFrame(jointIDs[i], next)); err != nil {
			// non-fatal: log and continue, connection state reflected via statePoller
			fmt.Printf("[arm] jog send failed joint=%d: %v\n", i, err)
		}
	}
	a.mu.Unlock()

	if changed {
		a.maybeRecord()
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
	jointIDs := [4]byte{protocol.JointBase, protocol.JointShoulder, protocol.JointElbow, protocol.JointWrist}

	for _, step := range steps {
		target := start.Add(time.Duration(step.TMs) * time.Millisecond)
		wait := time.Until(target)
		if wait > 0 {
			select {
			case <-ctx.Done():
				return
			case <-time.After(wait):
			}
		}

		a.mu.Lock()
		for i, ang := range step.Angles {
			if a.angles[i] == ang {
				continue
			}
			a.angles[i] = ang
			_ = a.link.Send(protocol.MoveFrame(jointIDs[i], ang))
		}
		a.mu.Unlock()
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

func (a *Arm) Home() {
	a.Jog(
		HomeAngle-a.angleOf(0),
		HomeAngle-a.angleOf(1),
		HomeAngle-a.angleOf(2),
		HomeAngle-a.angleOf(3),
	)
}

func (a *Arm) angleOf(i int) int {
	a.mu.Lock()
	defer a.mu.Unlock()
	return a.angles[i]
}

func (a *Arm) EmergencyStop() {
	a.mu.Lock()
	a.replaying = false
	if a.replayCancel != nil {
		a.replayCancel()
	}
	a.mu.Unlock()
	_ = a.link.Send(protocol.StopFrame())
}

func (a *Arm) ClawSet(direction int8, duty byte) {
	_ = a.link.Send(protocol.ClawFrame(direction, duty))
}

// --- Stats / state polling ---------------------------------------------

// statePoller periodically requests CmdGetState and measures round-trip
// latency for the stats panel. Runs for the lifetime of the Arm.
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
			if f.Cmd != protocol.CmdState {
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
	return Stats{
		Mode:          a.mode,
		Angles:        a.angles,
		Connected:     a.connected,
		LatencyMs:     a.latencyMs,
		Recording:     a.recording,
		RecordedSteps: len(a.recBuf),
		Replaying:     a.replaying,
	}
}

func clamp(v, lo, hi int) int {
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
