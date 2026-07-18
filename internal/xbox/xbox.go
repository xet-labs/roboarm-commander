// Package xbox receives normalized controller state from the Python
// evdev bridge (tools/xbox_bridge.py) over loopback UDP and resolves it
// into joint jog deltas for arm.Jog.
//
// Why not read evdev directly in Go: the existing ps3.uart.ctrl.py
// already does correct device discovery, deadzoning and D-pad handling.
// Re-implementing that in Go means matching the platform's input_event
// struct layout exactly (varies with 32/64-bit userspace), which is a
// real way to burn a day chasing a struct-packing bug with no easy way
// to verify without your actual hardware. Not worth it under this
// deadline — the bridge does one job (read pad, emit JSON) and does it
// with code that's already proven working.
//
// Axis-to-joint mapping (documented here since it's the one thing that's
// arbitrary — remap freely, this is the only place it lives):
//
//	left stick  X -> base    (left/right swing)
//	left stick  Y -> shoulder (up/down reach)
//	right stick Y -> elbow    (up/down)
//	right stick X -> wrist    (rotate)
//	LT/RT              -> claw open/close (handled by button/trigger, not jog)
package xbox

import (
	"encoding/json"
	"log"
	"net"

	"github.com/xet-labs/roboarm-commander/internal/arm"
)

// Packet mirrors tools/xbox_bridge.py's JSON output. Each field needs
// its OWN tag on its own line — `LX, LY, RX, RY int `json:"lx"“ looks
// like it works but silently applies "lx" to all four fields, which
// makes them ambiguous JSON keys that encoding/json refuses to
// populate at all. Caught by `go vet`, not by inspection — this would
// have meant every axis but possibly one reading zero forever, with no
// error, no panic, just an arm that doesn't jog. Worth remembering:
// this exact shape (comma-joined field list + single tag) is worth a
// second look anywhere else it appears.
type Packet struct {
	LX      int    `json:"lx"`
	LY      int    `json:"ly"`
	RX      int    `json:"rx"`
	RY      int    `json:"ry"`
	LT      int    `json:"lt"`
	RT      int    `json:"rt"`
	Buttons uint16 `json:"buttons"`
}

const (
	btnA     = 1 << 0
	btnStart = 1 << 7
)

// deadzone matches the python bridge's raw-axis threshold (~128 of
// int16 range). tickScaleMin/Max are in deg10 (tenths of a degree) per
// UDP packet — arm.Jog only updates in-memory target state per call
// now (actual wire sends are paced separately by arm's jogFlusher), so
// this just controls how fast the target angle accumulates per packet
// while the stick is held, not wire traffic. The bridge now sends
// continuously at a fixed rate (tools/xbox_bridge.py, --rate, default
// 50Hz) rather than only on value-change events, so scaling this by
// how far the stick is pushed gives proportional speed control instead
// of every jog moving at one fixed rate. First-pass guess — tune on
// hardware, these are the constants most likely to need adjusting for
// jog to feel right.
const (
	deadzone      = 3000 // out of ±32767
	tickScaleMin  = 3    // deg10 per packet just past the deadzone (= 0.3 deg)
	tickScaleMax  = 20   // deg10 per packet at full stick deflection (= 2.0 deg)
	axisFullScale = 32767
)

func Listen(addr string, a *arm.Arm) error {
	conn, err := net.ListenPacket("udp", addr)
	if err != nil {
		return err
	}
	log.Printf("[xbox] listening for controller bridge on %s", addr)

	buf := make([]byte, 512)
	var lastStart bool

	go func() {
		defer conn.Close()
		for {
			n, _, err := conn.ReadFrom(buf)
			if err != nil {
				log.Printf("[xbox] read error: %v", err)
				continue
			}

			var p Packet
			if err := json.Unmarshal(buf[:n], &p); err != nil {
				log.Printf("[xbox] bad packet: %v", err)
				continue
			}

			// edge-triggered home-on-Start, mirrors old firmware's
			// demo-mode toggle pattern but repurposed for homing
			curStart := p.Buttons&btnStart != 0
			if curStart && !lastStart {
				a.Home()
			}
			lastStart = curStart

			dBase := axisToDelta(p.LX)
			dShoulder := axisToDelta(-p.LY) // stick-up (negative Y) = arm up
			dElbow := axisToDelta(-p.RY)
			dWrist := axisToDelta(p.RX)

			a.Jog(dBase, dShoulder, dElbow, dWrist)

			// claw: RT closes, LT opens, whichever is pressed harder
			if p.RT > 20 || p.LT > 20 {
				if p.RT >= p.LT {
					a.ClawSet(1, byte(clampInt(p.RT, 0, 255)))
				} else {
					a.ClawSet(-1, byte(clampInt(p.LT, 0, 255)))
				}
			} else {
				a.ClawSet(0, 0)
			}
		}
	}()

	return nil
}

func axisToDelta(v int) int16 {
	mag := v
	if mag < 0 {
		mag = -mag
	}
	if mag < deadzone {
		return 0
	}

	// Linear ramp from tickScaleMin (just past deadzone) to tickScaleMax
	// (full deflection), so how hard the stick is pushed controls jog speed.
	span := axisFullScale - deadzone
	scaled := tickScaleMin + (tickScaleMax-tickScaleMin)*(mag-deadzone)/span
	if scaled > tickScaleMax {
		scaled = tickScaleMax
	}

	if v < 0 {
		return -int16(scaled)
	}
	return int16(scaled)
}

func clampInt(v, lo, hi int) int {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}
