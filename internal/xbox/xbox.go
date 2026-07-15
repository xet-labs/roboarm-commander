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

// Packet mirrors tools/xbox_bridge.py's JSON output.
type Packet struct {
	LX, LY, RX, RY int     `json:"lx" `
	LT, RT         int     `json:"lt"`
	Buttons        uint16  `json:"buttons"`
}

const (
	btnA     = 1 << 0
	btnStart = 1 << 7
)

// deadzone matches the python bridge's raw-axis threshold (~128 of
// int16 range); tick scale controls how many degrees/sec a full stick
// deflection produces at the poll rate the bridge sends at (~60Hz from
// the original evdev script, throttle further in the bridge if jog feels
// too twitchy on hardware — easier to tune there than here).
const (
	deadzone  = 3000  // out of ±32767
	tickScale = 3     // degrees per packet at full stick deflection
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

func axisToDelta(v int) int {
	if v > -deadzone && v < deadzone {
		return 0
	}
	if v > 0 {
		return tickScale
	}
	return -tickScale
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
