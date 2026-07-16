# roboarm-commander

Go commander for the RPi Zero 2 W side. Owns the UART link to the ESP32,
arm state (mode/angles/recording/replay), SQLite-backed replay profiles,
and the web dashboard. Controller input arrives via `tools/xbox_bridge.py`
over loopback UDP (see `internal/xbox/xbox.go` for why it's split this way).

## Status: builds clean, vet clean, protocol tests pass

This was originally dropped in as "written from memory, not compile-
tested." It has since been built, vetted, and tested for real in this
sandbox (Go 1.22, installed via `archive.ubuntu.com`, deps fetched by
bypassing the module proxy — see `go.mod`'s `replace` directives). That
caught real bugs — see below. What's still genuinely unverified is
everything that needs actual hardware: a real ESP32 on the other end
of the UART, a real Xbox pad through `xbox_bridge.py`, SQLite file
permissions on the actual Pi filesystem.

```
go vet ./...              # clean
go build ./cmd/roboarm     # clean
go test ./...              # clean (protocol package has real tests)
```

## What was actually wrong, and fixed

This drop was written before `roboArm_controller`'s firmware protocol
was finalized, so it targeted a **different, incompatible wire format**
— it would have compiled, run, and simply never worked with the real
ESP32. Full list of what changed:

1. **Protocol mismatch (the big one).** The old `protocol.go` used a
   frame with a dedicated `JOINT_ID` byte, whole-degree `uint8` angles,
   no move-time parameter, and a different command numbering (`CmdStop`
   =0x03, `CmdGetState`=0x04, response=0x84 — which collides with the
   firmware's `RSP_ERR`). Firmware's actual format (`protocol.h`) has
   no separate joint-id field, uses `int16` deg10 angles, carries a
   `timeMs` on every move, and different command numbers throughout.
   `protocol.go` is now a byte-for-byte match — verified with a
   hand-computed exact-bytes test (`TestMoveJointExactBytes`), not just
   "should be compatible."
2. **Critical bug in `xbox.go`'s `Packet` struct**, caught by `go vet`,
   not by reading it: `LX, LY, RX, RY int \`json:"lx"\`` looks like four
   tagged fields but Go applies one shared tag to the whole comma-
   joined line, so all four fields collided on the JSON key `"lx"`.
   `encoding/json` refuses to populate ambiguous fields — every axis
   but possibly one would have silently stayed zero forever. No error,
   no panic. The arm would just never jog, with no clue why. Fixed by
   giving every field its own line and tag.
3. **Jog could flood the firmware's command queue.** The original
   `Jog()` sent a UART frame per changed joint on every single incoming
   Xbox UDP packet — at a controller's native report rate (100Hz+),
   that's up to 400 frames/sec against a 16-deep queue that drains at
   50/sec. `Jog()` now only updates in-memory target state; a separate
   `jogFlusher` goroutine sends one coalesced `MoveAll` frame at a
   fixed, safe cadence (25Hz).
4. **`Home()` only worked in Live mode**, because it went through the
   mode-gated `Jog()`. Per the wireframe, "start / home arm" is a
   bottom-row control meant to work regardless of mode (same as "stop
   arm"). Home now sends directly, mode-independent.
5. **Replay snapped between keyframes** — `MoveFrame` had no time
   parameter, so every recorded step would have commanded an
   effectively-instant move on firmware's mg995 joints (no timeMs =
   firmware treats it as "as fast as possible"). Replay now computes
   each step's move time from the gap to the next recorded timestamp,
   so joints glide between keyframes instead of jerking.
6. **Angle units** were whole degrees (0-180); firmware works in deg10
   (tenths of a degree, matching SC15's real resolution). All internal
   state is deg10 now; conversion happens only at the two boundaries
   that need human-readable numbers — the JSON `Stats` API (for the
   dashboard) and CSV import/export.
7. **`go.bug.st/serial`'s vanity import domain isn't reachable** from
   this sandbox's network egress allowlist. Added `replace` directives
   in `go.mod` pointing it (and two of its own test-only transitive
   deps, `golang.org/x/sys` and `gopkg.in/yaml.v3`) at their real
   GitHub homes — same code, different fetch path. Harmless to leave
   in when building somewhere with normal internet access.

## Build

```bash
go mod tidy      # pulls go.bug.st/serial + mattn/go-sqlite3
CGO_ENABLED=1 go build -o roboarm ./cmd/roboarm
```

`mattn/go-sqlite3` needs CGO + a C compiler (gcc). Standard on Raspberry
Pi OS; if cross-compiling from the desktop instead, either build
on-device or set up an ARM cross toolchain — don't burn Day-1 time on
cross-compilation, just build on the Pi directly.

If your build environment also can't reach `go.bug.st`, `golang.org`,
or `gopkg.in` (unlikely on real hardware, but matches this sandbox),
the `replace` directives already in `go.mod` route around it via
`GOPROXY=direct GOSUMDB=off go mod tidy`.

## Run

```bash
./roboarm --uart /dev/ttyUSB0 --baud 115200 --web :8080 --xbox 127.0.0.1:9999
python3 tools/xbox_bridge.py --host 127.0.0.1 --port 9999
```

Then open `http://<pi-ip>:8080`.

## Still needed (not in this drop)

- End-to-end test against the real `roboArm_controller` firmware — the
  protocol now matches on paper (and by hand-verified test bytes), but
  nothing beats plugging it into the actual ESP32.
- `CmdSetTorque`/`CmdPing` exist in the protocol and firmware but
  aren't called from anywhere in the Go side yet — not needed for
  Day-1 jog/record/replay, only for future hybrid drag-teach or a
  dedicated latency stat separate from the 1Hz `GetState` poll.
- systemd units to run the Go binary + Python bridge on boot, if you
  want it to survive a reboot during the demo without manual restart.
