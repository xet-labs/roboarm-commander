# roboarm-commander

Go commander for the RPi Zero 2 W side. Owns the UART link to the ESP32,
arm state (mode/angles/recording/replay), SQLite-backed replay profiles,
and the web dashboard. Controller input arrives via `tools/xbox_bridge.py`
over loopback UDP (see `internal/xbox/xbox.go` for why it's split this way).

## IMPORTANT — not compile-tested

This sandbox has no route to the Go module proxy, so none of this has
been `go build`-verified. Package APIs (`go.bug.st/serial`, `mattn/go-sqlite3`)
are written from memory of their stable, well-documented surfaces, but
run `go build ./...` on real hardware before trusting it — fix compile
errors first, they'll be shallow (import paths, minor signature drift),
not logic errors.

## Build

```bash
go mod tidy      # pulls go.bug.st/serial + mattn/go-sqlite3
CGO_ENABLED=1 go build -o roboarm ./cmd/roboarm
```

`mattn/go-sqlite3` needs CGO + a C compiler (gcc). That's standard on
Raspberry Pi OS; if cross-compiling from the desktop instead, either
build on-device or set up an ARM cross toolchain — don't burn Day-1 time
on cross-compilation, just build on the Pi directly.

## Run

```bash
./roboarm --uart /dev/ttyUSB0 --baud 115200 --web :8080 --xbox 127.0.0.1:9999
python3 tools/xbox_bridge.py --host 127.0.0.1 --port 9999
```

Then open `http://<pi-ip>:8080`.

## Still needed (not in this drop)

- ESP32-side UART frame parser matching `internal/protocol/protocol.go`'s
  documented format — that's the firmware-side counterpart, next piece
  to write.
- INA219 current -> claw force-limit closed loop (currently `clawSet` in
  firmware is open-loop; Go's `/api/claw` just passes direction+duty
  through).
- systemd units to run the Go binary + Python bridge on boot, if you
  want it to survive a reboot during the demo without manual restart.
