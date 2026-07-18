#!/usr/bin/env python3
"""
Xbox controller -> Go commander bridge.

Adapted from the project's existing ps3.uart.ctrl.py. Same device
discovery / deadzone / D-pad handling (already proven working) — the
only change from that original is the output: instead of writing a raw
framed packet to the ESP32's serial port directly, this sends JSON
over loopback UDP to the Go commander, which now owns the UART link
and all arm logic.

Run this on the RPi alongside the Go binary:
    ./roboarm-commander &
    python3 xbox_bridge.py

--- Why the arm used to stutter / stop moving while the stick was held ---
evdev only emits an axis event (and the EV_SYN that follows it) when a
value CHANGES. The previous version of this bridge only sent a UDP
packet from inside the EV_SYN branch — so holding the stick at a
constant deflection produced exactly one event, then silence: no more
packets reached the Go commander, so arm.Jog() was never called again
and the joint stopped mid-move until the stick jittered enough to fire
another event. That's exactly the "stutters, doesn't move smoothly"
symptom.

Fix: a background thread sends the CURRENT state at a fixed rate
(--rate Hz, default 50) regardless of whether evdev produced a new
event. The evdev read loop's only job now is to keep `state` updated;
sending is entirely decoupled from that. (Go's arm.jogFlusher already
paces actual wire sends at its own fixed cadence and coalesces bursts,
so sending state at 50Hz here is not "too fast" — it just means the
Go side always has a fresh target to work from.)
"""
import argparse
import json
import re
import socket
import threading
import time
from datetime import datetime

from evdev import InputDevice, ecodes, list_devices

parser = argparse.ArgumentParser()
parser.add_argument("--host", default="127.0.0.1")
parser.add_argument("--port", type=int, default=9999)
parser.add_argument("--rate", type=float, default=50.0,
                     help="UDP send rate in Hz — packets go out continuously "
                          "at this rate, not just when the stick moves (default: 50)")
parser.add_argument("--quiet", action="store_true",
                     help="suppress the per-event console printout")
args = parser.parse_args()

devices = [InputDevice(path) for path in list_devices()]
controller = None
for dev in devices:
    if re.search(r"x[- ]?box(?:360)?", dev.name, re.IGNORECASE):
        controller = dev
        break

if controller is None:
    print("Xbox controller not found!")
    raise SystemExit(1)

print("Controller:", controller.name, controller.path)
print(f"Forwarding to {args.host}:{args.port} at {args.rate:.0f}Hz")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
dest = (args.host, args.port)

state = {"lx": 0, "ly": 0, "rx": 0, "ry": 0, "lt": 0, "rt": 0, "buttons": 0}
state_lock = threading.Lock()
running = True

button_map = {
    ecodes.BTN_SOUTH: 0, ecodes.BTN_EAST: 1, ecodes.BTN_NORTH: 2, ecodes.BTN_WEST: 3,
    ecodes.BTN_TL: 4, ecodes.BTN_TR: 5, ecodes.BTN_SELECT: 6, ecodes.BTN_START: 7,
    ecodes.BTN_THUMBL: 8, ecodes.BTN_THUMBR: 9, ecodes.BTN_MODE: 14,
}
axis_map = {
    ecodes.ABS_X: "lx", ecodes.ABS_Y: "ly",
    ecodes.ABS_RX: "rx", ecodes.ABS_RY: "ry",
    ecodes.ABS_Z: "lt", ecodes.ABS_RZ: "rt",
}

# Bit -> human-readable name, for the console printout (matches the old
# ps3.uart.ctrl.py script's button_names table).
button_names = {
    0: "A", 1: "B", 2: "X", 3: "Y",
    4: "LB", 5: "RB", 6: "Back", 7: "Start",
    8: "LStick", 9: "RStick", 10: "DPadUp", 11: "DPadDown",
    12: "DPadLeft", 13: "DPadRight",
}


def update_dpad(value, code):
    if code == ecodes.ABS_HAT0X:
        if value == -1:
            state["buttons"] |= (1 << 12); state["buttons"] &= ~(1 << 13)
        elif value == 1:
            state["buttons"] |= (1 << 13); state["buttons"] &= ~(1 << 12)
        else:
            state["buttons"] &= ~((1 << 12) | (1 << 13))
    elif code == ecodes.ABS_HAT0Y:
        if value == -1:
            state["buttons"] |= (1 << 10); state["buttons"] &= ~(1 << 11)
        elif value == 1:
            state["buttons"] |= (1 << 11); state["buttons"] &= ~(1 << 10)
        else:
            state["buttons"] &= ~((1 << 10) | (1 << 11))


def print_state():
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-2]
    pressed = [name for bit, name in button_names.items() if state["buttons"] & (1 << bit)]
    print("{}   LX: {:06d}   LY: {:06d}   RX: {:06d}   RY: {:06d}   LT: {:03d}   RT: {:03d}   Buttons: 0x{:04X} ({})".format(
        ts, state["lx"], state["ly"], state["rx"], state["ry"], state["lt"], state["rt"],
        state["buttons"], ", ".join(pressed) if pressed else "None"
    ))


def sender_loop():
    """Runs on its own thread. Sends the current state at a fixed rate
    no matter what the evdev read loop is doing — this is what keeps
    the arm moving smoothly while the stick is held at a constant
    deflection, instead of only reacting to value-change events."""
    period = 1.0 / args.rate
    while running:
        with state_lock:
            packet = dict(state)
        try:
            sock.sendto(json.dumps(packet).encode(), dest)
        except OSError as e:
            print(f"[WARN] UDP send failed: {e}")
        time.sleep(period)


sender_thread = threading.Thread(target=sender_loop, daemon=True)
sender_thread.start()

try:
    for event in controller.read_loop():
        if event.type == ecodes.EV_KEY:
            if event.code in button_map:
                bit = button_map[event.code]
                with state_lock:
                    if event.value == 1:
                        state["buttons"] |= (1 << bit)
                    elif event.value == 0:
                        state["buttons"] &= ~(1 << bit)
            continue

        elif event.type == ecodes.EV_ABS:
            with state_lock:
                if event.code in axis_map:
                    val = event.value
                    if event.code in (ecodes.ABS_X, ecodes.ABS_Y, ecodes.ABS_RX, ecodes.ABS_RY):
                        if abs(val) < 128:
                            val = 0
                    state[axis_map[event.code]] = val
                elif event.code in (ecodes.ABS_HAT0X, ecodes.ABS_HAT0Y):
                    update_dpad(event.value, event.code)
            continue

        elif event.type == ecodes.EV_SYN:
            # Sending itself now happens continuously on sender_loop —
            # this branch is just the console printout, matching the
            # old script's "print once per input report" behavior.
            if not args.quiet:
                print_state()

except KeyboardInterrupt:
    print("\nStopping...")
finally:
    running = False
    sender_thread.join(timeout=1.0)
