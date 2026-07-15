#!/usr/bin/env python3
"""
Xbox controller -> Go commander bridge.

Adapted from the project's existing ps3.uart.ctrl.py. Same device
discovery / deadzone / D-pad handling (already proven working) — the
only change is the output: instead of writing a raw framed packet to
the ESP32's serial port directly, this sends JSON over loopback UDP to
the Go commander, which now owns the UART link and all arm logic.

Run this on the RPi alongside the Go binary:
    ./roboarm-commander &
    python3 xbox_bridge.py
"""
import argparse
import json
import re
import socket
from evdev import InputDevice, ecodes, list_devices

parser = argparse.ArgumentParser()
parser.add_argument("--host", default="127.0.0.1")
parser.add_argument("--port", type=int, default=9999)
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
print(f"Forwarding to {args.host}:{args.port}")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
dest = (args.host, args.port)

state = {"lx": 0, "ly": 0, "rx": 0, "ry": 0, "lt": 0, "rt": 0, "buttons": 0}

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

try:
    for event in controller.read_loop():
        if event.type == ecodes.EV_KEY:
            if event.code in button_map:
                bit = button_map[event.code]
                if event.value == 1:
                    state["buttons"] |= (1 << bit)
                elif event.value == 0:
                    state["buttons"] &= ~(1 << bit)
            continue

        elif event.type == ecodes.EV_ABS:
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
            sock.sendto(json.dumps(state).encode(), dest)

except KeyboardInterrupt:
    print("\nStopping...")
