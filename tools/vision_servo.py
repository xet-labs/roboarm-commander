#!/usr/bin/env python3
"""
Red-object visual servoing -> Go commander bridge ("AI centers the
object in frame").

Camera is mounted on the END EFFECTOR, looking outward. This does NOT
use inverse kinematics -- centering an object in a 2D image is a pixel-
space feedback problem, not a 3D positioning one, so no link lengths,
joint offsets, or base elevation are needed. This is the classic
"Image-Based Visual Servoing" (IBVS) approach:

    every frame:
        find the red blob's center pixel
        compare it to the frame's center -> (error_x, error_y)
        nudge Base by  -Kp * error_x   (pan, left/right)
        nudge Wrist by -Kp * error_y   (tilt, up/down)
        repeat -- as the arm turns, error shrinks toward zero and the
        nudges shrink with it, so it naturally settles once centered

Sends relative deltas to the Go commander's existing /api/jog endpoint
(internal/web/server.go's handleJog) -- built for exactly this kind of
continuous relative nudge, same as the xbox bridge's stick deltas.
No changes needed to the Go server or ESP32 firmware.

One-time setup:
    pip install opencv-python requests

Before running: put the dashboard in Live mode (Jog is a no-op
outside Live mode) -- either click Live in the web UI, or:
    curl -X POST localhost:8080/api/mode -d '{"mode":"live"}'

Run:
    ./roboarm-commander &
    python3 vision_servo.py --camera tcp://<pi-ip>:5000 --auto-calibrate

When no red object is visible, the arm now HOLDS STILL and waits --
it no longer sweeps searching for the object. If you want scanning
back, that's a separate feature; ask for it explicitly.

--auto-calibrate (recommended, run once per camera/arm mounting):
    On startup, sends small known test nudges on Base and Wrist and
    watches which way the object's pixel position actually moves.
    From that it works out the correct sign for --invert-pan /
    --invert-wrist itself, instead of you guessing and toggling flags
    by hand. This is the real fix for "the arm moves the wrong way /
    the error gets WORSE instead of better" -- that symptom is a sign
    error (positive feedback: every correction pushes further off
    target instead of closer), not a tuning problem, and no amount of
    --kp or --deadzone-px adjustment fixes a sign error.

Tuning knobs, all as CLI flags: --kp (how aggressively it reacts),
--deadzone-px (how close to center counts as "done", kills jitter),
--max-step-deg (safety cap on how far a single nudge can move a
joint), --invert-pan / --invert-wrist (manual override if you don't
want to run --auto-calibrate), --proc-scale (downscale factor for
detection, cheaper = faster loop), --smoothing (0..1 EMA factor on
the detected centroid, kills single-frame jitter).
"""
import argparse
import threading
import time
from collections import deque

import cv2
import numpy as np
import requests

parser = argparse.ArgumentParser()
parser.add_argument("--host", default="127.0.0.1")
parser.add_argument("--port", type=int, default=8080)
parser.add_argument("--camera", default="tcp://127.0.0.1:5000",
                     help="video source: a local webcam index (e.g. '0'), or a "
                          "network stream URL, e.g. tcp://<pi-ip>:5000 to match "
                          "'rpicam-vid ... -o tcp://0.0.0.0:5000 -l'")
parser.add_argument("--rate", type=float, default=15.0,
                     help="control loop rate, Hz (default: 15)")
parser.add_argument("--kp", type=float, default=8.0,
                     help="proportional gain: deg10 of nudge per unit of "
                          "normalized pixel error (-1..1) (default: 8.0)")
parser.add_argument("--deadzone-px", type=float, default=20.0,
                     help="ignore error smaller than this many pixels from "
                          "center (in PROCESSING-resolution pixels) -- prevents "
                          "jitter once centered (default: 20)")
parser.add_argument("--max-step-deg", type=float, default=3.0,
                     help="hard cap on degrees moved per single nudge, "
                          "independent of gain -- safety limit against a "
                          "bad frame causing a big jump (default: 3.0)")
parser.add_argument("--invert-pan", action="store_true",
                     help="flip Base direction (ignored if --auto-calibrate succeeds)")
parser.add_argument("--invert-wrist", action="store_true",
                     help="flip Wrist direction (ignored if --auto-calibrate succeeds)")
parser.add_argument("--auto-calibrate", action="store_true",
                     help="on startup, determine --invert-pan/--invert-wrist "
                          "automatically by nudging each axis and watching "
                          "which way the object moves in frame")
parser.add_argument("--proc-scale", type=float, default=0.5,
                     help="downscale factor applied before HSV/contour "
                          "processing -- 0.5 means detection runs on a "
                          "quarter the pixels. Lower = faster loop, less "
                          "precise centroid. (default: 0.5)")
parser.add_argument("--smoothing", type=float, default=0.5,
                     help="EMA smoothing factor on detected centroid, 0=no "
                          "smoothing (raw, jittery), close to 1=heavy "
                          "smoothing (laggy). (default: 0.5)")
parser.add_argument("--headless", action="store_true",
                     help="skip cv2.imshow entirely -- saves real per-frame "
                          "cost, use once things are working and you no "
                          "longer need the live debug view")
parser.add_argument("--axis", choices=["both", "pan", "tilt"], default="both",
                     help="which axis to actively correct -- 'pan' tracks "
                          "horizontal only (Base), 'tilt' vertical only "
                          "(Wrist), 'both' does both (default: both)")
parser.add_argument("--min-area", type=float, default=80.0,
                     help="minimum contour area (in PROCESSING-resolution "
                          "pixels^2) to count as a detection, not noise "
                          "(default: 80 -- lower than before since a small "
                          "box downscaled needs a lower floor)")
args = parser.parse_args()

jog_url = f"http://{args.host}:{args.port}/api/jog"
stats_url = f"http://{args.host}:{args.port}/api/stats"
session = requests.Session()  # reuse the TCP connection instead of a new
                               # handshake per request -- shaves real
                               # round-trip latency off every single nudge

# Red in HSV wraps around 0/180, so red needs two ranges combined,
# unlike most other colors which are one contiguous hue band.
LOWER_RED_1 = np.array([0, 120, 70])
UPPER_RED_1 = np.array([10, 255, 255])
LOWER_RED_2 = np.array([170, 120, 70])
UPPER_RED_2 = np.array([180, 255, 255])

MAX_STEP_DEG10 = int(args.max_step_deg * 10)

# Low-latency flags for the FFmpeg backend: skip its internal buffering
# so we always decode the newest packet available instead of queuing
# frames up.
import os
os.environ.setdefault(
    "OPENCV_FFMPEG_CAPTURE_OPTIONS",
    "fflags;nobuffer|flags;low_delay|max_delay;0|reorder_queue_size;0"
)


class FreshFrameReader:
    """Runs cap.read() continuously on a background thread instead of
    blocking the control loop on decode speed. The control loop just
    grabs whatever's newest each tick. Without this, if decode ever
    takes longer than one control period, frames pile up in whatever
    internal buffer OpenCV/FFmpeg keeps, and the control loop is
    always looking at stale data -- which is exactly the "horrible
    lag" symptom, independent of resolution."""

    def __init__(self, cap):
        self.cap = cap
        self._lock = threading.Lock()
        self._frame = None
        self._ok = False
        self._stopped = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stopped:
            ok, frame = self.cap.read()
            with self._lock:
                self._ok, self._frame = ok, frame
            if not ok:
                break

    def read(self):
        with self._lock:
            return self._ok, self._frame

    def stop(self):
        self._stopped = True
        self._thread.join(timeout=1.0)


if args.camera.isdigit():
    cap = cv2.VideoCapture(int(args.camera))  # local webcam
else:
    cap = cv2.VideoCapture(args.camera, cv2.CAP_FFMPEG)  # network stream URL
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

if not cap.isOpened():
    print(f"Could not open video source: {args.camera}")
    print("Confirm rpicam-vid is running on the Pi and pointed at this "
          "machine (for TCP: the Pi listens, so --camera should be "
          "tcp://<pi-ip>:5000 -- you connect TO the Pi, same direction "
          "ffplay uses). Try 'ffplay tcp://<pi-ip>:5000' first to confirm "
          "the stream itself is fine before blaming this script.")
    raise SystemExit(1)

reader = FreshFrameReader(cap)

print(f"Servoing toward red object, sending jog deltas to {jog_url}")
print("Press 'q' in the video window to quit." if not args.headless else
      "Running headless -- Ctrl+C to quit.")

period = 1.0 / args.rate


def find_red_centroid(frame_bgr):
    """Runs on a (possibly downscaled) frame. Returns centroid in THAT
    frame's pixel coordinates -- caller is responsible for knowing
    what coordinate space it's in."""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_RED_1, UPPER_RED_1) | cv2.inRange(hsv, LOWER_RED_2, UPPER_RED_2)
    mask = cv2.erode(mask, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, mask

    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < args.min_area:
        return None, mask

    M = cv2.moments(largest)
    cx = M["m10"] / M["m00"]
    cy = M["m01"] / M["m00"]
    return (cx, cy), mask


def send_jog(delta_base_deg10: int, delta_wrist_deg10: int) -> None:
    try:
        session.post(jog_url, json={
            "Base": delta_base_deg10, "Shoulder": 0,
            "Elbow": 0, "Wrist": delta_wrist_deg10,
        }, timeout=0.5)
    except requests.RequestException as e:
        print(f"[WARN] jog send failed: {e}")


def get_frame_for_calibration(timeout_s=2.0):
    """Grabs a frame + detects centroid, used only during calibration."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        ok, frame = reader.read()
        if ok and frame is not None:
            small = cv2.resize(frame, None, fx=args.proc_scale, fy=args.proc_scale,
                                interpolation=cv2.INTER_NEAREST)
            centroid, _ = find_red_centroid(small)
            if centroid is not None:
                return centroid
        time.sleep(0.05)
    return None


def auto_calibrate():
    """Sends a known test nudge on each axis, observes which way the
    object's pixel position actually moves, and derives the correct
    invert flag automatically. This directly fixes the "corrections
    make it worse instead of better" problem: that symptom is a sign
    error between "which way I nudged" and "which way the pixel
    should move to counter it," and no gain/deadzone tuning can fix a
    sign error -- only flipping the sign does."""
    print("[calib] looking for the object to use as a reference point...")
    before = get_frame_for_calibration()
    if before is None:
        print("[calib] no red object visible -- skipping auto-calibrate, "
              "using --invert-pan/--invert-wrist flags as given.")
        return args.invert_pan, args.invert_wrist

    test_deg10 = 150  # 15 degrees, big enough to see clearly, small enough to be safe
    invert_pan, invert_wrist = args.invert_pan, args.invert_wrist

    print("[calib] testing Base axis...")
    send_jog(test_deg10, 0)
    time.sleep(0.6)
    after = get_frame_for_calibration()
    send_jog(-test_deg10, 0)  # move back
    time.sleep(0.6)
    if after is not None:
        dx = after[0] - before[0]
        # We sent a POSITIVE Base nudge and observed dx (object's pixel
        # x-movement). The control loop's formula sends
        # delta_base = +kp*norm_x when the object is right of center
        # (norm_x > 0), intending that nudge to pull it back toward
        # center (dx should then be NEGATIVE, back toward 0). So a
        # correctly-signed system means: positive nudge -> negative dx.
        # If instead positive nudge -> positive dx, the axis is
        # inverted relative to what the formula assumes.
        invert_pan = dx > 0
        print(f"[calib] Base: sent +{test_deg10/10}deg, object moved dx={dx:+.1f}px "
              f"-> invert_pan={invert_pan}")
    else:
        print("[calib] lost the object during Base test, keeping given flag")

    print("[calib] testing Wrist axis...")
    send_jog(0, test_deg10)
    time.sleep(0.6)
    after = get_frame_for_calibration()
    send_jog(0, -test_deg10)
    time.sleep(0.6)
    if after is not None:
        dy = after[1] - before[1]
        invert_wrist = dy > 0
        print(f"[calib] Wrist: sent +{test_deg10/10}deg, object moved dy={dy:+.1f}px "
              f"-> invert_wrist={invert_wrist}")
    else:
        print("[calib] lost the object during Wrist test, keeping given flag")

    print(f"[calib] done. invert_pan={invert_pan}, invert_wrist={invert_wrist}")
    return invert_pan, invert_wrist


if args.auto_calibrate:
    invert_pan, invert_wrist = auto_calibrate()
else:
    invert_pan, invert_wrist = args.invert_pan, args.invert_wrist

# EMA-smoothed centroid state (None until first detection)
smoothed_cx, smoothed_cy = None, None

# Divergence watchdog: tracks recent error magnitude. If it's been
# growing steadily over several frames DESPITE corrections being sent,
# that's the signature of a sign error slipping past calibration (e.g.
# large-angle cross-axis coupling) -- warn instead of silently
# oscillating or walking the object off screen.
error_history = deque(maxlen=10)
warned_divergence = False

while True:
    loop_start = time.monotonic()
    ok, frame = reader.read()
    if not ok or frame is None:
        time.sleep(0.01)
        continue

    proc = cv2.resize(frame, None, fx=args.proc_scale, fy=args.proc_scale,
                       interpolation=cv2.INTER_NEAREST)
    h, w = proc.shape[:2]
    center_x, center_y = w / 2.0, h / 2.0

    centroid, mask = find_red_centroid(proc)

    if centroid is not None:
        cx, cy = centroid
        if smoothed_cx is None:
            smoothed_cx, smoothed_cy = cx, cy
        else:
            a = args.smoothing
            smoothed_cx = a * smoothed_cx + (1 - a) * cx
            smoothed_cy = a * smoothed_cy + (1 - a) * cy

        error_x = smoothed_cx - center_x
        error_y = smoothed_cy - center_y

        if not args.headless:
            disp_cx = int(smoothed_cx / args.proc_scale)
            disp_cy = int(smoothed_cy / args.proc_scale)
            cv2.circle(frame, (disp_cx, disp_cy), 8, (0, 255, 0), -1)

        norm_x_raw = error_x / center_x
        norm_y_raw = error_y / center_y

        norm_x = 0.0 if abs(error_x) < args.deadzone_px else norm_x_raw
        norm_y = 0.0 if abs(error_y) < args.deadzone_px else norm_y_raw

        pan_sign = -1 if invert_pan else 1
        tilt_sign = -1 if invert_wrist else 1

        delta_base = int(pan_sign * args.kp * norm_x * 10) if args.axis != "tilt" else 0
        delta_wrist = int(tilt_sign * args.kp * norm_y * 10) if args.axis != "pan" else 0

        delta_base = max(-MAX_STEP_DEG10, min(MAX_STEP_DEG10, delta_base))
        delta_wrist = max(-MAX_STEP_DEG10, min(MAX_STEP_DEG10, delta_wrist))

        if delta_base != 0 or delta_wrist != 0:
            send_jog(delta_base, delta_wrist)

        # Divergence watchdog
        err_mag = (norm_x_raw ** 2 + norm_y_raw ** 2) ** 0.5
        error_history.append(err_mag)
        if len(error_history) == error_history.maxlen:
            first_half = sum(list(error_history)[:5]) / 5
            second_half = sum(list(error_history)[5:]) / 5
            if second_half > first_half * 1.3 and second_half > 0.15 and not warned_divergence:
                print("\n[WARN] error is growing despite corrections -- this "
                      "usually means a sign error (wrong invert-pan/"
                      "invert-wrist) or gain too high causing overshoot. "
                      "Try --auto-calibrate if you haven't, or lower --kp.")
                warned_divergence = True
            elif second_half < first_half:
                warned_divergence = False  # recovered, allow re-warning later

        status = f"error=({error_x:+.0f},{error_y:+.0f})px  nudge=({delta_base/10:+.1f},{delta_wrist/10:+.1f})deg"
    else:
        # Nothing to track -- HOLD STILL. Don't send any jog, don't
        # scan. Smoothing state resets so the next detection isn't
        # blended against a stale position.
        smoothed_cx, smoothed_cy = None, None
        error_history.clear()
        status = "no red object -- holding still"

    if not args.headless:
        cv2.drawMarker(frame, (int(frame.shape[1] / 2), int(frame.shape[0] / 2)),
                        (255, 255, 0), markerType=cv2.MARKER_CROSS,
                        markerSize=20, thickness=1)
        cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 255), 2)
        cv2.imshow("vision_servo (q to quit)", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        print(status, end="\r")

    elapsed = time.monotonic() - loop_start
    if elapsed < period:
        time.sleep(period - elapsed)

reader.stop()
cap.release()
if not args.headless:
    cv2.destroyAllWindows()
