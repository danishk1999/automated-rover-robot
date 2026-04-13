#!/usr/bin/env python3
"""
rover_dashboard.py — Autonomous Delivery Rover Dashboard
=========================================================
Hardware:  Raspberry Pi 3B + Arduino Uno (via USB serial)
           + Slamtec RPLIDAR A1 (via USB)
           + USB Camera (640×480)
           + HC-SR04 via Arduino

Run:  python3 rover_dashboard.py
Open: http://<pi-ip>:5000

Threading model
───────────────
  Main        → ThreadedHTTPServer (one thread per connection)
  lidar_t     → RPLIDAR scan loop (start_scan_express only)
  camera_t    → capture + display + line-follow commands
  distance_t  → HC-SR04 poll every 300 ms
  nav_t       → auto-nav decision loop every 200 ms

All serial access is protected by serial_lock.
Frame buffer is protected by frame_lock.
Shared state writes are protected by state_lock.
"""

import sys
import os
import io
import json
import math
import time
import signal
import socket
import datetime
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn

# ── Optional library imports ──────────────────────────────────────────────────
try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False
    print("[WARN] pyserial not installed — serial features disabled")

try:
    import cv2
    import numpy as np
    HAS_CV = True
except ImportError:
    HAS_CV = False
    print("[WARN] OpenCV (cv2) not installed — camera features disabled")

try:
    from pyrplidar import PyRPlidar
    HAS_LIDAR = True
except ImportError:
    HAS_LIDAR = False
    print("[WARN] pyrplidar not installed — LIDAR disabled")

# ═══════════════════════════════════════════════════════════════════════════════
#  CONSTANTS
# ═══════════════════════════════════════════════════════════════════════════════

HTTP_PORT    = 5000
SERIAL_BAUD  = 9600

# Arduino USB serial candidates (ACM = CH340/ATmega16U2, not USB0/1 used by LIDAR)
ARDUINO_PORTS = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2',
                 '/dev/ttyUSB2', '/dev/ttyUSB3']

# LIDAR USB candidates (USB0 primary, USB1 fallback)
LIDAR_PORTS = ['/dev/ttyUSB1', '/dev/ttyUSB0']   # USB1 is where LIDAR appears on this Pi

CAMERA_IDX = 0   # try 0 first, then 1

# Modes
MODE_MANUAL = 'manual'
MODE_AUTO   = 'auto_nav'
MODE_LINE   = 'line_follow'

# ── LIDAR sector definitions ─────────────────────────────────────────────────
# Angles in degrees. start > end means the sector crosses 0°.
SECTORS = {
    'front':       (340, 20),
    'front_left':  (20,  60),
    'front_right': (300, 340),
    'left':        (60,  120),
    'right':       (240, 300),
    'back':        (150, 210),
}

# ── Navigation thresholds ────────────────────────────────────────────────────
T_FRONT_CLEAR  = 2000   # mm — front path clear (increased: more reaction time)
T_FRONT_CLOSE  = 1200   # mm — front path blocked (increased: stop/turn earlier)
T_SIDE_CLEAR   = 700    # mm — side passage threshold
T_BACK_CLEAR   = 400    # mm — safe to reverse
T_MIN_DIST     = 150    # mm — ignore (rover body / noise)

T_ULTRA_EMRG   = 50     # cm — HC-SR04 emergency distance (stop 50 cm before collision)

T_EDGE_DANGER  = 0.55   # camera edge fill ratio (raised — floor/tape was causing false triggers)
T_EDGE_CAUTION = 0.35

# ── Navigation timing (seconds) ──────────────────────────────────────────────
NAV_COOLDOWN   = 1.5
NAV_STOP_HOLD  = 0.20
NAV_REVERSE_T  = 0.70
NAV_TURN_T     = 0.50

# ── Line-follow tuning (pixels / seconds) ────────────────────────────────────
LF_HARD_PX   = 90
LF_GENTLE_PX = 40
LF_GAP_S     = 0.40    # keep going fwd this long after losing line
LF_SEARCH_S  = 1.20   # search-turn this long before giving up

# ═══════════════════════════════════════════════════════════════════════════════
#  SHARED STATE
# ═══════════════════════════════════════════════════════════════════════════════

class RoverState:
    """All mutable shared state. Use the appropriate lock before writing."""

    def __init__(self):
        # ── Control ──────────────────────────────────────────────────────
        self.mode     = MODE_MANUAL
        self.estop    = False
        self.last_cmd = 'S'

        # ── LIDAR ────────────────────────────────────────────────────────
        self.lidar_ok      = False
        self.lidar_sectors = {k: 9999.0 for k in SECTORS}

        # ── HC-SR04 ──────────────────────────────────────────────────────
        self.dist_cm = -1.0
        self.dist_ts = 0.0

        # ── Camera / edge ────────────────────────────────────────────────
        self.cam_ok    = False
        self.edge_fill = 0.0
        self.edge_zone = 'OK'    # 'OK' | 'CAUTION' | 'DANGER'

        # ── Line follow ──────────────────────────────────────────────────
        self.lf_last_seen = 0.0
        self.lf_dir       = 'center'   # 'center' | 'left' | 'right'

        # ── Auto nav ─────────────────────────────────────────────────────
        self.nav_ts   = 0.0    # time last maneuver started
        self.nav_busy = False

        # ── Recording ────────────────────────────────────────────────────
        self.recording  = False
        self.rec_writer = None
        self.rec_path   = ''

        # ── Frame buffer (JPEG bytes) ─────────────────────────────────────
        self.frame_buf = None

        # ── Locks / events ───────────────────────────────────────────────
        self.serial_lock = threading.Lock()
        self.frame_lock  = threading.Lock()
        self.state_lock  = threading.Lock()
        self.shutdown    = threading.Event()


S = RoverState()       # global state singleton
_ser: 'serial.Serial | None' = None    # Arduino serial port
_lidar = None                          # PyRPlidar instance

# ═══════════════════════════════════════════════════════════════════════════════
#  SERIAL / ARDUINO
# ═══════════════════════════════════════════════════════════════════════════════

def _find_arduino():
    """Scan candidate ports, ping Arduino, return connected Serial or None."""
    if not HAS_SERIAL:
        return None
    for port in ARDUINO_PORTS:
        if not os.path.exists(port):
            continue
        try:
            p = serial.Serial(port, SERIAL_BAUD, timeout=1.0)
            time.sleep(2.0)   # wait for Arduino to finish boot/reset
            with S.serial_lock:
                p.reset_input_buffer()
                p.write(b'P')
                resp = p.readline().decode('utf-8', errors='ignore').strip()
            if resp == 'OK':
                print(f"[Serial] Arduino found at {port}")
                return p
            p.close()
        except Exception as e:
            pass
    print("[Serial] Arduino not found — motor commands will be no-ops")
    return None


# Left/Right are physically swapped on this rover (motor wiring reversed).
# Remap here so the fix applies everywhere: manual, auto-nav, line-follow.
_CMD_REMAP = {'L': 'R', 'R': 'L', 'l': 'r', 'r': 'l'}


def send_cmd(cmd: str) -> str:
    """
    Send a single-character command to Arduino.
    Acquires serial_lock, flushes input, writes, reads one response line.
    Returns the response string (stripped) or '' on error.
    Respects E-STOP: only 'S' passes through when estop is active.
    Applies _CMD_REMAP to correct physical L/R motor swap.
    """
    global _ser
    if _ser is None:
        print(f"[CMD] BLOCKED — no serial (_ser is None), cmd={cmd}")
        return ''
    if S.estop and cmd != 'S':
        print(f"[CMD] BLOCKED — ESTOP active, cmd={cmd}")
        return ''
    original = cmd
    cmd = _CMD_REMAP.get(cmd, cmd)   # fix hardware L/R swap
    try:
        with S.serial_lock:
            _ser.reset_input_buffer()
            _ser.write(cmd.encode('utf-8'))
            resp = _ser.readline().decode('utf-8', errors='ignore').strip()
        with S.state_lock:
            S.last_cmd = cmd
        print(f"[CMD] {original}→{cmd}  resp={resp!r}")
        return resp
    except Exception as e:
        print(f"[Serial] send_cmd('{cmd}') error: {e}")
        return ''

# ═══════════════════════════════════════════════════════════════════════════════
#  DISTANCE POLLING THREAD  (HC-SR04 via Arduino)
# ═══════════════════════════════════════════════════════════════════════════════

def distance_thread():
    """Poll HC-SR04 through Arduino every 150 ms. Uses serial_lock."""
    global _ser
    while not S.shutdown.is_set():
        if _ser is not None and not S.estop:
            try:
                with S.serial_lock:
                    _ser.reset_input_buffer()
                    _ser.write(b'D')
                    line = _ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('D:'):
                    raw = line[2:]
                    if raw and raw != '-1':
                        with S.state_lock:
                            S.dist_cm = float(raw)
                            S.dist_ts = time.time()
            except Exception:
                pass
        S.shutdown.wait(timeout=0.15)

# ═══════════════════════════════════════════════════════════════════════════════
#  RPLIDAR THREAD
# ═══════════════════════════════════════════════════════════════════════════════

def _angle_in_sector(angle: float, start: float, end: float) -> bool:
    """True if angle (0-360) falls in [start, end], handles wrap-around."""
    if start > end:            # sector crosses 0° (e.g. 340-20)
        return angle >= start or angle <= end
    return start <= angle <= end


def lidar_thread():
    """
    Connect to RPLIDAR A1, run start_scan_express(2) continuously,
    update S.lidar_sectors every full revolution.

    CRITICAL: Only start_scan_express(2) works on firmware 1.29 model 24.
    force_scan() and start_scan() cause 'index out of range' errors.
    """
    global _lidar
    if not HAS_LIDAR:
        print("[LIDAR] pyrplidar not available")
        return

    # ── Connect ───────────────────────────────────────────────────────────
    connected = False
    for port in LIDAR_PORTS:
        if S.shutdown.is_set():
            return
        if not os.path.exists(port):
            continue
        try:
            _lidar = PyRPlidar()
            _lidar.connect(port=port, baudrate=115200, timeout=3)
            time.sleep(0.5)
            try:
                _lidar.stop()    # stop any scan left over from previous session
            except Exception:
                pass
            time.sleep(0.5)
            _lidar.set_motor_pwm(500)
            time.sleep(2)
            info   = _lidar.get_info()
            health = _lidar.get_health()
            fw = getattr(info, 'firmware_version', 'unknown')
            print(f"[LIDAR] Connected on {port} — model={info.model} "
                  f"firmware={fw} health={health.status}")
            with S.state_lock:
                S.lidar_ok = True
            connected = True
            break
        except Exception as e:
            print(f"[LIDAR] {port}: {e}")
            try:
                _lidar.disconnect()
            except Exception:
                pass
            _lidar = None

    if not connected:
        print("[LIDAR] Not available — auto-nav uses camera + ultrasonic only")
        return

    # ── Scan loop ─────────────────────────────────────────────────────────
    # Find a working scan mode. Keep the SAME iterator — never stop/restart
    # after getting first data (stop() corrupts the binary protocol with boot text).
    active_iter = None
    for scan_mode, label in [(2, "express mode 2"), (0, "express mode 0")]:
        if S.shutdown.is_set():
            break
        try:
            print(f"[LIDAR] Trying {label}…")
            gen_fn    = _lidar.start_scan_express(scan_mode)
            candidate = gen_fn()   # start iterating — do NOT call stop() after this
            first     = next(candidate, None)
            if first is not None:
                print(f"[LIDAR] {label} OK — "
                      f"first angle={first.angle:.1f} dist={first.distance:.0f}")
                active_iter = candidate   # keep this iterator, don't restart
                break
            else:
                print(f"[LIDAR] {label} produced no data")
        except Exception as e:
            print(f"[LIDAR] {label} failed: {e}")

    if active_iter is None:
        print("[LIDAR] All scan modes failed — LIDAR disabled")
    else:
        print("[LIDAR] Scan running…")

    try:
        if active_iter is None:
            raise RuntimeError("no working scan mode")

        rev_buf: dict[float, float] = {}
        meas_count   = 0
        last_hb_time = time.time()

        for meas in active_iter:
            if S.shutdown.is_set():
                break

            meas_count += 1
            now = time.time()
            if now - last_hb_time >= 5.0:
                print(f"[LIDAR] ♥ alive — {meas_count} meas/5s")
                meas_count   = 0
                last_hb_time = now

            angle = meas.angle
            dist  = meas.distance

            if dist < T_MIN_DIST:
                continue

            if meas.start_flag and rev_buf:
                new_sectors: dict[str, float] = {}
                for name, (seg_start, seg_end) in SECTORS.items():
                    readings = [
                        rev_buf[a] for a in rev_buf
                        if _angle_in_sector(a, seg_start, seg_end)
                    ]
                    new_sectors[name] = min(readings) if readings else 9999.0
                with S.state_lock:
                    S.lidar_sectors.update(new_sectors)
                    S.lidar_ok = True
                rev_buf = {}

            rev_buf[round(angle, 1)] = dist

    except Exception as e:
        print(f"[LIDAR] Scan error: {e}")
    finally:
        with S.state_lock:
            S.lidar_ok = False
        if _lidar:
            try:
                _lidar.stop()
                _lidar.set_motor_pwm(0)
                _lidar.disconnect()
            except Exception:
                pass
        print("[LIDAR] Stopped")

# ═══════════════════════════════════════════════════════════════════════════════
#  AUTO NAVIGATION
# ═══════════════════════════════════════════════════════════════════════════════

def _run_maneuver(steps: list):
    """
    Execute a motor maneuver as a sequence of (cmd, sleep_s) pairs.
    Bails early if E-STOP is activated or shutdown is requested.
    """
    for cmd, delay in steps:
        if S.shutdown.is_set() or S.estop:
            send_cmd('S')
            break
        send_cmd(cmd)
        if delay > 0:
            time.sleep(delay)


def _auto_nav_step():
    """
    One navigation decision cycle. Called from nav_thread every 200 ms.
    Reads all sensor data from shared state, then either sends a single
    command (forward/stop) or blocks to execute a multi-step maneuver.
    """
    # ── Snapshot shared state ─────────────────────────────────────────────
    with S.state_lock:
        sectors  = dict(S.lidar_sectors)
        dist_cm  = S.dist_cm
        edge_z   = S.edge_zone
        lidar_ok = S.lidar_ok
        nav_ts   = S.nav_ts
        nav_busy = S.nav_busy

    if nav_busy:
        return

    now   = time.time()
    front = sectors['front']
    fl    = sectors['front_left']
    fr    = sectors['front_right']
    left  = sectors['left']
    right = sectors['right']
    back  = sectors['back']

    def begin_maneuver():
        with S.state_lock:
            S.nav_busy = True
            S.nav_ts   = time.time()

    def end_maneuver():
        with S.state_lock:
            S.nav_busy = False

    # ── Priority 1: HC-SR04 emergency ─────────────────────────────────────
    if dist_cm > 0 and dist_cm < T_ULTRA_EMRG:
        if now - nav_ts < NAV_COOLDOWN:
            return
        begin_maneuver()
        turn = 'R' if right > left else 'L'
        steps = [('S', NAV_STOP_HOLD)]
        if back > T_BACK_CLEAR:
            steps.append(('b', NAV_REVERSE_T))
        steps += [(turn, NAV_TURN_T), ('S', 0)]
        _run_maneuver(steps)
        end_maneuver()
        return

    # ── Priority 2: Camera danger zone ───────────────────────────────────
    if edge_z == 'DANGER':
        if now - nav_ts < NAV_COOLDOWN:
            return
        begin_maneuver()
        turn = 'R' if right > left else 'L'
        steps = [('S', NAV_STOP_HOLD)]
        if back > T_BACK_CLEAR:
            steps.append(('b', NAV_REVERSE_T))
        steps += [(turn, NAV_TURN_T), ('S', 0)]
        _run_maneuver(steps)
        end_maneuver()
        return

    # ── No LIDAR: use camera caution only ────────────────────────────────
    if not lidar_ok:
        send_cmd('S' if edge_z == 'CAUTION' else 'F')
        return

    # ── LIDAR-based navigation ────────────────────────────────────────────
    if front > T_FRONT_CLEAR:
        # Path is clear — drive forward
        send_cmd('F')
        return

    if front > T_FRONT_CLOSE:
        # Approaching — continue forward (cooldown prevents jitter)
        send_cmd('F')
        return

    # ── Front blocked ─────────────────────────────────────────────────────
    if now - nav_ts < NAV_COOLDOWN:
        return   # respect cooldown before another maneuver

    begin_maneuver()

    if fl < T_SIDE_CLEAR and fr < T_SIDE_CLEAR:
        # Boxed in on front and both flanks
        if back > T_BACK_CLEAR:
            # Reverse then turn toward most open side
            turn = 'R' if right > left else 'L'
            steps = [('S', NAV_STOP_HOLD),
                     ('b', NAV_REVERSE_T),
                     (turn, NAV_TURN_T),
                     ('S', 0)]
        else:
            # Completely boxed in — just stop
            steps = [('S', 0)]
    else:
        # At least one flank is open — turn toward it
        turn = 'R' if fr >= fl else 'L'
        steps = [('S', NAV_STOP_HOLD),
                 (turn, NAV_TURN_T),
                 ('S', 0)]

    _run_maneuver(steps)
    end_maneuver()


def nav_thread():
    """Auto-nav decision loop — runs only when mode is MODE_AUTO."""
    while not S.shutdown.is_set():
        if S.mode == MODE_AUTO and not S.estop:
            try:
                _auto_nav_step()
            except Exception as e:
                print(f"[Nav] Error in decision step: {e}")
        S.shutdown.wait(timeout=0.1)

# ═══════════════════════════════════════════════════════════════════════════════
#  CAMERA THREAD
# ═══════════════════════════════════════════════════════════════════════════════

def _make_placeholder_jpeg() -> bytes | None:
    """Generate a 'NO CAMERA' placeholder JPEG."""
    if not HAS_CV:
        return None
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(img, "NO CAMERA", (190, 230),
                cv2.FONT_HERSHEY_SIMPLEX, 1.6, (70, 70, 70), 2)
    cv2.putText(img, "Check USB connection", (175, 275),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (50, 50, 50), 1)
    _, buf = cv2.imencode('.jpg', img)
    return buf.tobytes()


def _draw_backup_grid(frame, h: int, w: int):
    """
    Draw backup-camera style guide overlay:
      red zone line  at 75% height
      yellow zone    at 55%
      green zone     at 40%
      converging guide lines from center-bottom
    """
    cx      = w // 2
    y_red   = int(h * 0.75)
    y_yel   = int(h * 0.55)
    y_grn   = int(h * 0.40)

    cv2.line(frame, (0, y_red), (w, y_red), (0, 0, 255),   1)
    cv2.line(frame, (0, y_yel), (w, y_yel), (0, 220, 220), 1)
    cv2.line(frame, (0, y_grn), (w, y_grn), (0, 255, 0),   1)

    # Two pairs of converging guide lines
    cv2.line(frame, (cx, h), (0,          y_red), (0, 180, 180), 1)
    cv2.line(frame, (cx, h), (w,          y_red), (0, 180, 180), 1)
    cv2.line(frame, (cx, h), (int(w*.2), y_grn),  (0, 180, 180), 1)
    cv2.line(frame, (cx, h), (int(w*.8), y_grn),  (0, 180, 180), 1)
    return frame


def _process_edge_detection(raw_frame, display, h: int, w: int):
    """
    Canny edge detection on the bottom 25% of the frame (danger zone).
    Updates S.edge_fill and S.edge_zone.
    Overlays a coloured edge map on the display frame.
    """
    y_danger = int(h * 0.75)
    roi = raw_frame[y_danger:, :]

    gray    = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges   = cv2.Canny(blurred, 50, 150)

    total = edges.size
    fill  = float(np.count_nonzero(edges)) / total if total > 0 else 0.0

    if fill > T_EDGE_DANGER:
        zone = 'DANGER'
    elif fill > T_EDGE_CAUTION:
        zone = 'CAUTION'
    else:
        zone = 'OK'

    with S.state_lock:
        S.edge_fill = fill
        S.edge_zone = zone

    # Colour-code edge overlay
    e3 = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
    if zone == 'DANGER':
        coloured = e3 * np.array([0, 0, 1], dtype=np.uint8)
    elif zone == 'CAUTION':
        coloured = e3 * np.array([0, 1, 1], dtype=np.uint8)
    else:
        coloured = e3 * np.array([0, 1, 0], dtype=np.uint8)

    display[y_danger:, :] = cv2.addWeighted(
        display[y_danger:, :], 0.70, coloured, 0.30, 0)

    label_col = {'DANGER': (0, 0, 255), 'CAUTION': (0, 220, 220),
                 'OK': (0, 255, 0)}[zone]
    cv2.putText(display, f"CAM:{zone} {fill:.0%}",
                (4, h - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, label_col, 1)
    return display


def _lf_safe_fwd():
    """Send F only if HC-SR04 distance is safe (>50cm). Otherwise stop."""
    with S.state_lock:
        dist = S.dist_cm
    if dist > 0 and dist < T_ULTRA_EMRG:
        send_cmd('S')
        return False   # blocked
    send_cmd('F')
    return True


def _process_line_follow(raw_frame, display, h: int, w: int):
    """
    Line follow — black electrical tape on a light floor.

    Detection uses Otsu adaptive threshold (handles any lighting automatically)
    combined with an HSV darkness check to reject coloured objects.
    A contour must pass area + width + aspect-ratio checks to be accepted as tape.

    Collision safety: HC-SR04 is checked before every forward command.
    When line is lost the rover STOPS immediately — it does NOT drive blind.
    """
    roi_y = int(h * 0.80)   # bottom 20% only — stays within red danger zone,
    roi   = raw_frame[roi_y:, :]  # avoids detecting distant objects/furniture

    # ── Step 1: Otsu threshold on grayscale ───────────────────────────────
    # Light blur only — heavy blur erases thin tape lines.
    # Otsu auto-finds the threshold between dark tape and bright floor.
    blurred = cv2.GaussianBlur(roi, (5, 5), 0)
    gray    = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
    _, otsu = cv2.threshold(gray, 0, 255,
                            cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    # ── Step 2: HSV darkness gate — reject coloured objects ───────────────
    hsv  = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    dark = cv2.inRange(hsv,
                       np.array([0,   0,   0]),
                       np.array([180, 255, 90]))  # Value < 90 = dark/black pixel

    # ── Step 3: Combine both masks ────────────────────────────────────────
    mask = cv2.bitwise_and(otsu, dark)

    # ── Step 4: Morphological cleanup ─────────────────────────────────────
    # IMPORTANT: use small 3×3 kernel for OPEN so thin tape lines are not erased.
    # A 7×7 open requires a 7×7 solid block to survive — a 15px wide tape line fails that.
    k_open  = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    k_close = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask    = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k_open)   # remove tiny noise dots
    mask    = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k_close)  # fill small gaps in tape

    # ── ROI overlay: cyan = detected tape ────────────────────────────────
    overlay = np.zeros_like(display[roi_y:, :])
    overlay[mask > 0] = (255, 255, 0)
    display[roi_y:, :] = cv2.addWeighted(display[roi_y:, :], 0.6, overlay, 0.4, 0)
    cv2.rectangle(display, (0, roi_y), (w - 1, h - 1), (255, 0, 255), 2)

    # ── Step 5: Contour detection + shape validation ──────────────────────
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cx_frame = w // 2
    now      = time.time()

    valid_cnts = []
    for c in cnts:
        area = cv2.contourArea(c)
        if area < 200:
            continue          # too small — noise dot

        if area > 12000:
            continue          # too large — chair base, table leg base, large shadow
                              # tape at any angle/distance fits within this limit

        _x, _y, cw, _ch = cv2.boundingRect(c)
        if cw < 10:
            continue          # too narrow — single-pixel edge artifact

        # ── Elongation check using minimum-area rotated rectangle ────────
        # Works for diagonal tape: finds the true long/short axes regardless of angle.
        # Tape (long thin stripe): ratio >> 1.8
        # Circular chair base:     ratio ≈ 1.0  → rejected
        # Square/blob objects:     ratio ≈ 1.0  → rejected
        if len(c) >= 5:       # minAreaRect needs at least 5 points
            _, (rw, rh), _ = cv2.minAreaRect(c)
            long_side  = max(rw, rh)
            short_side = min(rw, rh)
            if short_side > 0 and long_side / short_side < 1.8:
                continue      # not elongated enough to be tape

        valid_cnts.append(c)

    if valid_cnts:
        largest = max(valid_cnts, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M['m00'] > 0:
            lx = int(M['m10'] / M['m00'])
            ly = int(M['m01'] / M['m00']) + roi_y

            cv2.drawContours(display[roi_y:, :], [largest], -1, (0, 255, 0), 2)
            cv2.circle(display, (lx, ly), 8, (0, 100, 255), -1)
            cv2.line(display, (cx_frame, h - 1), (lx, ly), (255, 220, 0), 2)

            error = lx - cx_frame
            cv2.putText(display, f"err:{error:+d}px", (4, roi_y + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 220, 0), 1)

            with S.state_lock:
                S.lf_last_seen = now
                S.lf_dir = ('left'  if error < -LF_GENTLE_PX else
                             'right' if error >  LF_GENTLE_PX else 'center')

            if not S.estop:
                ae = abs(error)
                if   ae < LF_GENTLE_PX: _lf_safe_fwd()                       # straight
                elif ae < LF_HARD_PX:   send_cmd('l' if error < 0 else 'r')  # gentle curve
                else:                   send_cmd('L' if error < 0 else 'R')  # hard pivot
            return display

    # ── Line not found ────────────────────────────────────────────────────
    with S.state_lock:
        lost_s = now - S.lf_last_seen
        lf_dir = S.lf_dir

    cv2.putText(display, f"LINE LOST {lost_s:.1f}s",
                (cx_frame - 65, roi_y + 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 60, 255), 2)

    if not S.estop:
        if lost_s < 0.25:
            send_cmd('S')                              # stop immediately — don't drive blind
        elif lost_s < LF_SEARCH_S:
            send_cmd('L' if lf_dir == 'left' else 'R')  # slow search toward last direction
        else:
            send_cmd('S')                              # give up completely

    return display


def camera_thread():
    """
    Camera capture, frame processing, JPEG encoding.
    Writes JPEG bytes to S.frame_buf (protected by S.frame_lock).
    Drives line-follow commands when in MODE_LINE.
    """
    if not HAS_CV:
        print("[Camera] OpenCV not available")
        return

    cap = None
    for idx in [CAMERA_IDX, 1 - CAMERA_IDX]:
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 30)
            print(f"[Camera] Opened index {idx}")
            with S.state_lock:
                S.cam_ok = True
            break
        cap.release()
        cap = None

    if cap is None:
        print("[Camera] No camera found")
        placeholder = _make_placeholder_jpeg()
        with S.frame_lock:
            S.frame_buf = placeholder
        return

    while not S.shutdown.is_set():
        ret, frame = cap.read()
        if not ret:
            S.shutdown.wait(timeout=0.05)
            continue

        h, w = frame.shape[:2]
        display = frame.copy()
        mode    = S.mode

        # ── Per-mode processing ───────────────────────────────────────────
        if mode == MODE_LINE and not S.estop:
            display = _process_line_follow(frame, display, h, w)
        elif mode == MODE_AUTO and not S.estop:
            display = _draw_backup_grid(display, h, w)
            display = _process_edge_detection(frame, display, h, w)
        else:
            display = _draw_backup_grid(display, h, w)

        # ── Mode label ────────────────────────────────────────────────────
        mode_labels = {MODE_MANUAL: 'MANUAL',
                       MODE_AUTO:   'AUTO NAV',
                       MODE_LINE:   'LINE FOLLOW'}
        label = 'EMERGENCY STOP' if S.estop else mode_labels.get(mode, mode.upper())
        lcolor = (0, 0, 200) if S.estop else (230, 230, 230)
        # Background pill
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        cv2.rectangle(display, (2, 4), (tw + 10, th + 12), (0, 0, 0), -1)
        cv2.putText(display, label, (6, th + 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, lcolor, 2)

        # ── Recording ─────────────────────────────────────────────────────
        if S.recording:
            if S.rec_writer is None:
                ts     = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
                path   = f'/tmp/rover_{ts}.avi'
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                S.rec_writer = cv2.VideoWriter(path, fourcc, 15.0, (w, h))
                S.rec_path   = path
                print(f"[Camera] Recording → {path}")
            if S.rec_writer:
                S.rec_writer.write(display)
            # REC badge
            cv2.circle(display, (w - 18, 14), 8, (0, 0, 220), -1)
            cv2.putText(display, 'REC', (w - 54, 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 220), 1)
        elif S.rec_writer is not None:
            S.rec_writer.release()
            S.rec_writer = None
            print(f"[Camera] Recording saved → {S.rec_path}")

        # ── Encode and store ──────────────────────────────────────────────
        ok, buf = cv2.imencode('.jpg', display, [cv2.IMWRITE_JPEG_QUALITY, 72])
        if ok:
            with S.frame_lock:
                S.frame_buf = buf.tobytes()

        S.shutdown.wait(timeout=0.033)   # ~30 fps target

    cap.release()
    if S.rec_writer:
        S.rec_writer.release()
    print("[Camera] Stopped")

# ═══════════════════════════════════════════════════════════════════════════════
#  HTTP SERVER
# ═══════════════════════════════════════════════════════════════════════════════

# ── Embedded dashboard HTML ───────────────────────────────────────────────────
DASHBOARD_HTML = """\
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Rover Dashboard</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#0d1117;color:#c9d1d9;font-family:'Courier New',monospace;font-size:13px;
     overflow-x:hidden;user-select:none}

/* ── E-Stop banner ─────────────────────────────────────────────── */
#estop-banner{display:none;text-align:center;padding:10px;font-size:18px;
  font-weight:bold;cursor:pointer;letter-spacing:2px;
  background:#7f1d1d;animation:pulse-red 1s ease-in-out infinite}
#estop-banner.active{display:block}
@keyframes pulse-red{0%,100%{background:#7f1d1d}50%{background:#dc2626}}

/* ── Top bar ───────────────────────────────────────────────────── */
.topbar{display:flex;align-items:center;background:#161b22;padding:8px 14px;
  gap:12px;border-bottom:2px solid #21262d}
.topbar h1{color:#f85149;font-size:16px;letter-spacing:1px;flex:1}
.topbar-status{font-size:11px;color:#6e7681}
.topbar-status span{color:#58a6ff}

/* ── Layout ────────────────────────────────────────────────────── */
.main{display:grid;grid-template-columns:1fr 310px;gap:10px;
  padding:10px;height:calc(100vh - 88px);min-height:0}

/* ── Camera panel ──────────────────────────────────────────────── */
.cam-panel{background:#000;border-radius:8px;overflow:hidden;position:relative;
  display:flex;align-items:center;justify-content:center;
  border:2px solid #21262d;min-height:200px}
.cam-panel img{max-width:100%;max-height:100%;object-fit:contain;display:block}
.cam-badge{position:absolute;top:6px;right:8px;background:rgba(0,0,0,.65);
  padding:3px 8px;border-radius:4px;font-size:10px;color:#58a6ff}
.rec-dot{color:#f85149;animation:blink 1s infinite}
@keyframes blink{50%{opacity:0}}

/* ── Right column ──────────────────────────────────────────────── */
.right-col{display:flex;flex-direction:column;gap:8px;overflow-y:auto}
.panel{background:#161b22;border-radius:8px;padding:10px;
  border:1px solid #21262d;flex-shrink:0}
.panel-title{color:#f85149;font-size:10px;font-weight:bold;
  text-transform:uppercase;letter-spacing:1px;margin-bottom:8px}

/* ── Mode buttons ──────────────────────────────────────────────── */
.mode-row{display:flex;gap:6px}
.btn-mode{flex:1;padding:8px 4px;border:2px solid #30363d;background:#0d1117;
  color:#8b949e;border-radius:6px;cursor:pointer;font-size:11px;
  font-family:inherit;font-weight:bold;letter-spacing:.5px;transition:all .15s}
.btn-mode:hover{border-color:#58a6ff;color:#c9d1d9}
.btn-mode.active{background:#1f6feb;border-color:#58a6ff;color:#fff}

/* ── Drive grid ────────────────────────────────────────────────── */
.drive-grid{display:grid;grid-template-columns:repeat(3,1fr);
  grid-template-rows:repeat(3,1fr);gap:5px;aspect-ratio:1;max-height:180px}
.btn-drive{display:flex;align-items:center;justify-content:center;
  background:#0d1117;border:2px solid #30363d;border-radius:8px;
  color:#8b949e;font-size:22px;cursor:pointer;transition:all .1s;
  touch-action:manipulation;-webkit-tap-highlight-color:transparent;
  min-height:44px}
.btn-drive:active,.btn-drive.pressed{background:#1f3a5f;border-color:#58a6ff;
  color:#fff;transform:scale(.94)}
.btn-stop{background:#1c0a0a!important;border-color:#da3633!important;
  color:#f85149!important;font-size:11px!important;font-weight:bold}
.btn-stop:active{background:#5a1a1a!important}
.cell-empty{border:none;background:transparent;pointer-events:none}

/* ── LIDAR sector grid ─────────────────────────────────────────── */
.sector-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:5px}
.sector-box{text-align:center;padding:6px 3px;border-radius:6px;
  border:1px solid #30363d;background:#0d1117}
.sector-box.ok    {border-color:#1a3a1a}
.sector-box.warn  {border-color:#3a3010}
.sector-box.danger{border-color:#3a1010}
.sec-label{font-size:9px;color:#6e7681;text-transform:uppercase}
.sec-dist {font-size:15px;font-weight:bold;margin:2px 0;line-height:1}
.sec-dist.ok    {color:#3fb950}
.sec-dist.warn  {color:#d29922}
.sec-dist.danger{color:#f85149}

/* ── Sensor rows ───────────────────────────────────────────────── */
.sensor-row{display:flex;justify-content:space-between;align-items:center;
  padding:5px 0;border-bottom:1px solid #21262d}
.sensor-row:last-child{border:none}
.sensor-label{color:#6e7681;font-size:11px}
.sensor-val  {font-size:14px;font-weight:bold}
.sv-ok     {color:#3fb950}
.sv-warn   {color:#d29922}
.sv-danger {color:#f85149}
.sv-neutral{color:#8b949e}

/* ── Record button ─────────────────────────────────────────────── */
.btn-record{width:100%;padding:7px;background:#0d1117;border:2px solid #3a1010;
  color:#f85149;border-radius:6px;cursor:pointer;font-family:inherit;
  font-size:12px;font-weight:bold;transition:all .2s}
.btn-record.active{background:#3a1010;border-color:#f85149;
  animation:pulse-red 1s infinite}
.btn-record:hover{border-color:#f85149}

/* ── Emergency stop ────────────────────────────────────────────── */
.btn-estop{width:100%;padding:14px;background:#3a1010;border:3px solid #f85149;
  color:#fff;border-radius:8px;cursor:pointer;font-family:inherit;font-size:15px;
  font-weight:bold;letter-spacing:2px;animation:pulse-red 2.5s infinite}
.btn-estop.resumed{background:#0d2818;border-color:#3fb950;color:#3fb950;
  animation:none;letter-spacing:.5px}

/* ── Status bar ────────────────────────────────────────────────── */
.statusbar{position:fixed;bottom:0;left:0;right:0;background:#161b22;
  border-top:1px solid #21262d;padding:3px 14px;display:flex;gap:16px;
  font-size:10px;color:#6e7681}
.statusbar span{color:#58a6ff}
</style>
</head>
<body>

<div id="estop-banner" onclick="toggleEstop()">
  ⚡ EMERGENCY STOP ACTIVE — TAP TO RESUME ⚡
</div>

<div class="topbar">
  <h1>ROVER DASHBOARD</h1>
  <div class="topbar-status">
    Mode: <span id="ts-mode">MANUAL</span> &nbsp;|&nbsp;
    Cmd:  <span id="ts-cmd">S</span>
  </div>
</div>

<div class="main">

  <!-- ── Left: Camera ─────────────────────────────────────────── -->
  <div class="cam-panel">
    <img id="cam-img" src="" alt="camera">
    <div class="cam-badge">
      <span id="rec-dot" class="rec-dot" style="display:none">● </span>640×480
    </div>
  </div>

  <!-- ── Right: Controls ──────────────────────────────────────── -->
  <div class="right-col">

    <!-- Mode selection -->
    <div class="panel">
      <div class="panel-title">Mode</div>
      <div class="mode-row">
        <button class="btn-mode active" id="bm-manual"     onclick="setMode('manual')">MANUAL</button>
        <button class="btn-mode"        id="bm-auto_nav"   onclick="setMode('auto_nav')">AUTO NAV</button>
        <button class="btn-mode"        id="bm-line_follow" onclick="setMode('line_follow')">LINE FOLLOW</button>
      </div>
    </div>

    <!-- Drive controls -->
    <div class="panel">
      <div class="panel-title">Drive — WASD / Arrows / Mouse</div>
      <div class="drive-grid">
        <div class="btn-drive cell-empty"></div>
        <div class="btn-drive" id="bd-F"
             onmousedown="beginDrive('F')" ontouchstart="beginDrive('F')"
             onmouseup="endDrive()"       ontouchend="endDrive()">▲</div>
        <div class="btn-drive cell-empty"></div>

        <div class="btn-drive" id="bd-L"
             onmousedown="beginDrive('L')" ontouchstart="beginDrive('L')"
             onmouseup="endDrive()"        ontouchend="endDrive()">◄</div>
        <div class="btn-drive btn-stop"
             onclick="driveNow('S')">STOP</div>
        <div class="btn-drive" id="bd-R"
             onmousedown="beginDrive('R')" ontouchstart="beginDrive('R')"
             onmouseup="endDrive()"        ontouchend="endDrive()">►</div>

        <div class="btn-drive cell-empty"></div>
        <div class="btn-drive" id="bd-b"
             onmousedown="beginDrive('b')" ontouchstart="beginDrive('b')"
             onmouseup="endDrive()"        ontouchend="endDrive()">▼</div>
        <div class="btn-drive cell-empty"></div>
      </div>
    </div>

    <!-- LIDAR sectors -->
    <div class="panel">
      <div class="panel-title">LIDAR Sectors
        <span id="lidar-badge" style="color:#6e7681;font-size:9px;margin-left:4px">(offline)</span>
      </div>
      <div class="sector-grid">
        <!-- Row 1: FL, F, FR -->
        <div class="sector-box" id="sb-front_left">
          <div class="sec-label">FRONT-L</div>
          <div class="sec-dist" id="sd-front_left">---</div>
        </div>
        <div class="sector-box" id="sb-front">
          <div class="sec-label">FRONT</div>
          <div class="sec-dist" id="sd-front">---</div>
        </div>
        <div class="sector-box" id="sb-front_right">
          <div class="sec-label">FRONT-R</div>
          <div class="sec-dist" id="sd-front_right">---</div>
        </div>
        <!-- Row 2: L, B, R -->
        <div class="sector-box" id="sb-left">
          <div class="sec-label">LEFT</div>
          <div class="sec-dist" id="sd-left">---</div>
        </div>
        <div class="sector-box" id="sb-back">
          <div class="sec-label">BACK</div>
          <div class="sec-dist" id="sd-back">---</div>
        </div>
        <div class="sector-box" id="sb-right">
          <div class="sec-label">RIGHT</div>
          <div class="sec-dist" id="sd-right">---</div>
        </div>
      </div>
    </div>

    <!-- Sensors -->
    <div class="panel">
      <div class="panel-title">Sensors</div>
      <div class="sensor-row">
        <div class="sensor-label">HC-SR04 Distance</div>
        <div class="sensor-val sv-neutral" id="sv-ultra">-- cm</div>
      </div>
      <div class="sensor-row">
        <div class="sensor-label">Camera Zone</div>
        <div class="sensor-val sv-neutral" id="sv-zone">--</div>
      </div>
      <div class="sensor-row">
        <div class="sensor-label">Edge Fill</div>
        <div class="sensor-val sv-neutral" id="sv-fill">--%</div>
      </div>
    </div>

    <!-- Recording -->
    <div class="panel">
      <div class="panel-title">Recording</div>
      <button class="btn-record" id="btn-rec" onclick="toggleRecord()">
        ● START RECORDING
      </button>
    </div>

    <!-- Emergency Stop -->
    <div class="panel">
      <button class="btn-estop" id="btn-estop" onclick="toggleEstop()">
        ⚡ EMERGENCY STOP ⚡
      </button>
    </div>

  </div><!-- .right-col -->
</div><!-- .main -->

<div class="statusbar">
  <div>Mode: <span id="sb-mode">MANUAL</span></div>
  <div>Cmd:  <span id="sb-cmd">S</span></div>
  <div>LIDAR:<span id="sb-lidar">offline</span></div>
  <div>HC-SR04:<span id="sb-dist">--</span></div>
  <div>Cam:<span id="sb-cam">--</span></div>
</div>

<script>
// ── State ────────────────────────────────────────────────────────────────────
let currentMode = 'manual';
let estopActive = false;
let recording   = false;
let pressedId   = null;
let releaseTimer = null;

// ── Drive ────────────────────────────────────────────────────────────────────
function driveNow(cmd) {
  if (estopActive) return;
  if (cmd !== 'S' && currentMode !== 'manual') return;
  fetch('/command', {
    method: 'POST',
    headers: {'Content-Type':'application/json'},
    body: JSON.stringify({cmd})
  });
  document.querySelectorAll('.btn-drive').forEach(b => b.classList.remove('pressed'));
  const btn = document.getElementById('bd-' + cmd);
  if (btn) btn.classList.add('pressed');
}

function beginDrive(cmd) {
  if (releaseTimer) { clearTimeout(releaseTimer); releaseTimer = null; }
  driveNow(cmd);
  pressedId = cmd;
}

function endDrive() {
  if (pressedId) {
    const btn = document.getElementById('bd-' + pressedId);
    if (btn) btn.classList.remove('pressed');
    pressedId = null;
  }
  // Auto-stop 200 ms after button release (safety)
  releaseTimer = setTimeout(() => {
    if (currentMode === 'manual' && !estopActive) driveNow('S');
  }, 200);
}

// ── Mode ─────────────────────────────────────────────────────────────────────
function setMode(mode) {
  fetch('/mode', {
    method: 'POST',
    headers: {'Content-Type':'application/json'},
    body: JSON.stringify({mode})
  }).then(r => r.json()).then(d => {
    if (d.ok) syncModeButtons(mode);
  });
}

function syncModeButtons(mode) {
  currentMode = mode;
  document.querySelectorAll('.btn-mode').forEach(b => b.classList.remove('active'));
  const btn = document.getElementById('bm-' + mode);
  if (btn) btn.classList.add('active');
}

// ── E-Stop ───────────────────────────────────────────────────────────────────
function toggleEstop() {
  estopActive = !estopActive;
  fetch('/estop', {
    method: 'POST',
    headers: {'Content-Type':'application/json'},
    body: JSON.stringify({active: estopActive})
  });
  applyEstopUI();
}

function applyEstopUI() {
  const banner = document.getElementById('estop-banner');
  const btn    = document.getElementById('btn-estop');
  if (estopActive) {
    banner.classList.add('active');
    btn.classList.remove('resumed');
    btn.textContent = '⚡ EMERGENCY STOP ⚡';
  } else {
    banner.classList.remove('active');
    btn.classList.add('resumed');
    btn.textContent = '✓ OPERATION RESUMED — tap to re-arm stop';
  }
}

// ── Recording ────────────────────────────────────────────────────────────────
function toggleRecord() {
  recording = !recording;
  fetch('/record', {
    method: 'POST',
    headers: {'Content-Type':'application/json'},
    body: JSON.stringify({recording})
  });
  const btn = document.getElementById('btn-rec');
  const dot = document.getElementById('rec-dot');
  if (recording) {
    btn.textContent = '■ STOP RECORDING';
    btn.classList.add('active');
    dot.style.display = 'inline';
  } else {
    btn.textContent = '● START RECORDING';
    btn.classList.remove('active');
    dot.style.display = 'none';
  }
}

// ── Keyboard ─────────────────────────────────────────────────────────────────
const KEY_MAP = {
  ArrowUp:'F', w:'F', W:'F',
  ArrowDown:'b', s:'b', S:'b',
  ArrowLeft:'L', a:'L', A:'L',
  ArrowRight:'R', d:'R', D:'R',
  ' ':'S'
};
const keysHeld = new Set();

document.addEventListener('keydown', e => {
  if (keysHeld.has(e.key)) return;
  if (e.key === ' ') e.preventDefault();
  keysHeld.add(e.key);
  const cmd = KEY_MAP[e.key];
  if (cmd) { beginDrive(cmd); }
});
document.addEventListener('keyup', e => {
  keysHeld.delete(e.key);
  if (KEY_MAP[e.key]) endDrive();
});

// ── Status polling ────────────────────────────────────────────────────────────
function sectorClass(mm) {
  if (mm >= 9990) return 'ok';
  if (mm < 800)   return 'danger';
  if (mm < 1500)  return 'warn';
  return 'ok';
}
function sectorLabel(mm) {
  if (mm >= 9990) return '∞';
  return Math.round(mm / 10) + 'cm';
}

function updateStatus() {
  fetch('/status')
    .then(r => r.json())
    .then(d => {
      // Mode
      const ml = {manual:'MANUAL', auto_nav:'AUTO NAV', line_follow:'LINE FOLLOW'};
      document.getElementById('ts-mode').textContent = ml[d.mode] || d.mode;
      document.getElementById('ts-cmd').textContent  = d.last_cmd || '--';
      document.getElementById('sb-mode').textContent = ml[d.mode] || d.mode;
      document.getElementById('sb-cmd').textContent  = d.last_cmd || '--';
      if (d.mode !== currentMode) syncModeButtons(d.mode);

      // E-Stop sync
      if (d.estop !== estopActive) {
        estopActive = d.estop;
        applyEstopUI();
      }

      // LIDAR sectors
      const ok = d.lidar_ok;
      document.getElementById('lidar-badge').textContent = ok ? '(online)' : '(offline)';
      document.getElementById('sb-lidar').textContent = ok ? 'online' : 'offline';
      for (const [name, mm] of Object.entries(d.lidar_sectors || {})) {
        const box  = document.getElementById('sb-' + name);
        const dist = document.getElementById('sd-' + name);
        if (!box || !dist) continue;
        const cls = sectorClass(mm);
        box.className  = 'sector-box ' + cls;
        dist.className = 'sec-dist ' + cls;
        dist.textContent = sectorLabel(mm);
      }

      // Ultrasonic
      const cm = d.dist_cm;
      const ultraEl = document.getElementById('sv-ultra');
      ultraEl.textContent = cm > 0 ? cm.toFixed(1) + ' cm' : '-- cm';
      ultraEl.className = 'sensor-val ' +
        (cm > 0 && cm < 18 ? 'sv-danger' : cm > 0 && cm < 40 ? 'sv-warn' : 'sv-ok');
      document.getElementById('sb-dist').textContent = cm > 0 ? cm.toFixed(1) + 'cm' : '--';

      // Camera zone
      const zone   = d.edge_zone || 'OK';
      const zoneEl = document.getElementById('sv-zone');
      zoneEl.textContent = zone;
      zoneEl.className = 'sensor-val ' +
        (zone === 'DANGER' ? 'sv-danger' : zone === 'CAUTION' ? 'sv-warn' : 'sv-ok');
      document.getElementById('sv-fill').textContent =
        ((d.edge_fill || 0) * 100).toFixed(1) + '%';
      document.getElementById('sb-cam').textContent = zone;
    })
    .catch(() => {});
}

// ── Camera frame polling ──────────────────────────────────────────────────────
// We poll /frame?ts instead of using MJPEG src directly to handle
// reconnections cleanly and avoid browser MJPEG buffering issues.
let lastFrameTs = 0;
function refreshFrame() {
  const img = document.getElementById('cam-img');
  const url = '/frame?' + Date.now();
  const tmp = new Image();
  tmp.onload = () => {
    img.src = tmp.src;
    lastFrameTs = Date.now();
  };
  tmp.onerror = () => {};
  tmp.src = url;
}

setInterval(updateStatus, 500);
setInterval(refreshFrame,  100);

// Bootstrap
updateStatus();
refreshFrame();
</script>
</body>
</html>
"""


# ── HTTP handler ──────────────────────────────────────────────────────────────

class RoverHandler(BaseHTTPRequestHandler):
    """Handles all HTTP requests from the dashboard."""

    def log_message(self, fmt, *args):
        pass   # suppress default per-request logging

    # ── GET ───────────────────────────────────────────────────────────────
    def do_GET(self):
        path = self.path.split('?')[0]

        if path == '/':
            self._send_html(DASHBOARD_HTML)

        elif path == '/frame':
            # Single JPEG frame for JS polling
            with S.frame_lock:
                data = S.frame_buf
            if data:
                self._send_bytes(data, 'image/jpeg',
                                 extra=[('Cache-Control', 'no-cache, no-store')])
            else:
                self.send_response(503)
                self.end_headers()

        elif path == '/stream':
            # MJPEG stream (kept as fallback / direct use)
            self.send_response(200)
            self.send_header('Content-Type',
                             'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'keep-alive')
            self.end_headers()
            try:
                while not S.shutdown.is_set():
                    with S.frame_lock:
                        data = S.frame_buf
                    if data:
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n')
                        self.wfile.write(
                            f'Content-Length: {len(data)}\r\n\r\n'.encode())
                        self.wfile.write(data)
                        self.wfile.write(b'\r\n')
                        self.wfile.flush()
                    time.sleep(0.033)
            except (BrokenPipeError, ConnectionResetError):
                pass

        elif path == '/status':
            with S.state_lock:
                payload = {
                    'mode':          S.mode,
                    'estop':         S.estop,
                    'recording':     S.recording,
                    'last_cmd':      S.last_cmd,
                    'lidar_ok':      S.lidar_ok,
                    'lidar_sectors': dict(S.lidar_sectors),
                    'dist_cm':       S.dist_cm,
                    'cam_ok':        S.cam_ok,
                    'edge_fill':     S.edge_fill,
                    'edge_zone':     S.edge_zone,
                }
            self._send_json(payload)

        else:
            self.send_response(404)
            self.end_headers()

    # ── POST ──────────────────────────────────────────────────────────────
    def do_POST(self):
        length = int(self.headers.get('Content-Length', 0))
        body   = self.rfile.read(length)
        try:
            data = json.loads(body)
        except Exception:
            data = {}

        path = self.path.split('?')[0]

        if path == '/command':
            cmd = str(data.get('cmd', ''))
            if len(cmd) == 1 and not S.estop:
                if S.mode == MODE_MANUAL or cmd == 'S':
                    send_cmd(cmd)
            self._send_json({'ok': True})

        elif path == '/mode':
            mode = data.get('mode', MODE_MANUAL)
            if mode in (MODE_MANUAL, MODE_AUTO, MODE_LINE):
                with S.state_lock:
                    S.mode = mode
                send_cmd('S')   # safe stop on mode switch
            self._send_json({'ok': True, 'mode': S.mode})

        elif path == '/estop':
            active = bool(data.get('active', True))
            with S.state_lock:
                S.estop = active
            if active:
                send_cmd('S')
            self._send_json({'ok': True, 'estop': active})

        elif path == '/record':
            rec = bool(data.get('recording', False))
            with S.state_lock:
                S.recording = rec
                if not rec and S.rec_writer is not None:
                    S.rec_writer.release()
                    S.rec_writer = None
            self._send_json({'ok': True, 'recording': rec})

        else:
            self.send_response(404)
            self.end_headers()

    # ── Helpers ───────────────────────────────────────────────────────────
    def _send_html(self, html: str):
        body = html.encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, obj):
        body = json.dumps(obj).encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_bytes(self, data: bytes, mime: str, extra=None):
        try:
            self.send_response(200)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(data)))
            if extra:
                for k, v in extra:
                    self.send_header(k, v)
            self.end_headers()
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass   # browser closed connection before frame was fully sent — normal


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """One thread per connection — needed for MJPEG stream + concurrent API."""
    daemon_threads = True
    allow_reuse_address = True

    def handle_error(self, request, client_address):
        """Silence BrokenPipe — these are normal with 100 ms frame polling."""
        import sys
        exc = sys.exc_info()[1]
        if isinstance(exc, (BrokenPipeError, ConnectionResetError)):
            return
        super().handle_error(request, client_address)

# ═══════════════════════════════════════════════════════════════════════════════
#  STARTUP / SHUTDOWN
# ═══════════════════════════════════════════════════════════════════════════════

def _shutdown(sig=None, frame=None):
    print("\n[Rover] Shutting down…")
    S.shutdown.set()

    # Stop motors
    if _ser:
        try:
            with S.serial_lock:
                _ser.write(b'S')
            time.sleep(0.15)
            _ser.close()
        except Exception:
            pass

    # Stop LIDAR
    if _lidar and S.lidar_ok:
        try:
            _lidar.stop()
            _lidar.set_motor_pwm(0)
            _lidar.disconnect()
        except Exception:
            pass

    # Stop recording
    with S.state_lock:
        if S.rec_writer:
            S.rec_writer.release()
            S.rec_writer = None

    print("[Rover] All systems stopped. Goodbye.")
    sys.exit(0)


def _get_local_ip() -> str:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return '127.0.0.1'


def main():
    global _ser

    print("=" * 62)
    print("  ROVER DASHBOARD  —  Autonomous Delivery Rover")
    print("=" * 62)
    print()

    # ── Arduino ───────────────────────────────────────────────────────────
    print("[Serial] Searching for Arduino…")
    _ser = _find_arduino()

    # ── Background threads ────────────────────────────────────────────────
    thread_specs = [
        ('lidar',    lidar_thread),
        ('camera',   camera_thread),
        ('distance', distance_thread),
        ('nav',      nav_thread),
    ]
    for name, target in thread_specs:
        t = threading.Thread(target=target, name=name, daemon=True)
        t.start()
        print(f"[Main]   Thread '{name}' started")

    # ── Signal handlers ───────────────────────────────────────────────────
    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    # ── HTTP server ───────────────────────────────────────────────────────
    server = ThreadedHTTPServer(('0.0.0.0', HTTP_PORT), RoverHandler)
    local_ip = _get_local_ip()

    print()
    print(f"[Dashboard] http://{local_ip}:{HTTP_PORT}      ← open this on any device")
    print(f"[Dashboard] http://localhost:{HTTP_PORT}")
    print("[Dashboard] Press Ctrl+C to stop")
    print()

    server.serve_forever()


if __name__ == '__main__':
    main()
