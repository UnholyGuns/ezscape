# V49: bounded loiter family + source-timeline floor.
# V44: empirical physical-cadence scheduler for synthetic relocations.
# V21: paired source-profile/native-report timing experiment.
"""
USB HID mouse bridge with recording and position-aware sequential playback.

Coordinate model
----------------
The bridge maintains ONE cumulative position:

    x, y = where the bridge believes the Windows cursor is

(0, 0) is simply the cursor location when this program starts.

Physical mouse movement updates x/y during normal passthrough. Generated HID
movement updates the same x/y during playback. Therefore every recording is
stored in the same coordinate system.

Each CSV stores:
    time_ns, dx, dy, wheel, buttons, x, y

The first row's x/y identifies the recording's starting region, and the
subsequent x/y values describe the recorded trajectory in the same persistent
coordinate system.

Controls
--------
BTN_SIDE  : start/stop recording
BTN_EXTRA : short press/release = start/stop playback; hold 3 s = clear ACTIVE pattern data

Playback
--------
Recordings in the selected pattern folder are played sequentially by filename. Before each recording, the
bridge smoothly moves from its current tracked position to the recording's
stored starting position, then replays the recorded relative HID reports.

Assumptions
-----------
This is integrated relative motion, not cursor-position feedback from Windows.
For the coordinate model to remain valid:
  * Windows "Enhance pointer precision" should be disabled.
  * The cursor should not hit/clamp against a screen edge.
  * The cursor should not be moved by another input source.
  * The physical mouse should not be moved during playback.
"""

from evdev import InputDevice, ecodes
import csv
import json
import hashlib
import glob
import os
import math
import re
import secrets
import shutil
import threading
import time
import argparse


# ============================================================
# Configuration
# ============================================================

MOUSE_DEVICE = "/dev/input/event4"
HID_DEVICE = "/dev/hidg0"
# Optional pattern folder: --pattern drawCircle (or --pattern drawRectangle).
# Existing default remains unchanged when --pattern is omitted.
BASE_RECORDING_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "mouse_recordings"
)
_parser = argparse.ArgumentParser(description="USB HID mouse bridge")
_parser.add_argument("--pattern", default=None,
                     help="Existing or new pattern subdirectory under mouse_recordings")
_args = _parser.parse_args()
if _args.pattern:
    if (os.path.basename(_args.pattern) != _args.pattern
            or _args.pattern in (".", "..")
            or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]*", _args.pattern)):
        _parser.error("--pattern must be a simple folder name (letters, digits, _ or -)")
    RECORDING_DIR = os.path.join(BASE_RECORDING_DIR, _args.pattern)
else:
    RECORDING_DIR = BASE_RECORDING_DIR

# Position assigned to the cursor at the FINAL middle-click calibration step.
# For reusable recordings, reference the same physical point in every session.
REFERENCE_X = 0
REFERENCE_Y = 0
TRANSITION_DIAGNOSTIC_LOG = os.path.join(
    RECORDING_DIR, "transition_diagnostics.txt"
)

SMOOTH_MOVE_INTERVAL = 0.008
SMOOTH_MOVE_DURATION = 0.5

# Playback transition style. Change this to "linear" or "curve".
SMOOTH_MOVE_MODE = "curve"

# Curve strength as a fraction of the start-to-target distance.
CURVE_AMOUNT = 0.15

# Fresh random playback-start offset around each recording's stored start.
# Units are mouse counts. Set to 0 to disable.
PLAYBACK_START_OFFSET_MAX = 5

# Playback engine:
#   "recorded"  = original dx/dy reports and timing
#   "generated" = newly generated cubic-Bezier paths between button events
PLAYBACK_MODE = "generated"

# Generated-path settings.
# Geometry is randomized, while each recorded segment's normalized hand-speed
# profile is preserved. Recorded button state is held during generated motion,
# so click-and-drag motion is reproduced correctly.
GENERATED_MOVE_INTERVAL = 0.008
GENERATED_CURVE_MIN = 0.18
GENERATED_CURVE_MAX = 0.65
GENERATED_CONTROL_JITTER = 0.70
GENERATED_ARC_TABLE_STEPS = 300

# V7: permit Bezier control points to project behind the start or beyond the
# target. Endpoints remain exact, but X/Y no longer have to monotonically
# approach the target.
GENERATED_CONTROL1_LONGITUDINAL_MIN = -0.60
GENERATED_CONTROL1_LONGITUDINAL_MAX = 0.85
GENERATED_CONTROL2_LONGITUDINAL_MIN = 0.15
GENERATED_CONTROL2_LONGITUDINAL_MAX = 1.60

# V8 timed loiter:
# A long stationary, no-button source interval keeps its ORIGINAL end time.
# Playback waits at the original point until LOITER_TIMEOUT, makes one generated
# excursion to a temporary point, waits there for the remainder of the original
# stationary interval, and then proceeds normally from that temporary point.
# There is NO return to the old point.
LOITER_ENABLED = True
LOITER_TIMEOUT = 2.0
LOITER_MOTION_THRESHOLD = 3.0
LOITER_MIN_RADIUS = 200.0
LOITER_MAX_RADIUS = 900.0
LOITER_TARGET_MAX_ATTEMPTS = 200

# V11: loiter excursions prioritize a realistic source-derived speed over
# forcing the excursion to finish inside the remaining stationary time.
# If the excursion itself runs long, subsequent playback simply shifts later.
LOITER_PRESERVE_SPEED_OVER_SCHEDULE = True

# V48 bounded loiter timing:
# Loiter geometry may still use the full configured radius, but each synthetic
# loiter leg gets an explicit bounded duration. The selected physical gesture
# supplies only the normalized acceleration/deceleration (speed_shape).
LOITER_EXCURSION_DURATION_MIN = 0.8
LOITER_EXCURSION_DURATION_MAX = 1.6
LOITER_EXIT_DURATION_MIN = 0.8
LOITER_EXIT_DURATION_MAX = 1.6
LOITER_ANCHOR_RETURN_DURATION_MIN = 0.8
LOITER_ANCHOR_RETURN_DURATION_MAX = 1.6
LOITER_DURATION_STEP = 0.1

# Generated playback may run late, but must never run ahead of the source.
PLAYBACK_ENFORCE_SOURCE_TIMELINE_FLOOR = True

# Movement timing variation. 0.08 means each generated movement interval is
# independently stretched/compressed by up to +/-8%. The normalized recorded
# acceleration/deceleration shape is preserved. Set to 0.0 to disable.
GENERATED_TIMING_VARIATION = 0.08

# V16: build motion profiles from the ACTIVE part of a recorded segment.
# Leading/trailing reports with no XY motion are treated as dwell, not as part
# of the acceleration/deceleration profile. Internal zero reports are preserved.
TRIM_PROFILE_STATIONARY_EDGES = True

# V17: generated HID reports use timing intervals measured from the physical
# recording CSVs instead of an idealized 8.000 ms schedule.
#
# Timing samples are collected only from adjacent reports while XY motion is
# active, then robustly trimmed so long pauses / segment boundaries are not
# injected into normal generated motion. A contiguous block is replayed at a
# time to retain short-term timing structure from the physical device/OS.
MEASURED_REPORT_TIMING_ENABLED = True
MEASURED_TIMING_LOW_QUANTILE = 0.005
MEASURED_TIMING_HIGH_QUANTILE = 0.995
MEASURED_TIMING_ABSOLUTE_MIN = 0.004
MEASURED_TIMING_ABSOLUTE_MAX = 0.020
MEASURED_TIMING_BLOCK_MIN = 16
MEASURED_TIMING_BLOCK_MAX = 64

# V44 synthetic HID scheduler. Synthetic relocations draw native report intervals
# independently from the measured physical active-motion timing pool. Long
# 16-18 ms active gaps are excluded here: offline stress testing showed that
# replaying those rare gaps substantially restored the synthetic acceleration
# tail, while the <=10.5 ms pool retained 218/236 (92.4%) measured intervals
# and closely matched the central physical timing CDF.
SYNTHETIC_EMPIRICAL_TIMING_ENABLED = True
SYNTHETIC_EMPIRICAL_TIMING_MAX = 0.0105

# V18: keep native report timing coupled to the SAME physical motion profile
# that supplies cumulative-distance progress. This replaces V17's independent
# random timing-block selection during generated movement.
#
# The source profile already contains (elapsed_time, cumulative_progress) at
# native physical report boundaries, so its elapsed-time differences are the
# source dt sequence. After duration / arc-length scaling, those timestamps are
# scaled together with the profile rather than borrowing dt values elsewhere.
PAIRED_PROFILE_REPORT_TIMING = True

# V20: source pauses keep their exact recorded elapsed duration. We no longer
# use a fixed 25 ms pause threshold. Instead, the upper edge of the measured
# active-report timing distribution defines what counts as a normal report
# interval. Anything longer is treated as missing/stationary report time and
# is filled with zero-motion HID reports for the exact observed gap duration.
PAUSE_ZERO_REPORTS_ENABLED = True
PAUSE_TIMING_QUANTILE = 0.995

# V21 diagnostics. This does not change generated motion behavior.
# Each generated recording-transition report is logged with normalized
# transition progress so acceleration can be plotted against 0..1 progress.
TRANSITION_PROGRESS_DEBUG_ENABLED = False
TRANSITION_PROGRESS_DEBUG_FILE = os.path.join(
    RECORDING_DIR, "transition_progress_debug.csv"
)

# V9: aggressive curves are longer than the straight-line source displacement.
# Stretch generated movement duration by Bezier_arc_length / endpoint_distance
# so stronger curves do not imply artificially higher cursor speed.
GENERATED_ARC_LENGTH_TIMING = True
GENERATED_ARC_LENGTH_TIMING_MAX_SCALE = 3.0

# Version 5 button-dwell variation. This is applied only to intervals where a
# button is held and the recorded pointer motion is effectively stationary.
# It is deliberately separate from GENERATED_TIMING_VARIATION so drag motion
# is not randomized twice. 0.12 means +/-12% around the recorded dwell.
BUTTON_DWELL_VARIATION = 0.12
BUTTON_DWELL_MOTION_THRESHOLD = 3.0  # cumulative mouse counts
BUTTON_DWELL_MIN_DURATION = 0.020    # 20 ms safety floor
BUTTON_DWELL_MAX_DURATION = 2.000    # ordinary click/short-hold ceiling

# Native generated-output capture. This records every generated HID report
# emitted during playback and is the data source used by the current analyzer.
NATIVE_OUTPUT_CAPTURE_ENABLED = True
NATIVE_OUTPUT_CAPTURE_DIR = os.path.join(RECORDING_DIR, "generated_native")

# EXTRA-button long-hold reset. A short press/release still toggles playback.
PLAYBACK_DELETE_HOLD_SECONDS = 3.0
# The gesture library lives beside this script, not inside RECORDING_DIR. Keep it
# by default so clearing recordings does not silently erase learned gestures.
DELETE_GESTURE_LIBRARY_ON_RECORDING_RESET = False

# Transition-to-recording-start settings (V23 distance/speed matched).
# The transition uses the SAME randomized cubic geometry as generated playback,
# but borrows a real normalized hand-speed profile from the available recordings.
# Duration is scaled from the borrowed profile according to the transition distance.
TRANSITION_MIN_DURATION = 0.20
TRANSITION_MAX_DURATION = 1.50
TRANSITION_FALLBACK_DURATION = SMOOTH_MOVE_DURATION

# V23: recording-start transitions borrow profiles from source gestures with
# reasonably similar traveled distance. More importantly, transition duration
# is NEVER hard-capped: duration = source_duration * generated_distance /
# source_traveled_distance. generated_move_to() subsequently compensates for
# Bezier arc length, so the selected source gesture's absolute speed scale is
# preserved even when the generated curve is longer than its endpoint distance.
TRANSITION_DISTANCE_RATIO_MIN = 0.50
TRANSITION_DISTANCE_RATIO_MAX = 2.00
TRANSITION_SPEED_BAND_LOW = 0.60
TRANSITION_SPEED_BAND_HIGH = 1.40

# V25 persistent per-gesture motion library.
# Stored beside this bridge script, independently of the recording directory.
GESTURE_LIBRARY_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "gesture_library.json",
)

# Only synthetic relocations use this library. Normal recorded replay keeps the
# motion profile belonging to its recorded segment.
MOTION_MODEL_ENABLED = True
MOTION_MODEL_CRUISE_FRACTION = 0.80
MOTION_MODEL_DV_QUANTILE = 0.75
MOTION_MODEL_MAX_SPEED_QUANTILE = 0.90
MOTION_MODEL_MIN_STEP_COUNTS = 0.25

# Ignore tiny accidental movements when building the persistent gesture library.
# This is traveled HID-count distance, not screen pixels.
GESTURE_LIBRARY_MIN_DISTANCE = 40.0

# V28: a library gesture defines timing + launch/arrival dynamics.
# Peak/cruise velocity is solved for the requested synthetic distance.
GESTURE_ACCEL_PHASE_FRACTION = 0.25
GESTURE_DECEL_PHASE_FRACTION = 0.25
GESTURE_MIN_PHASE_SAMPLES = 2
SYNTHETIC_DURATION_MIN_SCALE = 0.75
SYNTHETIC_DURATION_MAX_SCALE = 1.35

# Keep runtime output readable while testing profile selection.
PLAYBACK_VERBOSE_DEBUG = False



# ============================================================
# Device Setup
# ============================================================

mouse = InputDevice(MOUSE_DEVICE)
hid = open(HID_DEVICE, "wb", buffering=0)
os.makedirs(RECORDING_DIR, exist_ok=True)


# ============================================================
# Runtime State
# ============================================================

# Physical mouse state accumulated until Linux emits SYN_REPORT.
buttons = 0
dx = 0
dy = 0
wheel = 0

# ONE persistent coordinate system for recording and playback.
# The cursor location when the program starts is defined as (0, 0).
x = 0
y = 0
position_lock = threading.Lock()

# Recording state.
recording = False
record_file = None
record_writer = None
record_start_time = 0

# Playback state.
playback = False
playback_thread = None
playback_stop_event = threading.Event()

# Native generated-output capture state is declared near its helper functions.

# EXTRA-button hold state.
playback_button_lock = threading.Lock()
playback_button_held = False
playback_button_press_ns = 0
playback_button_delete_triggered = False
playback_button_generation = 0

# Calibration state
calibrating = False
calibration_step = 0
calibration_left = calibration_right = None
calibration_top = calibration_bottom = None
screen_calibrated = False
calibration_awaiting_reference = False
consume_reference_middle_release = False
SCREEN_MIN_X = SCREEN_MAX_X = None
SCREEN_MIN_Y = SCREEN_MAX_Y = None


# ============================================================
# Generic Helpers
# ============================================================

# V34.1: restored V32/V33 distance-aware selection constants
SYNTHETIC_DISTANCE_RATIO_MIN = 0.50
SYNTHETIC_DISTANCE_RATIO_MAX = 2.00

# V34.2: restored V32/V33 synthetic-motion configuration
SYNTHETIC_DURATION_DISTANCE_EXPONENT = 0.50
SYNTHETIC_MIN_COUNTS_PER_REPORT = 1.0

def get_next_recording_number():
    """Return the first unused recording_XXX.csv number."""
    number = 1

    while True:
        filename = os.path.join(
            RECORDING_DIR,
            f"recording_{number:03d}.csv"
        )

        if not os.path.exists(filename):
            return number

        number += 1


def clamp(value, minimum=-127, maximum=127):
    """Clamp a value to the signed 8-bit range used by this HID report."""
    return max(minimum, min(maximum, value))


def signed_byte(value):
    """Convert a signed movement value to the byte representation HID expects."""
    return value & 0xFF


recording_number = get_next_recording_number()


# ============================================================
# Calibration / Boundary Helpers
# ============================================================

def begin_calibration():
    global calibrating, calibration_step
    global calibration_left, calibration_right
    global calibration_top, calibration_bottom
    global calibration_awaiting_reference

    if recording or playback:
        print("Stop recording/playback before calibration.")
        return

    calibrating = True
    calibration_step = 0
    calibration_awaiting_reference = False
    calibration_left = calibration_right = None
    calibration_top = calibration_bottom = None

    print("\n=== SCREEN CALIBRATION ===")
    print("Move to LEFT limit, then LEFT CLICK.")


def capture_calibration_point():
    global calibrating, calibration_step, screen_calibrated
    global calibration_left, calibration_right
    global calibration_top, calibration_bottom
    global SCREEN_MIN_X, SCREEN_MAX_X, SCREEN_MIN_Y, SCREEN_MAX_Y
    global x, y, calibration_awaiting_reference

    with position_lock:
        cx, cy = x, y

    if calibration_step == 0:
        calibration_left = cx
        calibration_step = 1
        print(f"LEFT captured: {cx}")
        print("Move to RIGHT limit, then LEFT CLICK.")
        return

    if calibration_step == 1:
        calibration_right = cx
        calibration_step = 2
        print(f"RIGHT captured: {cx}")
        print("Move to TOP limit, then LEFT CLICK.")
        return

    if calibration_step == 2:
        calibration_top = cy
        calibration_step = 3
        print(f"TOP captured: {cy}")
        print("Move to BOTTOM limit, then LEFT CLICK.")
        return

    calibration_bottom = cy

    raw_min_x = min(calibration_left, calibration_right)
    raw_max_x = max(calibration_left, calibration_right)
    raw_min_y = min(calibration_top, calibration_bottom)
    raw_max_y = max(calibration_top, calibration_bottom)

    width = raw_max_x - raw_min_x
    height = raw_max_y - raw_min_y

    if width <= 0 or height <= 0:
        print("Invalid calibration; please run it again.")
        calibrating = False
        return

    # Normalize the chosen upper-left boundary to (0, 0).
    # This changes only our coordinate reference; it does not move Windows.
    with position_lock:
        x -= raw_min_x
        y -= raw_min_y

    SCREEN_MIN_X = 0
    SCREEN_MAX_X = width
    SCREEN_MIN_Y = 0
    SCREEN_MAX_Y = height
    screen_calibrated = True
    calibration_awaiting_reference = True
    calibration_step = 4

    print(f"BOTTOM captured: {calibration_bottom}")
    print("\n=== FINAL POSITION REFERENCE ===")
    print("Move the cursor to your repeatable reference point.")
    print("Press MIDDLE once to assign that point "
          f"({REFERENCE_X}, {REFERENCE_Y}).")
    print("Use the SAME screen point in later sessions to reuse recordings.")



def finish_calibration_reference():
    """Translate tracker AND screen limits; never move the actual cursor."""
    global x, y, SCREEN_MIN_X, SCREEN_MAX_X, SCREEN_MIN_Y, SCREEN_MAX_Y
    global calibrating, calibration_step, calibration_awaiting_reference
    global consume_reference_middle_release

    if not calibrating or not calibration_awaiting_reference:
        return

    with position_lock:
        offset_x = REFERENCE_X - x
        offset_y = REFERENCE_Y - y
        x += offset_x
        y += offset_y
        SCREEN_MIN_X += offset_x
        SCREEN_MAX_X += offset_x
        SCREEN_MIN_Y += offset_y
        SCREEN_MAX_Y += offset_y

    calibration_awaiting_reference = False
    calibrating = False
    calibration_step = 0
    consume_reference_middle_release = True
    print("\n=== CALIBRATION COMPLETE ===")
    print(f"Reference position: ({x}, {y})")
    print(f"X limits: {SCREEN_MIN_X} .. {SCREEN_MAX_X}")
    print(f"Y limits: {SCREEN_MIN_Y} .. {SCREEN_MAX_Y}")
    print(f"Active recording directory: {RECORDING_DIR}\n")

def limit_motion(current_x, current_y, requested_dx, requested_dy):
    """Return the portion of a relative move that stays inside calibration."""
    if not screen_calibrated or calibrating:
        return requested_dx, requested_dy

    tx = max(SCREEN_MIN_X, min(SCREEN_MAX_X, current_x + requested_dx))
    ty = max(SCREEN_MIN_Y, min(SCREEN_MAX_Y, current_y + requested_dy))
    return tx - current_x, ty - current_y


def clamp_target_to_screen(target_x, target_y):
    """Keep an absolute playback target inside the calibrated rectangle."""
    if not screen_calibrated:
        return target_x, target_y

    return (
        max(SCREEN_MIN_X, min(SCREEN_MAX_X, target_x)),
        max(SCREEN_MIN_Y, min(SCREEN_MAX_Y, target_y)),
    )



# ============================================================
# Recording Functions
# ============================================================

def start_recording():
    """Start a new CSV recording at the current persistent x/y position.

    Unlike the older implementation, x/y are NOT reset to (0, 0). This means
    recordings made at different times retain their spatial relationship.
    """
    global recording
    global record_file
    global record_writer
    global record_start_time
    global recording_number

    filename = os.path.join(
        RECORDING_DIR,
        f"recording_{recording_number:03d}.csv"
    )

    record_file = open(filename, "w", newline="")
    record_writer = csv.writer(record_file)

    record_writer.writerow([
        "time_ns",
        "dx",
        "dy",
        "wheel",
        "buttons",
        "x",
        "y"
    ])

    record_start_time = time.monotonic_ns()
    recording = True

    with position_lock:
        start_x = x
        start_y = y

    print()
    print(f"*** RECORDING STARTED: {filename} ***")
    print(f"    Tracked start: ({start_x}, {start_y})")
    print()

    recording_number += 1


def stop_recording():
    """Stop recording and preserve the exact terminal stationary duration.

    evdev may emit nothing while the mouse is motionless.  Therefore the old
    recorder lost any pause between the last input frame and SIDE-stop.  Before
    closing, V22 writes one explicit zero-motion boundary row at stop time.
    """
    global recording
    global record_file
    global record_writer

    # Capture stop time/position BEFORE disabling the writer.
    stop_timestamp = time.monotonic_ns() - record_start_time
    with position_lock:
        end_x = x
        end_y = y

    if recording and record_writer is not None:
        record_writer.writerow([
            stop_timestamp, 0, 0, 0, buttons, end_x, end_y
        ])

    recording = False

    completed_recording = None
    if record_file is not None:
        completed_recording = record_file.name
        record_file.flush()
        record_file.close()

    if completed_recording is not None:
        _ingest_recording_into_gesture_library(completed_recording)

    record_file = None
    record_writer = None

    print()
    print("*** RECORDING STOPPED ***")
    print(f"    Tracked end: ({end_x}, {end_y})")
    print()


# ============================================================
# Playback Helpers
# ============================================================

def get_recording_start(filename):
    """Return the tracked x/y position associated with a recording's start.

    The first recorded SYN_REPORT may contain movement, so its x/y describes
    the position AFTER that row's dx/dy. Subtracting that row's dx/dy recovers
    the position immediately before the first recorded movement.
    """
    with open(filename, "r", newline="") as file:
        reader = csv.DictReader(file)
        first_row = next(reader, None)

    if first_row is None:
        raise ValueError(f"Recording is empty: {filename}")

    required = {"dx", "dy", "x", "y"}
    if not required.issubset(first_row):
        raise ValueError(
            f"Recording uses an incompatible CSV format: {filename}"
        )

    start_x = int(first_row["x"]) - int(first_row["dx"])
    start_y = int(first_row["y"]) - int(first_row["dy"])

    return start_x, start_y


def _send_transition_points(points):
    """Send absolute path points as relative HID reports."""
    global x, y

    with position_lock:
        previous_x = x
        previous_y = y

    for point_x, point_y in points:
        if playback_stop_event.is_set():
            return False

        step_dx = point_x - previous_x
        step_dy = point_y - previous_y

        step_dx, step_dy = limit_motion(
            previous_x, previous_y, step_dx, step_dy
        )
        hid_dx = clamp(step_dx)
        hid_dy = clamp(step_dy)

        hid.write(bytes([
            0,
            signed_byte(hid_dx),
            signed_byte(hid_dy),
            0
        ]))

        with position_lock:
            x += hid_dx
            y += hid_dy
            previous_x = x
            previous_y = y

        if playback_stop_event.wait(SMOOTH_MOVE_INTERVAL):
            return False

    return True


def smooth_move_linear(target_x, target_y, duration=SMOOTH_MOVE_DURATION):
    """Move to the target along a straight line."""
    with position_lock:
        start_x = x
        start_y = y

    total_dx = target_x - start_x
    total_dy = target_y - start_y

    steps = max(
        1,
        int(duration / SMOOTH_MOVE_INTERVAL),
        (abs(total_dx) + 126) // 127,
        (abs(total_dy) + 126) // 127
    )

    print(
        f"Linear move: ({start_x}, {start_y}) -> "
        f"({target_x}, {target_y})"
    )

    points = []
    for step in range(1, steps + 1):
        t = step / steps
        point_x = round(start_x + total_dx * t)
        point_y = round(start_y + total_dy * t)
        points.append((point_x, point_y))

    return _send_transition_points(points)


def smooth_move_curve(target_x, target_y, duration=SMOOTH_MOVE_DURATION):
    """Move to the target along a quadratic Bezier curve."""
    with position_lock:
        start_x = x
        start_y = y

    total_dx = target_x - start_x
    total_dy = target_y - start_y
    distance = math.hypot(total_dx, total_dy)

    if distance == 0:
        return True

    steps = max(
        1,
        int(duration / SMOOTH_MOVE_INTERVAL),
        (abs(total_dx) + 126) // 127,
        (abs(total_dy) + 126) // 127,
        int(distance / 100) + 1
    )

    midpoint_x = (start_x + target_x) / 2
    midpoint_y = (start_y + target_y) / 2

    # Perpendicular unit vector determines which way the arc bows.
    perpendicular_x = -total_dy / distance
    perpendicular_y = total_dx / distance

    curve_offset = distance * CURVE_AMOUNT
    control_x = midpoint_x + perpendicular_x * curve_offset
    control_y = midpoint_y + perpendicular_y * curve_offset

    print(
        f"Curved move: ({start_x}, {start_y}) -> "
        f"({target_x}, {target_y})"
    )

    points = []
    for step in range(1, steps + 1):
        t = step / steps
        one_minus_t = 1.0 - t

        point_x = round(
            one_minus_t**2 * start_x
            + 2 * one_minus_t * t * control_x
            + t**2 * target_x
        )
        point_y = round(
            one_minus_t**2 * start_y
            + 2 * one_minus_t * t * control_y
            + t**2 * target_y
        )

        points.append((point_x, point_y))

    return _send_transition_points(points)


def choose_random_circular_offset(max_radius=PLAYBACK_START_OFFSET_MAX):
    """Choose a fresh non-deterministic integer offset inside a circle."""
    if max_radius <= 0:
        return 0, 0

    rng = secrets.SystemRandom()

    # sqrt(U) makes points uniform over the circle's area.
    radius = max_radius * math.sqrt(rng.random())
    angle = rng.uniform(0.0, 2.0 * math.pi)

    return (
        round(radius * math.cos(angle)),
        round(radius * math.sin(angle))
    )


def smooth_move_to(target_x, target_y, duration=SMOOTH_MOVE_DURATION):
    """Use the transition style selected by SMOOTH_MOVE_MODE."""
    if SMOOTH_MOVE_MODE == "linear":
        return smooth_move_linear(target_x, target_y, duration)

    if SMOOTH_MOVE_MODE == "curve":
        return smooth_move_curve(target_x, target_y, duration)

    raise ValueError(
        f"Unknown SMOOTH_MOVE_MODE: {SMOOTH_MOVE_MODE!r}. "
        'Use "linear" or "curve".'
    )


def _build_speed_profile_details(segment_rows, segment_start_time_ns):
    """Build a motion profile and keep stationary edge time separate.

    V16 treats leading/trailing zero-XY reports as dwell rather than baking
    them into the normalized movement profile. Zero reports *inside* the
    active movement are retained because they are part of the sampled motion.

    Returns:
        profile: (active_elapsed_seconds, normalized_distance)
        active_duration: time occupied by the active movement
        leading_dwell: stationary time before the first moving report
        trailing_dwell: stationary time after the last moving report
    """
    if not segment_rows:
        return [(0.0, 0.0)], 0.0, 0.0, 0.0

    times = [
        max(0.0, (int(r["time_ns"]) - segment_start_time_ns) / 1e9)
        for r in segment_rows
    ]
    moving = [
        (int(r["dx"]) != 0 or int(r["dy"]) != 0)
        for r in segment_rows
    ]
    total_duration = times[-1]

    if not TRIM_PROFILE_STATIONARY_EDGES or not any(moving):
        samples = [(0.0, 0.0)]
        cumulative = 0.0
        for row, elapsed in zip(segment_rows, times):
            cumulative += math.hypot(int(row["dx"]), int(row["dy"]))
            samples.append((elapsed, cumulative))
        if cumulative <= 0.0:
            return [(0.0, 0.0), (total_duration, 1.0)], total_duration, 0.0, 0.0
        return (
            [(t, d / cumulative) for t, d in samples],
            total_duration, 0.0, 0.0
        )

    first = next(i for i, flag in enumerate(moving) if flag)
    last = len(moving) - 1 - next(
        i for i, flag in enumerate(reversed(moving)) if flag
    )

    # The first moving report represents motion accumulated since the preceding
    # report boundary. Use the previous timestamp as the active interval start
    # when available so we do not throw away that first report's time.
    active_start = times[first - 1] if first > 0 else 0.0
    active_end = times[last]
    active_duration = max(0.0, active_end - active_start)
    leading_dwell = max(0.0, active_start)
    trailing_dwell = max(0.0, total_duration - active_end)

    samples = [(0.0, 0.0)]
    cumulative = 0.0
    for i in range(first, last + 1):
        row = segment_rows[i]
        cumulative += math.hypot(int(row["dx"]), int(row["dy"]))
        samples.append((max(0.0, times[i] - active_start), cumulative))

    if cumulative <= 0.0 or active_duration <= 0.0:
        return [(0.0, 0.0), (active_duration, 1.0)], active_duration, leading_dwell, trailing_dwell

    profile = [(t, d / cumulative) for t, d in samples]
    # Guarantee an exact terminal point at the active-motion end.
    if profile[-1][0] < active_duration:
        profile.append((active_duration, 1.0))
    else:
        profile[-1] = (active_duration, 1.0)

    return profile, active_duration, leading_dwell, trailing_dwell


def _build_speed_profile(segment_rows, segment_start_time_ns):
    """Compatibility wrapper returning only the active movement profile."""
    profile, _, _, _ = _build_speed_profile_details(
        segment_rows, segment_start_time_ns
    )
    return profile


def _make_generated_segment(
    segment_rows,
    segment_start_time_ns,
    held_buttons,
    end_buttons,
    offset_x,
    offset_y,
    force_loiter=False,
):
    """Build one generated-playback segment from a contiguous row range."""
    if not segment_rows:
        return None

    last = segment_rows[-1]
    end_time_ns = int(last["time_ns"])
    end_x = int(last["x"]) + offset_x
    end_y = int(last["y"]) + offset_y
    end_x, end_y = clamp_target_to_screen(end_x, end_y)

    total_duration = max(
        0.0, (end_time_ns - segment_start_time_ns) / 1e9
    )
    speed_profile, active_motion_duration, leading_dwell, trailing_dwell = (
        _build_speed_profile_details(segment_rows, segment_start_time_ns)
    )

    return {
        "x": end_x,
        "y": end_y,
        "duration": total_duration,
        "active_motion_duration": active_motion_duration,
        "leading_motion_dwell": leading_dwell,
        "trailing_motion_dwell": trailing_dwell,
        "held_buttons": held_buttons,
        "end_buttons": end_buttons,
        "speed_profile": speed_profile,
        "motion_distance": sum(
            math.hypot(int(r["dx"]), int(r["dy"]))
            for r in segment_rows
        ),
        "sample_count": len(segment_rows),
        "force_loiter": force_loiter,
        "after_loiter": False,
    }


def _split_no_button_interval_for_loiter(
    interval_rows,
    interval_start_time_ns,
    end_buttons,
    offset_x,
    offset_y,
):
    """Split a button-up interval around long stationary runs.

    V9 only looked at the total motion of an entire button-state interval.
    Example:

        release -> 5 s stationary -> move to next click

    was one segment, so the later movement made motion_distance nonzero and the
    5 s stationary portion was never recognized as a loiter.

    V10 finds stationary runs *inside* the interval and emits them as their own
    generated segments.
    """
    if not interval_rows:
        return []

    result = []
    cursor = 0
    segment_start_ns = interval_start_time_ns
    n = len(interval_rows)
    next_motion_after_loiter = False

    while cursor < n:
        # Search for the next candidate stationary run. We allow only a tiny
        # cumulative amount of recorded pointer motion across the whole run.
        found = None

        for candidate_start in range(cursor, n):
            candidate_start_ns = (
                segment_start_ns
                if candidate_start == cursor
                else int(interval_rows[candidate_start - 1]["time_ns"])
            )

            cumulative = 0.0
            for candidate_end in range(candidate_start, n):
                row = interval_rows[candidate_end]
                cumulative += math.hypot(
                    int(row["dx"]), int(row["dy"])
                )

                if cumulative > LOITER_MOTION_THRESHOLD:
                    break

                duration = (
                    int(row["time_ns"]) - candidate_start_ns
                ) / 1e9

                if duration > LOITER_TIMEOUT:
                    # Extend this qualifying stationary run as far as possible
                    # while staying within the cumulative-motion threshold.
                    final_end = candidate_end
                    final_cumulative = cumulative

                    for j in range(candidate_end + 1, n):
                        r = interval_rows[j]
                        next_cumulative = final_cumulative + math.hypot(
                            int(r["dx"]), int(r["dy"])
                        )
                        if next_cumulative > LOITER_MOTION_THRESHOLD:
                            break
                        final_cumulative = next_cumulative
                        final_end = j

                    found = (candidate_start, final_end)
                    break

            if found is not None:
                break

        if found is None:
            tail = interval_rows[cursor:]
            seg = _make_generated_segment(
                tail,
                segment_start_ns,
                0,
                end_buttons,
                offset_x,
                offset_y,
                force_loiter=False,
            )
            if seg:
                if next_motion_after_loiter:
                    seg["after_loiter"] = True
                    next_motion_after_loiter = False
                result.append(seg)
            break

        loiter_start, loiter_end = found

        # Motion before the stationary run.
        if loiter_start > cursor:
            before = interval_rows[cursor:loiter_start]
            before_end_ns = int(before[-1]["time_ns"])
            seg = _make_generated_segment(
                before,
                segment_start_ns,
                0,
                0,  # internal split; button state does not change here
                offset_x,
                offset_y,
                force_loiter=False,
            )
            if seg:
                if next_motion_after_loiter:
                    seg["after_loiter"] = True
                    next_motion_after_loiter = False
                result.append(seg)
            segment_start_ns = before_end_ns

        # The stationary run itself.
        loiter_rows = interval_rows[loiter_start:loiter_end + 1]
        is_final_piece = (loiter_end == n - 1)
        seg = _make_generated_segment(
            loiter_rows,
            segment_start_ns,
            0,
            end_buttons if is_final_piece else 0,
            offset_x,
            offset_y,
            force_loiter=True,
        )
        if seg:
            result.append(seg)

        segment_start_ns = int(loiter_rows[-1]["time_ns"])
        cursor = loiter_end + 1

        if cursor >= n:
            break

        # The next generated motion begins from the randomized loiter position,
        # not from the source recording's old stationary coordinate. Mark it so
        # playback can derive a fresh duration from the actual new distance.
        next_motion_after_loiter = True

    return result


def _insert_stationary_gap_boundaries(rows):
    """Represent long evdev-silent intervals explicitly for loiter parsing.

    Linux may provide no SYN_REPORT rows while the mouse is stationary.  If two
    recorded rows are separated by more than the measured active-report timing
    limit, insert a zero-motion row immediately before the later event.  Its
    position/button state is inherited from the previous row.

    This does NOT invent the gap duration: both endpoints come from recorded
    timestamps.  It only gives the segment parser an explicit stationary
    boundary so a multi-second hover can become a loiter segment.
    """
    if len(rows) < 2:
        return rows

    gap_limit_ns = int(_measured_active_report_gap_limit() * 1e9)
    expanded = [rows[0]]

    for current in rows[1:]:
        previous = expanded[-1]
        prev_t = int(previous["time_ns"])
        cur_t = int(current["time_ns"])
        gap_ns = cur_t - prev_t

        if gap_ns > gap_limit_ns:
            # Keep the later real movement event intact.  Place the synthetic
            # stationary endpoint one nominal active interval before it so the
            # long silence belongs to the preceding stationary interval.
            boundary_t = max(prev_t, cur_t - gap_limit_ns)
            synthetic = dict(previous)
            synthetic["time_ns"] = str(boundary_t)
            synthetic["dx"] = "0"
            synthetic["dy"] = "0"
            synthetic["wheel"] = "0"
            expanded.append(synthetic)

        expanded.append(current)

    return expanded


def load_generated_segments(filename, offset_x=0, offset_y=0):
    """Convert a recording into generated-motion segments.

    V10 preserves button-transition segmentation but additionally splits
    button-up intervals around long stationary runs, so loiter detection is
    independent of whether movement occurs later before the next click.
    """
    with open(filename, "r", newline="") as file:
        rows = list(csv.DictReader(file))

    if rows:
        rows = _insert_stationary_gap_boundaries(rows)

    if not rows:
        raise ValueError(f"Recording is empty: {filename}")

    first = rows[0]
    start_x = int(first["x"]) - int(first["dx"]) + offset_x
    start_y = int(first["y"]) - int(first["dy"]) + offset_y
    start_x, start_y = clamp_target_to_screen(start_x, start_y)

    segment_start_time_ns = 0
    held_buttons = 0
    segment_rows = []
    segments = []

    def append_interval(rows_for_interval, start_ns, held, end_buttons):
        if not rows_for_interval:
            return

        if held == 0 and LOITER_ENABLED:
            segments.extend(
                _split_no_button_interval_for_loiter(
                    rows_for_interval,
                    start_ns,
                    end_buttons,
                    offset_x,
                    offset_y,
                )
            )
        else:
            seg = _make_generated_segment(
                rows_for_interval,
                start_ns,
                held,
                end_buttons,
                offset_x,
                offset_y,
            )
            if seg:
                segments.append(seg)

    for row in rows:
        segment_rows.append(row)
        row_buttons = int(row["buttons"])

        if row_buttons != held_buttons:
            end_time_ns = int(row["time_ns"])
            append_interval(
                segment_rows,
                segment_start_time_ns,
                held_buttons,
                row_buttons,
            )
            segment_start_time_ns = end_time_ns
            held_buttons = row_buttons
            segment_rows = []

    if segment_rows:
        append_interval(
            segment_rows,
            segment_start_time_ns,
            held_buttons,
            int(segment_rows[-1]["buttons"]),
        )

    return start_x, start_y, segments



def _random_cubic_controls(start_x, start_y, target_x, target_y):
    """Create constrained randomized cubic-Bezier control points."""
    rng = secrets.SystemRandom()
    vx = target_x - start_x
    vy = target_y - start_y
    distance = math.hypot(vx, vy)

    if distance == 0:
        return (start_x, start_y), (target_x, target_y)

    px = -vy / distance
    py = vx / distance

    # Longitudinal control placement may now extend outside the start-target
    # interval. This permits temporary movement away from the target on either
    # axis while retaining the exact endpoint.
    j1 = rng.uniform(-GENERATED_CONTROL_JITTER, GENERATED_CONTROL_JITTER)
    j2 = rng.uniform(-GENERATED_CONTROL_JITTER, GENERATED_CONTROL_JITTER)
    a1 = rng.uniform(
        GENERATED_CONTROL1_LONGITUDINAL_MIN,
        GENERATED_CONTROL1_LONGITUDINAL_MAX,
    ) + 0.15 * j1
    a2 = rng.uniform(
        GENERATED_CONTROL2_LONGITUDINAL_MIN,
        GENERATED_CONTROL2_LONGITUDINAL_MAX,
    ) + 0.15 * j2

    max_offset = distance * rng.uniform(GENERATED_CURVE_MIN, GENERATED_CURVE_MAX)
    off1 = rng.uniform(-max_offset, max_offset)
    off2 = rng.uniform(-max_offset, max_offset)

    c1x = start_x + vx * a1 + px * off1
    c1y = start_y + vy * a1 + py * off1
    c2x = start_x + vx * a2 + px * off2
    c2y = start_y + vy * a2 + py * off2

    if screen_calibrated:
        c1x = max(SCREEN_MIN_X, min(SCREEN_MAX_X, c1x))
        c1y = max(SCREEN_MIN_Y, min(SCREEN_MAX_Y, c1y))
        c2x = max(SCREEN_MIN_X, min(SCREEN_MAX_X, c2x))
        c2y = max(SCREEN_MIN_Y, min(SCREEN_MAX_Y, c2y))

    return (c1x, c1y), (c2x, c2y)


def _bezier_point(t, start_x, start_y, c1, c2, target_x, target_y):
    omt = 1.0 - t
    return (
        omt**3 * start_x + 3.0 * omt**2 * t * c1[0]
        + 3.0 * omt * t**2 * c2[0] + t**3 * target_x,
        omt**3 * start_y + 3.0 * omt**2 * t * c1[1]
        + 3.0 * omt * t**2 * c2[1] + t**3 * target_y,
    )


def _build_bezier_arc_table(start_x, start_y, c1, c2, target_x, target_y):
    """Approximate arc length so speed profile follows distance, not Bezier t."""
    table = [(0.0, 0.0)]
    previous = (start_x, start_y)
    total = 0.0

    for i in range(1, GENERATED_ARC_TABLE_STEPS + 1):
        t = i / GENERATED_ARC_TABLE_STEPS
        point = _bezier_point(t, start_x, start_y, c1, c2, target_x, target_y)
        total += math.hypot(point[0] - previous[0], point[1] - previous[1])
        table.append((t, total))
        previous = point

    if total <= 0.0:
        return [(0.0, 0.0), (1.0, 1.0)]

    return [(t, length / total) for t, length in table]


def _interpolate_profile(profile, elapsed):
    """Return recorded normalized path progress at elapsed seconds."""
    if not profile:
        return 1.0
    if elapsed <= profile[0][0]:
        return profile[0][1]
    if elapsed >= profile[-1][0]:
        return profile[-1][1]

    for i in range(1, len(profile)):
        t1, p1 = profile[i]
        if elapsed <= t1:
            t0, p0 = profile[i - 1]
            if t1 <= t0:
                return p1
            a = (elapsed - t0) / (t1 - t0)
            return p0 + a * (p1 - p0)
    return 1.0


def _progress_to_bezier_t(arc_table, progress):
    """Map normalized arc-length progress to Bezier parameter t."""
    progress = max(0.0, min(1.0, progress))
    for i in range(1, len(arc_table)):
        t1, p1 = arc_table[i]
        if progress <= p1:
            t0, p0 = arc_table[i - 1]
            if p1 <= p0:
                return t1
            a = (progress - p0) / (p1 - p0)
            return t0 + a * (t1 - t0)
    return 1.0


def _profile_duration(profile):
    """Return the recorded duration represented by a speed profile."""
    if not profile:
        return 0.0
    return max(0.0, float(profile[-1][0]))


def _normalize_profile_time(profile):
    """Convert a real recorded profile to normalized time 0..1.

    Distance progress is left untouched.  This preserves the shape of the
    recorded acceleration/deceleration while allowing it to be stretched to a
    different transition duration.
    """
    if not profile:
        return [(0.0, 0.0), (1.0, 1.0)]

    duration = _profile_duration(profile)
    if duration <= 0.0:
        return [(0.0, 0.0), (1.0, 1.0)]

    normalized = [(max(0.0, min(1.0, t / duration)), p) for t, p in profile]
    if normalized[0][0] > 0.0:
        normalized.insert(0, (0.0, 0.0))
    if normalized[-1][0] < 1.0:
        normalized.append((1.0, 1.0))
    return normalized


def _scale_normalized_profile(profile, duration):
    """Stretch a normalized-time profile to a requested duration."""
    duration = max(0.0, duration)
    return [(u * duration, p) for u, p in profile]


def _vary_duration_and_profile(duration, profile):
    """Apply small random timing variation without changing motion shape.

    The recorded profile is first normalized in time to 0..1, then stretched
    to the randomized duration. Distance-progress values are left untouched,
    so the original acceleration/deceleration pattern is preserved.
    """
    duration = max(0.0, float(duration))
    if duration <= 0.0 or GENERATED_TIMING_VARIATION <= 0.0:
        return duration, profile

    rng = secrets.SystemRandom()
    variation = max(0.0, min(0.95, GENERATED_TIMING_VARIATION))
    scale = rng.uniform(1.0 - variation, 1.0 + variation)
    new_duration = duration * scale

    normalized = _normalize_profile_time(profile)
    varied_profile = _scale_normalized_profile(normalized, new_duration)
    return new_duration, varied_profile


def _vary_button_dwell(duration):
    """Randomize a stationary button-hold interval around its real duration.

    This is intentionally independent of generated movement timing. Long holds
    above BUTTON_DWELL_MAX_DURATION are left unchanged because they are more
    likely intentional holds than ordinary click dwell.
    """
    duration = max(0.0, float(duration))
    if duration <= 0.0 or BUTTON_DWELL_VARIATION <= 0.0:
        return duration
    if duration > BUTTON_DWELL_MAX_DURATION:
        return duration

    rng = secrets.SystemRandom()
    variation = max(0.0, min(0.95, BUTTON_DWELL_VARIATION))
    scale = rng.uniform(1.0 - variation, 1.0 + variation)
    return max(BUTTON_DWELL_MIN_DURATION, duration * scale)


def _is_button_dwell_segment(segment):
    """True for a held-button interval with essentially no pointer motion.

    A drag has appreciable cumulative motion and therefore stays on the normal
    generated-motion timing path. Tiny hand tremor during a click is tolerated.
    """
    return (
        int(segment.get("held_buttons", 0)) != 0
        and float(segment.get("motion_distance", 0.0))
            <= BUTTON_DWELL_MOTION_THRESHOLD
    )


def _collect_real_motion_profiles():
    """Collect usable speed profiles from all current recording CSVs.

    Each candidate is a real motion interval from a recording.  The return
    value also stores the interval's straight-line displacement so transition
    duration can be scaled sensibly when that profile is reused.
    """
    candidates = []

    for filename in sorted(glob.glob(os.path.join(RECORDING_DIR, "recording_*.csv"))):
        try:
            start_x, start_y, segments = load_generated_segments(filename, 0, 0)
        except (OSError, ValueError, KeyError):
            continue

        previous_x, previous_y = start_x, start_y
        for segment in segments:
            endpoint_distance = math.hypot(
                segment["x"] - previous_x,
                segment["y"] - previous_y,
            )
            traveled_distance = float(segment.get("motion_distance", 0.0))
            duration = max(0.0, float(segment.get("active_motion_duration", segment["duration"])))
            profile = segment["speed_profile"]

            # Only borrow profiles from genuine source movement. Never use a
            # synthetic loiter split or a stationary button dwell as a speed
            # reference. Use cumulative traveled distance as the velocity scale;
            # endpoint displacement can badly underestimate curved motion.
            genuine_motion = (
                not bool(segment.get("force_loiter", False))
                and not _is_button_dwell_segment(segment)
                and traveled_distance >= 2.0
                and endpoint_distance >= 2.0
                and duration > 0.0
                and len(profile) >= 2
            )

            if genuine_motion:
                candidates.append({
                    "profile": _normalize_profile_time(profile),
                    "duration": duration,
                    "distance": traveled_distance,
                    "endpoint_distance": endpoint_distance,
                    "source": os.path.basename(filename),
                })

            previous_x, previous_y = segment["x"], segment["y"]

    return candidates


def _quantile(values, q, default):
    values = sorted(float(v) for v in values if math.isfinite(float(v)))
    if not values:
        return float(default)
    q = max(0.0, min(1.0, float(q)))
    index = q * (len(values) - 1)
    lo = int(math.floor(index))
    hi = int(math.ceil(index))
    if lo == hi:
        return values[lo]
    f = index - lo
    return values[lo] * (1.0 - f) + values[hi] * f


def _gesture_fingerprint(distance, duration, profile):
    """Stable hash used only to prevent importing the same gesture twice."""
    payload = {
        "distance": round(float(distance), 6),
        "duration": round(float(duration), 9),
        "profile": [
            [round(float(t), 9), round(float(p), 9)]
            for t, p in profile
        ],
    }
    raw = json.dumps(payload, separators=(",", ":"), sort_keys=True).encode("utf-8")
    return hashlib.sha256(raw).hexdigest()


def _candidate_to_library_gesture(candidate):
    """Convert one physical gesture into timing + start/stop dynamic constraints."""
    distance = max(0.0, float(candidate["distance"]))
    duration = max(1e-9, float(candidate["duration"]))
    profile = [(float(t), float(p)) for t, p in candidate["profile"]]

    dt_samples = []
    velocity_samples = []
    prev_t = None
    prev_p = None
    for t, p in profile:
        if prev_t is not None:
            dt = t - prev_t
            dp = p - prev_p
            if dt > 1e-6:
                dt_samples.append(dt)
                velocity_samples.append(max(0.0, dp * distance / dt))
        prev_t, prev_p = t, p

    if len(dt_samples) < 2 or len(velocity_samples) < 2:
        return None

    # Acceleration is estimated from report-to-report scalar speed change.
    accel_samples = []
    for i in range(1, len(velocity_samples)):
        # velocity[i] belongs to interval i; use the current interval dt.
        dt = max(1e-6, dt_samples[min(i, len(dt_samples) - 1)])
        accel_samples.append((velocity_samples[i] - velocity_samples[i - 1]) / dt)

    phase_n_acc = max(
        GESTURE_MIN_PHASE_SAMPLES,
        int(math.ceil(len(accel_samples) * GESTURE_ACCEL_PHASE_FRACTION)),
    )
    phase_n_dec = max(
        GESTURE_MIN_PHASE_SAMPLES,
        int(math.ceil(len(accel_samples) * GESTURE_DECEL_PHASE_FRACTION)),
    )
    phase_n_acc = min(phase_n_acc, len(accel_samples))
    phase_n_dec = min(phase_n_dec, len(accel_samples))

    start_region = [a for a in accel_samples[:phase_n_acc] if a > 0.0]
    stop_region = [-a for a in accel_samples[-phase_n_dec:] if a < 0.0]

    # Robust representative magnitudes. If one edge is too flat/noisy, fall back
    # to the positive/negative acceleration samples from the whole gesture.
    if not start_region:
        start_region = [a for a in accel_samples if a > 0.0]
    if not stop_region:
        stop_region = [-a for a in accel_samples if a < 0.0]

    start_accel = _quantile(start_region, 0.50, 1.0)
    stop_decel = _quantile(stop_region, 0.50, start_accel)

    # V45: the gesture library supplies a physical dynamics envelope rather
    # than a source timeline. Robust upper quantiles avoid defining the entire
    # gesture from one quantized HID report while retaining its characteristic
    # high-speed/high-acceleration capability.
    peak_speed = _quantile(velocity_samples, 0.95, max(velocity_samples))
    accel_magnitudes = [abs(a) for a in accel_samples if math.isfinite(a)]
    accel_limit = _quantile(accel_magnitudes, 0.90, max(start_accel, stop_decel))

    # V46: retain the physical scalar-speed SHAPE as a compact normalized
    # template.  Geometry is deliberately discarded: the synthetic Bezier
    # supplies direction, while this profile supplies tangential dynamics.
    # A small symmetric moving average suppresses one-report HID quantization
    # spikes without turning the gesture into an analytic minimum-jerk curve.
    smooth = []
    radius = 3
    for i in range(len(velocity_samples)):
        lo = max(0, i - radius)
        hi = min(len(velocity_samples), i + radius + 1)
        smooth.append(sum(velocity_samples[lo:hi]) / (hi - lo))

    vmax = max(smooth) if smooth else 0.0
    if vmax <= 0.0:
        return None

    # Progress coordinate is normalized elapsed time of each velocity interval.
    cumulative_t = 0.0
    speed_shape = [[0.0, 0.0]]
    total_t = sum(dt_samples)
    for i, v in enumerate(smooth):
        cumulative_t += dt_samples[i]
        u = min(1.0, cumulative_t / max(total_t, 1e-9))
        speed_shape.append([u, max(0.0, min(1.0, v / vmax))])
    speed_shape[-1] = [1.0, 0.0]

    return {
        "fingerprint": _gesture_fingerprint(distance, duration, profile),
        "distance": distance,
        "duration": duration,
        "dt": dt_samples,
        "speed_shape": speed_shape,
        "start_accel": max(1.0, float(start_accel)),
        "stop_decel": max(1.0, float(stop_decel)),
        "peak_speed": max(1.0, float(peak_speed)),
        "accel_limit": max(1.0, float(accel_limit)),
    }



def _load_gesture_library():
    try:
        with open(GESTURE_LIBRARY_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
    except FileNotFoundError:
        return []
    except (OSError, ValueError, TypeError) as exc:
        print(f"Gesture library load failed: {exc}")
        return []

    if not isinstance(data, list):
        print("Gesture library ignored: expected a JSON list.")
        return []

    return [
        row for row in data
        if isinstance(row, dict)
        and row.get("fingerprint")
        and isinstance(row.get("dt"), list) and row["dt"]
        and float(row.get("start_accel", 0.0)) > 0.0
        and float(row.get("stop_decel", 0.0)) > 0.0
        and float(row.get("peak_speed", 0.0)) > 0.0
        and float(row.get("accel_limit", 0.0)) > 0.0
        and isinstance(row.get("speed_shape"), list) and len(row["speed_shape"]) >= 3
    ]


def _save_gesture_library(library):
    tmp = GESTURE_LIBRARY_FILE + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(library, f, indent=2)
        f.write("\n")
    os.replace(tmp, GESTURE_LIBRARY_FILE)


def _ingest_recording_into_gesture_library(filename):
    """Append genuine gestures from ONE completed recording, skipping duplicates."""
    library = _load_gesture_library()
    known = {row["fingerprint"] for row in library}

    try:
        start_x, start_y, segments = load_generated_segments(filename, 0, 0)
    except (OSError, ValueError, KeyError) as exc:
        print(f"Gesture library: could not ingest {filename}: {exc}")
        return 0

    added = 0
    previous_x, previous_y = start_x, start_y

    for segment in segments:
        endpoint_distance = math.hypot(
            segment["x"] - previous_x,
            segment["y"] - previous_y,
        )
        traveled_distance = float(segment.get("motion_distance", 0.0))
        duration = max(
            0.0,
            float(segment.get("active_motion_duration", segment["duration"])),
        )
        profile = segment["speed_profile"]

        genuine_motion = (
            not bool(segment.get("force_loiter", False))
            and not _is_button_dwell_segment(segment)
            and traveled_distance >= GESTURE_LIBRARY_MIN_DISTANCE
            and endpoint_distance >= GESTURE_LIBRARY_MIN_DISTANCE
            and duration > 0.0
            and len(profile) >= 2
        )

        if genuine_motion:
            # Keep the profile timestamps in REAL ELAPSED SECONDS here.
            # _candidate_to_library_gesture() derives per-report dt and acceleration
            # directly from these timestamps. Normalizing time to 0..1 here would
            # turn dimensionless time fractions into apparent seconds when the
            # gesture is later reused for synthetic transitions.
            candidate = {
                "profile": profile,
                "duration": duration,
                "distance": traveled_distance,
                "endpoint_distance": endpoint_distance,
                "source": os.path.basename(filename),
            }
            row = _candidate_to_library_gesture(candidate)
            if row is not None and row["fingerprint"] not in known:
                library.append(row)
                known.add(row["fingerprint"])
                added += 1

        previous_x, previous_y = segment["x"], segment["y"]

    if added:
        try:
            _save_gesture_library(library)
        except OSError as exc:
            print(f"Gesture library save failed: {exc}")
            return 0

    print(
        f"Gesture library: ingested {os.path.basename(filename)} -> "
        f"+{added} new profile(s), {len(library)} total"
    )
    return added


def _update_gesture_library_from_recordings():
    """Append newly observed gestures; deleting source CSVs does not delete them."""
    library = _load_gesture_library()
    known = {row["fingerprint"] for row in library}
    added = 0

    for candidate in _collect_real_motion_profiles():
        row = _candidate_to_library_gesture(candidate)
        if row is None or row["fingerprint"] in known:
            continue
        library.append(row)
        known.add(row["fingerprint"])
        added += 1

    if added:
        try:
            _save_gesture_library(library)
            print(
                f"Gesture library: +{added} new, {len(library)} total "
                f"({GESTURE_LIBRARY_FILE})"
            )
        except OSError as exc:
            print(f"Gesture library save failed: {exc}")
    else:
        print(f"Gesture library: {len(library)} profile(s), no new gestures.")

    return library


def _log_transition_diagnostic(message):
    """V34: intentionally disabled during playback timing tests."""
    return


def _build_motion_profile_from_gesture(distance, gesture, return_diagnostics=False, forced_duration=None):
    """V46: reuse a recorded hand-speed shape on a new synthetic path.

    The gesture contributes a normalized scalar-speed shape plus a robust
    physical peak-speed target.  The requested move distance determines the
    duration from the area under that shape.  Integrating the scaled speed
    gives normalized ARC-LENGTH progress; generated_move_to() maps that
    progress through the Bezier arc table, so the Bezier controls direction
    while the recorded gesture controls tangential dynamics.
    """
    distance = max(0.0, float(distance))
    if distance <= 0.0:
        result = (0.0, [(0.0, 0.0)])
        return (*result, {}) if return_diagnostics else result

    shape = gesture.get("speed_shape", [])
    if not isinstance(shape, list) or len(shape) < 3:
        # Old V45 libraries do not contain enough information to reconstruct
        # hand acceleration shape. Rebuild the library from physical recordings.
        raise ValueError(
            "Gesture library entry has no V46 speed_shape; delete gesture_library.json "
            "once and rebuild it from physical recordings."
        )

    points = []
    for row in shape:
        try:
            u, q = float(row[0]), float(row[1])
        except (TypeError, ValueError, IndexError):
            continue
        if math.isfinite(u) and math.isfinite(q):
            points.append((max(0.0, min(1.0, u)), max(0.0, min(1.0, q))))
    points.sort()
    if len(points) < 3:
        raise ValueError("Gesture speed_shape is invalid.")

    peak_speed = max(1.0, float(gesture.get("peak_speed", 1.0)))
    dts = [float(v) for v in gesture.get("dt", []) if float(v) > 0.0]
    median_dt = _quantile(dts, 0.50, GENERATED_MOVE_INTERVAL)

    # Trapezoidal integral of normalized speed shape over normalized time.
    area = 0.0
    for i in range(1, len(points)):
        u0, q0 = points[i - 1]
        u1, q1 = points[i]
        if u1 > u0:
            area += 0.5 * (q0 + q1) * (u1 - u0)
    area = max(area, 1e-4)

    # Normal V46 behavior derives duration from source peak speed. V48 loiter
    # callers may instead own duration explicitly while retaining speed_shape.
    if forced_duration is None:
        effective_duration = max(0.020, distance / (peak_speed * area))
        profile_peak_speed = peak_speed
    else:
        effective_duration = max(0.020, float(forced_duration))
        profile_peak_speed = distance / max(effective_duration * area, 1e-9)

    def shape_at(u):
        if u <= points[0][0]:
            return points[0][1]
        for i in range(1, len(points)):
            u1, q1 = points[i]
            if u <= u1:
                u0, q0 = points[i - 1]
                if u1 <= u0:
                    return q1
                f = (u - u0) / (u1 - u0)
                return q0 + f * (q1 - q0)
        return points[-1][1]

    measured_pool = _load_measured_report_timing_pool() if SYNTHETIC_EMPIRICAL_TIMING_ENABLED else []
    timing_pool = [float(dt) for dt in measured_pool if 0.0 < float(dt) <= SYNTHETIC_EMPIRICAL_TIMING_MAX]

    # Numerically integrate the physical speed template at the V44 empirical
    # report times.  The integral is normalized at the end so the endpoint is exact.
    samples = [(0.0, 0.0)]
    elapsed = 0.0
    cumulative = 0.0
    prev_v = 0.0
    while elapsed < effective_duration - 1e-9:
        dt = _measured_timing_rng.choice(timing_pool) if timing_pool else median_dt
        remaining = effective_duration - elapsed
        terminal_floor = 0.004
        if remaining > dt and (remaining - dt) < terminal_floor:
            next_t = effective_duration
        else:
            next_t = min(effective_duration, elapsed + dt)
        actual_dt = next_t - elapsed
        u = max(0.0, min(1.0, next_t / effective_duration))
        v = profile_peak_speed * shape_at(u)
        cumulative += 0.5 * (prev_v + v) * actual_dt
        samples.append((next_t, cumulative))
        elapsed = next_t
        prev_v = v

    total = max(samples[-1][1], 1e-9)
    profile = [(t, max(0.0, min(1.0, d / total))) for t, d in samples]
    profile[-1] = (effective_duration, 1.0)

    diagnostics = {
        "source_distance": float(gesture.get("distance", 0.0)),
        "target_distance": distance,
        "source_peak_speed": peak_speed,
        "profile_peak_speed": profile_peak_speed,
        "forced_duration": None if forced_duration is None else float(forced_duration),
        "shape_area": area,
        "effective_duration": effective_duration,
        "timing_pool_size": len(timing_pool),
        "motion_model": "physical_speed_shape",
    }
    if return_diagnostics:
        return effective_duration, profile, diagnostics
    return effective_duration, profile



def _choose_library_motion_profile(distance, log_transition=False):
    """Select one physically recorded gesture as the synthetic dynamics template.

    V46 uses the selected gesture's stored physical speed-shape profile and
    robust peak speed.  The target distance determines the generated duration.
    """
    library = _load_gesture_library()
    if not library:
        return None

    target_distance = max(1e-9, float(distance))
    eligible = [
        g for g in library
        if SYNTHETIC_DISTANCE_RATIO_MIN
        <= (float(g["distance"]) / target_distance)
        <= SYNTHETIC_DISTANCE_RATIO_MAX
    ]
    if eligible:
        gesture = secrets.SystemRandom().choice(eligible)
        selection_mode = "distance_band"
    else:
        gesture = min(
            library,
            key=lambda g: abs(math.log(
                max(1e-9, float(g["distance"])) / target_distance
            )),
        )
        selection_mode = "nearest_fallback"

    built = _build_motion_profile_from_gesture(
        distance, gesture, return_diagnostics=log_transition
    )
    if built is None:
        return None

    if log_transition:
        duration, profile, diag = built
    else:
        duration, profile = built
        diag = None

    short_id = gesture["fingerprint"][:8]
    source = (
        f"gesture={short_id}, library={len(library)}, "
        f"source_dist={float(gesture['distance']):.1f}, "
        f"peak_speed={float(gesture['peak_speed']):.1f}, "
        f"accel_limit={float(gesture['accel_limit']):.1f}, "
        f"target_dist={float(distance):.1f}, generated_duration={duration:.3f}s"
    )

    if log_transition and diag is not None:
        _log_transition_diagnostic(
            "\n[TRANSITION V46.1]\n"
            f"gesture_id          : {short_id}\n"
            f"selection_mode      : {selection_mode}\n"
            f"source_distance     : {diag['source_distance']:.1f}\n"
            f"target_distance     : {diag['target_distance']:.1f}\n"
            f"source_peak_speed   : {diag['source_peak_speed']:.1f} counts/s\n"
            f"speed_shape_area    : {diag['shape_area']:.6f}\n"
            f"generated_duration  : {diag['effective_duration']:.6f} s\n"
            f"timing_pool_size    : {diag['timing_pool_size']}\n"
            f"motion_model        : {diag['motion_model']}\n"
            "----------------------------------------"
        )

    return duration, profile, source


def _choose_transition_motion_profile(transition_distance, max_duration=None):
    """Recording-start synthetic relocation from the persistent gesture library."""
    if MOTION_MODEL_ENABLED:
        modeled = _choose_library_motion_profile(transition_distance, log_transition=True)
        if modeled is not None:
            return modeled

    duration = max(TRANSITION_MIN_DURATION, TRANSITION_FALLBACK_DURATION)
    return duration, [(0.0, 0.0), (duration, 1.0)], "fallback-linear-profile"



def _choose_discrete_duration(min_duration, max_duration, step):
    """Choose a bounded duration from an inclusive discrete time grid."""
    lo = max(0.020, float(min_duration))
    hi = max(lo, float(max_duration))
    step = max(0.001, float(step))
    count = max(0, int(math.floor((hi - lo) / step + 1e-9)))
    index = secrets.SystemRandom().randint(0, count)
    return min(hi, lo + index * step)


def _choose_loiter_motion_profile(transition_distance, motion_type="loiter_excursion"):
    """V48: bounded-duration loiter using a real physical speed shape."""
    library = _load_gesture_library()

    if motion_type == "loiter_exit":
        duration = _choose_discrete_duration(
            LOITER_EXIT_DURATION_MIN, LOITER_EXIT_DURATION_MAX, LOITER_DURATION_STEP
        )
    elif motion_type == "button_anchor_return":
        duration = _choose_discrete_duration(
            LOITER_ANCHOR_RETURN_DURATION_MIN,
            LOITER_ANCHOR_RETURN_DURATION_MAX,
            LOITER_DURATION_STEP,
        )
    else:
        duration = _choose_discrete_duration(
            LOITER_EXCURSION_DURATION_MIN, LOITER_EXCURSION_DURATION_MAX, LOITER_DURATION_STEP
        )

    if not library:
        return duration, [(0.0, 0.0), (duration, 1.0)], "fallback-linear-profile"

    target_distance = max(1e-9, float(transition_distance))
    eligible = [
        g for g in library
        if SYNTHETIC_DISTANCE_RATIO_MIN
        <= (float(g["distance"]) / target_distance)
        <= SYNTHETIC_DISTANCE_RATIO_MAX
    ]
    if eligible:
        gesture = secrets.SystemRandom().choice(eligible)
        selection_mode = "loiter_distance_band"
    else:
        gesture = min(
            library,
            key=lambda g: abs(math.log(max(1e-9, float(g["distance"])) / target_distance)),
        )
        selection_mode = "loiter_nearest_distance"

    duration, profile, diag = _build_motion_profile_from_gesture(
        transition_distance,
        gesture,
        return_diagnostics=True,
        forced_duration=duration,
    )

    short_id = gesture["fingerprint"][:8]
    source = (
        f"loiter gesture={short_id}, mode={selection_mode}, type={motion_type}, "
        f"source_dist={float(gesture['distance']):.1f}, "
        f"target_dist={float(transition_distance):.1f}, duration={duration:.3f}s, "
        f"required_peak={diag['profile_peak_speed']:.1f}"
    )
    return duration, profile, source



def generated_transition_to(target_x, target_y):
    """Move to a recording start using generated geometry + real hand timing."""
    # Never calculate acceleration across two independent transitions.
    _reset_transition_progress_debug()

    with position_lock:
        start_x, start_y = x, y

    target_x, target_y = clamp_target_to_screen(target_x, target_y)
    distance = math.hypot(target_x - start_x, target_y - start_y)
    if distance <= 0.0:
        return True

    duration, speed_profile, source = _choose_transition_motion_profile(distance)
    pass  # V34: playback print suppressed

    # No button is held while moving between recordings.
    return generated_move_to(
        target_x, target_y, duration, speed_profile, 0,
        motion_type="recording_transition",
        reconstruct_profile_pauses=False,
    )



# V15 trajectory instrumentation. These fields describe the generated motion
# responsible for each native HID report; they do not alter playback behavior.
diagnostic_segment_counter = 0
diagnostic_context = {
    "segment_id": -1,
    "motion_type": "non_curve",
    "bezier_t": float("nan"),
    "profile_progress": float("nan"),
    "curve_progress": float("nan"),
}

def _set_diagnostic_context(segment_id=-1, motion_type="non_curve",
                            bezier_t=float("nan"),
                            profile_progress=float("nan"),
                            curve_progress=float("nan")):
    diagnostic_context["segment_id"] = segment_id
    diagnostic_context["motion_type"] = motion_type
    diagnostic_context["bezier_t"] = bezier_t
    diagnostic_context["profile_progress"] = profile_progress
    diagnostic_context["curve_progress"] = curve_progress


native_capture_file = None
native_capture_writer = None
native_capture_start_ns = None

def _next_native_capture_filename():
    os.makedirs(NATIVE_OUTPUT_CAPTURE_DIR, exist_ok=True)
    highest = 0
    for filename in glob.glob(os.path.join(NATIVE_OUTPUT_CAPTURE_DIR, "generated_native_*.csv")):
        match = re.search(r"generated_native_(\d+)\.csv$", os.path.basename(filename))
        if match:
            highest = max(highest, int(match.group(1)))
    return os.path.join(NATIVE_OUTPUT_CAPTURE_DIR, f"generated_native_{highest + 1:04d}.csv")

def start_native_output_capture():
    global native_capture_file, native_capture_writer, native_capture_start_ns
    if not NATIVE_OUTPUT_CAPTURE_ENABLED:
        return
    filename = _next_native_capture_filename()
    native_capture_file = open(filename, "w", newline="", buffering=1)
    native_capture_writer = csv.writer(native_capture_file)
    native_capture_writer.writerow([
        "time_ns","dx","dy","wheel","buttons","x","y",
        "segment_id","motion_type","bezier_t","profile_progress","curve_progress"
    ])
    native_capture_start_ns = time.monotonic_ns()
    print(f"Native generated-output capture started: {filename}")

def capture_native_report(dx_value, dy_value, wheel_value, button_state):
    if native_capture_writer is None:
        return
    now = time.monotonic_ns()
    with position_lock:
        sample_x, sample_y = x, y
    native_capture_writer.writerow([
        now-native_capture_start_ns, int(dx_value), int(dy_value),
        int(wheel_value), int(button_state), int(sample_x), int(sample_y),
        diagnostic_context["segment_id"],
        diagnostic_context["motion_type"],
        diagnostic_context["bezier_t"],
        diagnostic_context["profile_progress"],
        diagnostic_context["curve_progress"],
    ])

def stop_native_output_capture():
    global native_capture_file, native_capture_writer, native_capture_start_ns
    if native_capture_writer is None:
        return
    filename = native_capture_file.name
    native_capture_file.flush()
    native_capture_file.close()
    native_capture_file = None
    native_capture_writer = None
    native_capture_start_ns = None
    print(f"Native generated-output capture saved: {filename}")

def capture_emitted_report(dx_value, dy_value, wheel_value, button_state):
    """Record one generated HID report at native report cadence."""
    capture_native_report(dx_value, dy_value, wheel_value, button_state)


_measured_timing_cache_signature = None
_measured_timing_pool = []
_measured_timing_rng = secrets.SystemRandom()


def _quantile_sorted(values, q):
    """Simple linear quantile; avoids adding a numpy dependency to the bridge."""
    if not values:
        return 0.0
    q = max(0.0, min(1.0, float(q)))
    pos = q * (len(values) - 1)
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return values[lo]
    a = pos - lo
    return values[lo] * (1.0 - a) + values[hi] * a


def _recording_timing_signature():
    files = sorted(glob.glob(os.path.join(RECORDING_DIR, "recording_*.csv")))
    signature = []
    for filename in files:
        try:
            st = os.stat(filename)
            signature.append((filename, st.st_mtime_ns, st.st_size))
        except OSError:
            pass
    return tuple(signature)


def _load_measured_report_timing_pool():
    """Return physical inter-report intervals suitable for active generated motion.

    We use intervals ending at a report where either the current or preceding
    physical report contains XY motion. This keeps the native ~8 ms cadence
    around movement while excluding unrelated long stationary gaps.

    The central quantile range is retained, with an additional absolute bound.
    """
    global _measured_timing_cache_signature, _measured_timing_pool

    signature = _recording_timing_signature()
    if (
        signature == _measured_timing_cache_signature
        and _measured_timing_pool
    ):
        return _measured_timing_pool

    raw = []

    for filename, _, _ in signature:
        try:
            with open(filename, "r", newline="") as f:
                rows = list(csv.DictReader(f))
        except (OSError, csv.Error):
            continue

        previous = None
        for row in rows:
            try:
                t_ns = int(row["time_ns"])
                dx = int(row["dx"])
                dy = int(row["dy"])
            except (KeyError, TypeError, ValueError):
                previous = None
                continue

            if previous is not None:
                prev_t_ns, prev_dx, prev_dy = previous
                dt = (t_ns - prev_t_ns) / 1e9
                motion_adjacent = (
                    dx != 0 or dy != 0 or prev_dx != 0 or prev_dy != 0
                )
                if (
                    motion_adjacent
                    and MEASURED_TIMING_ABSOLUTE_MIN
                        <= dt
                        <= MEASURED_TIMING_ABSOLUTE_MAX
                ):
                    raw.append(dt)

            previous = (t_ns, dx, dy)

    if not raw:
        _measured_timing_pool = []
        _measured_timing_cache_signature = signature
        print(
            "Measured report timing: no usable physical intervals; "
            f"falling back to {GENERATED_MOVE_INTERVAL*1000:.3f} ms."
        )
        return _measured_timing_pool

    ordered = sorted(raw)
    low = _quantile_sorted(ordered, MEASURED_TIMING_LOW_QUANTILE)
    high = _quantile_sorted(ordered, MEASURED_TIMING_HIGH_QUANTILE)
    pool = [dt for dt in raw if low <= dt <= high]

    if not pool:
        pool = raw

    _measured_timing_pool = pool
    _measured_timing_cache_signature = signature

    ordered_pool = sorted(pool)
    median = _quantile_sorted(ordered_pool, 0.5)
    p05 = _quantile_sorted(ordered_pool, 0.05)
    p95 = _quantile_sorted(ordered_pool, 0.95)
    mean = sum(pool) / len(pool)

    pause_limit = _quantile_sorted(
        ordered_pool, PAUSE_TIMING_QUANTILE
    ) if "PAUSE_TIMING_QUANTILE" in globals() else high
    print(
        "Measured report timing pool: "
        f"N={len(pool)}, mean={mean*1000:.3f} ms, "
        f"median={median*1000:.3f} ms, "
        f"p05={p05*1000:.3f} ms, p95={p95*1000:.3f} ms, "
        f"active-gap-limit={pause_limit*1000:.3f} ms"
    )
    return _measured_timing_pool


class _MeasuredTimingBlockSampler:
    """Sample contiguous blocks from the physical dt pool.

    The pool is stored in original recording order, so contiguous blocks retain
    short-term timing relationships better than independent random draws.
    """

    def __init__(self, pool):
        self.pool = pool
        self.block = []
        self.index = 0

    def _new_block(self):
        if not self.pool:
            self.block = []
            self.index = 0
            return

        block_min = max(1, int(MEASURED_TIMING_BLOCK_MIN))
        block_max = max(block_min, int(MEASURED_TIMING_BLOCK_MAX))
        length = _measured_timing_rng.randint(block_min, block_max)
        length = min(length, len(self.pool))

        max_start = max(0, len(self.pool) - length)
        start = (
            _measured_timing_rng.randint(0, max_start)
            if max_start > 0 else 0
        )
        self.block = self.pool[start:start + length]
        self.index = 0

    def next_dt(self):
        if not self.pool:
            return GENERATED_MOVE_INTERVAL
        if self.index >= len(self.block):
            self._new_block()
        dt = self.block[self.index]
        self.index += 1
        return dt


def _build_generated_report_schedule(duration):
    """Build elapsed report timestamps ending exactly at duration.

    Measured physical intervals determine the native sample times. The final
    interval is shortened only as needed to land on the segment endpoint, so
    timing variation does not change the requested movement duration.
    """
    duration = max(0.0, float(duration))
    if duration <= 0.0:
        return [0.0]

    if not MEASURED_REPORT_TIMING_ENABLED:
        steps = max(1, int(round(duration / GENERATED_MOVE_INTERVAL)))
        return [duration * i / steps for i in range(1, steps + 1)]

    pool = _load_measured_report_timing_pool()
    if not pool:
        steps = max(1, int(round(duration / GENERATED_MOVE_INTERVAL)))
        return [duration * i / steps for i in range(1, steps + 1)]

    sampler = _MeasuredTimingBlockSampler(pool)
    elapsed = 0.0
    schedule = []

    while True:
        dt = max(1e-6, float(sampler.next_dt()))
        next_elapsed = elapsed + dt

        if next_elapsed >= duration:
            schedule.append(duration)
            break

        schedule.append(next_elapsed)
        elapsed = next_elapsed

    return schedule




def _measured_active_report_gap_limit():
    """Return the data-derived upper edge of ordinary active report timing.

    This is not a pause-duration estimate. The recording already gives the
    exact elapsed gap. This value only answers whether that gap still looks
    like one ordinary native report interval.
    """
    pool = _load_measured_report_timing_pool()
    if not pool:
        # Fallback only when no physical timing data is available.
        return GENERATED_MOVE_INTERVAL * 2.0

    ordered = sorted(float(dt) for dt in pool if dt > 0.0)
    if not ordered:
        return GENERATED_MOVE_INTERVAL * 2.0

    return _quantile_sorted(ordered, PAUSE_TIMING_QUANTILE)


def _borrow_pause_report_intervals(pause_duration):
    """Fill a source-motion pause with plausible zero-report intervals."""
    pause_duration = max(0.0, float(pause_duration))
    if pause_duration <= 0.0:
        return []

    pool = _load_measured_report_timing_pool()
    sampler = _MeasuredTimingBlockSampler(pool) if pool else None

    intervals = []
    elapsed = 0.0
    while elapsed < pause_duration - 1e-9:
        remaining = pause_duration - elapsed
        dt = (
            max(1e-6, float(sampler.next_dt()))
            if sampler is not None
            else GENERATED_MOVE_INTERVAL
        )

        # Land exactly on the end of the source pause.
        dt = min(dt, remaining)
        intervals.append(dt)
        elapsed += dt

    return intervals


def _build_paired_profile_report_schedule(effective_profile, duration):
    """Return (elapsed_time, force_zero) report entries.

    Normal moving reports retain the selected source profile's own timestamps.

    The exact elapsed gap between source points comes directly from the
    recording. To decide whether it represents one ordinary native report or
    contains unobserved stationary report time, compare it with the measured
    upper edge of active report timing. Longer gaps are filled internally with
    zero-motion reports, but their total duration is left exactly unchanged.
    """
    duration = max(0.0, float(duration))
    if duration <= 0.0:
        return [(0.0, False)]

    points = []
    for t, p in effective_profile:
        t = max(0.0, min(duration, float(t)))
        p = max(0.0, min(1.0, float(p)))
        if t <= 1e-9:
            continue
        if points and t <= points[-1][0] + 1e-9:
            points[-1] = (points[-1][0], p)
        else:
            points.append((t, p))

    schedule = []
    previous_t = 0.0
    normal_gap_limit = _measured_active_report_gap_limit()

    for t, _ in points:
        gap = t - previous_t

        if PAUSE_ZERO_REPORTS_ENABLED and gap > normal_gap_limit:
            borrowed = _borrow_pause_report_intervals(gap)
            cursor_t = previous_t

            # All borrowed reports except the last one are stationary fill.
            # The last boundary is represented by the real profile report at t.
            for dt in borrowed[:-1]:
                cursor_t += dt
                if cursor_t < t - 1e-9:
                    schedule.append((cursor_t, True))

        schedule.append((t, False))
        previous_t = t

    if not schedule or schedule[-1][0] < duration - 1e-9:
        schedule.append((duration, False))
    else:
        schedule[-1] = (duration, schedule[-1][1])

    return schedule




_transition_debug_previous = None
_transition_debug_id = 0


def _log_transition_progress_debug(progress, dx, dy, elapsed, force_zero):
    """Append native generated transition samples for offline diagnostics."""
    global _transition_debug_previous
    global _transition_debug_id

    if not TRANSITION_PROGRESS_DEBUG_ENABLED:
        return

    now_ns = time.monotonic_ns()
    dt_s = 0.0
    velocity = 0.0
    acceleration = 0.0

    if _transition_debug_previous is not None:
        prev_ns, prev_velocity = _transition_debug_previous
        dt_s = max(0.0, (now_ns - prev_ns) / 1e9)
        if dt_s > 0.0:
            velocity = math.hypot(dx, dy) / dt_s
            acceleration = (velocity - prev_velocity) / dt_s

    _transition_debug_previous = (now_ns, velocity)

    os.makedirs(os.path.dirname(TRANSITION_PROGRESS_DEBUG_FILE), exist_ok=True)
    new_file = not os.path.exists(TRANSITION_PROGRESS_DEBUG_FILE)
    with open(TRANSITION_PROGRESS_DEBUG_FILE, "a", newline="") as f:
        writer = csv.writer(f)
        if new_file:
            writer.writerow([
                "transition_id", "time_ns", "elapsed_s", "progress",
                "dx", "dy", "dt_s", "velocity",
                "signed_acceleration", "acceleration_magnitude",
                "pause_fill"
            ])
        writer.writerow([
            _transition_debug_id, now_ns, elapsed, progress, dx, dy, dt_s, velocity,
            acceleration, abs(acceleration), int(bool(force_zero))
        ])


def _reset_transition_progress_debug():
    global _transition_debug_previous
    global _transition_debug_id
    _transition_debug_previous = None
    _transition_debug_id += 1


def generated_move_to(target_x, target_y, duration, speed_profile, held_buttons,
                      arc_length_timing=True, motion_type="normal",
                      reconstruct_profile_pauses=True):
    """Generate fresh geometry while preserving recorded hand-speed profile."""
    global x, y, diagnostic_segment_counter
    diagnostic_segment_counter += 1
    diagnostic_segment_id = diagnostic_segment_counter

    with position_lock:
        start_x, start_y = x, y

    target_x, target_y = clamp_target_to_screen(target_x, target_y)

    if start_x == target_x and start_y == target_y:
        # Keep the recorded hold state even during a stationary pause.
        _set_diagnostic_context(
            diagnostic_segment_id, motion_type, 1.0, 1.0, 1.0
        )
        hid.write(bytes([held_buttons, 0, 0, 0]))
        capture_emitted_report(0, 0, 0, held_buttons)
        return not playback_stop_event.wait(max(0.0, duration))

    c1, c2 = _random_cubic_controls(start_x, start_y, target_x, target_y)
    arc_table = _build_bezier_arc_table(
        start_x, start_y, c1, c2, target_x, target_y
    )

    distance = math.hypot(target_x - start_x, target_y - start_y)

    # V9 arc-length timing compensation:
    # _build_bezier_arc_table() stores cumulative distance in column 1.
    # If randomized geometry makes the actual path longer than the direct
    # endpoint displacement, give it proportionally more time. This preserves
    # the source movement's approximate speed scale instead of forcing a long
    # arc through the original short duration.
    effective_duration = max(0.0, duration)
    arc_length = arc_table[-1][1] if arc_table else distance
    arc_scale = 1.0

    if GENERATED_ARC_LENGTH_TIMING and arc_length_timing and distance > 1e-9:
        arc_scale = arc_length / distance
        arc_scale = max(
            1.0,
            min(GENERATED_ARC_LENGTH_TIMING_MAX_SCALE, arc_scale),
        )
        effective_duration *= arc_scale

    # The source profile expresses cumulative distance progress versus time.
    # Preserve its normalized shape but stretch it over effective_duration.
    effective_profile = _scale_normalized_profile(
        _normalize_profile_time(speed_profile),
        effective_duration,
    )

    if arc_scale > 1.01:
        if PLAYBACK_VERBOSE_DEBUG:
            pass  # V34: playback print suppressed

    # V18: report timestamps come from the SAME physical source profile that
    # supplies cumulative-distance progress. This preserves the coupling
    # between each native dt and the motion that originally occurred over it.
    #
    # V17 instead selected an independent contiguous timing block from the
    # global recording pool. That could combine a steep profile increment with
    # an unrelated short dt and create artificial velocity/acceleration spikes.
    if PAIRED_PROFILE_REPORT_TIMING and reconstruct_profile_pauses:
        report_schedule = _build_paired_profile_report_schedule(
            effective_profile, effective_duration
        )
    elif PAIRED_PROFILE_REPORT_TIMING:
        # V39: synthetic active-motion profiles already contain the report
        # timestamps we want. Do not reinterpret stretched gaps as recorded
        # stationary pauses and insert force_zero reports.
        report_schedule = []
        for t, _ in effective_profile:
            t = max(0.0, min(effective_duration, float(t)))
            if t <= 1e-9:
                continue
            if report_schedule and t <= report_schedule[-1][0] + 1e-9:
                report_schedule[-1] = (report_schedule[-1][0], False)
            else:
                report_schedule.append((t, False))
        if not report_schedule or report_schedule[-1][0] < effective_duration - 1e-9:
            report_schedule.append((effective_duration, False))
        else:
            report_schedule[-1] = (effective_duration, False)
    else:
        report_schedule = [
            (t, False) for t in _build_generated_report_schedule(effective_duration)
        ]

    steps = len(report_schedule)

    with position_lock:
        previous_x, previous_y = x, y

    segment_start = time.monotonic()

    for index, (elapsed, force_zero) in enumerate(report_schedule, start=1):
        if playback_stop_event.is_set():
            return False

        progress = _interpolate_profile(effective_profile, elapsed)
        if index == steps and not force_zero:
            progress = 1.0

        bezier_t = _progress_to_bezier_t(arc_table, progress)
        bx, by = _bezier_point(
            bezier_t, start_x, start_y, c1, c2, target_x, target_y
        )
        point_x, point_y = round(bx), round(by)

        # During a reconstructed source pause, maintain report cadence but keep
        # the cursor fixed. Profile/curve progress resumes on the next real
        # source-profile report.
        if force_zero:
            point_x, point_y = previous_x, previous_y

        step_dx = point_x - previous_x
        step_dy = point_y - previous_y

        if motion_type == "recording_transition":
            _log_transition_progress_debug(
                progress, step_dx, step_dy, elapsed, force_zero
            )
        step_dx, step_dy = limit_motion(
            previous_x, previous_y, step_dx, step_dy
        )
        hid_dx = clamp(step_dx)
        hid_dy = clamp(step_dy)

        # Record where this report came from on the generated curve.
        # profile_progress is normalized source distance progress.
        # bezier_t is geometric parameter position. curve_progress is the
        # scheduled step fraction through this generated segment.
        _set_diagnostic_context(
            diagnostic_segment_id,
            motion_type,
            bezier_t,
            progress,
            index / steps,
        )

        # Crucial for dragging: every motion report carries the button state
        # that was held during this interval in the original recording.
        hid.write(bytes([
            held_buttons,
            signed_byte(hid_dx),
            signed_byte(hid_dy),
            0
        ]))

        with position_lock:
            x += hid_dx
            y += hid_dy
            previous_x, previous_y = x, y

        capture_emitted_report(hid_dx, hid_dy, 0, held_buttons)

        deadline = segment_start + elapsed
        remaining = deadline - time.monotonic()
        if remaining > 0 and playback_stop_event.wait(remaining):
            return False

    return True



def _is_loiter_segment(segment):
    """Long stationary source interval with no button held."""
    return (
        LOITER_ENABLED
        and int(segment.get("held_buttons", 0)) == 0
        and (
            bool(segment.get("force_loiter", False))
            or (
                float(segment.get("motion_distance", 0.0))
                    <= LOITER_MOTION_THRESHOLD
                and float(segment.get("duration", 0.0)) > LOITER_TIMEOUT
            )
        )
    )


def _choose_loiter_target(origin_x, origin_y):
    """Choose a valid temporary target without collapsing it onto a boundary.

    V9 generated a point in a large circle and then clamped it. With a very
    large LOITER_MAX_RADIUS, many samples were outside the calibrated rectangle;
    clamping could collapse them onto an edge or even back onto the current
    point. V10 rejects invalid samples and tries again instead.
    """
    rng = secrets.SystemRandom()
    rmin = max(0.0, float(LOITER_MIN_RADIUS))
    rmax = max(rmin, float(LOITER_MAX_RADIUS))

    if screen_calibrated:
        for _ in range(max(1, int(LOITER_TARGET_MAX_ATTEMPTS))):
            radius = math.sqrt(rng.uniform(rmin * rmin, rmax * rmax))
            angle = rng.uniform(0.0, 2.0 * math.pi)
            tx = round(origin_x + radius * math.cos(angle))
            ty = round(origin_y + radius * math.sin(angle))

            if (
                SCREEN_MIN_X <= tx <= SCREEN_MAX_X
                and SCREEN_MIN_Y <= ty <= SCREEN_MAX_Y
                and math.hypot(tx - origin_x, ty - origin_y) >= rmin
            ):
                return tx, ty

        # If max radius is enormous relative to the screen, rejection sampling
        # can be inefficient. Fall back to choosing directly inside the screen,
        # still respecting the minimum radius when possible.
        for _ in range(max(1, int(LOITER_TARGET_MAX_ATTEMPTS))):
            tx = rng.randint(int(SCREEN_MIN_X), int(SCREEN_MAX_X))
            ty = rng.randint(int(SCREEN_MIN_Y), int(SCREEN_MAX_Y))
            d = math.hypot(tx - origin_x, ty - origin_y)
            if rmin <= d <= rmax:
                return tx, ty

        # Degenerate calibration/radius combination: stay valid and visible.
        return clamp_target_to_screen(origin_x, origin_y)

    radius = math.sqrt(rng.uniform(rmin * rmin, rmax * rmax))
    angle = rng.uniform(0.0, 2.0 * math.pi)
    return (
        round(origin_x + radius * math.cos(angle)),
        round(origin_y + radius * math.sin(angle)),
    )


def play_loiter_segment(segment):
    """Insert one excursion inside the ORIGINAL stationary time window.

    Timeline for a 5 s source loiter with LOITER_TIMEOUT=1 s:

        0.0 ----- 1.0 ---- excursion ---- random point ----- 5.0
         A         A                         R

    At 5.0 s playback continues normally FROM R.  We do not return to A and
    the excursion does not push the original loiter end time later.
    """
    total_duration = max(0.0, float(segment["duration"]))

    # 1) Preserve the beginning of the original stationary interval.
    timeout = min(max(0.0, float(LOITER_TIMEOUT)), total_duration)
    loiter_start = time.monotonic()

    hid.write(bytes([0, 0, 0, 0]))
    capture_emitted_report(0, 0, 0, 0)

    if timeout > 0.0 and playback_stop_event.wait(timeout):
        return False

    # 2) At the timeout, choose a temporary point and borrow a real recorded
    #    acceleration/deceleration profile for the excursion.
    with position_lock:
        origin_x, origin_y = x, y

    loiter_x, loiter_y = _choose_loiter_target(origin_x, origin_y)
    distance = math.hypot(loiter_x - origin_x, loiter_y - origin_y)

    excursion_duration, excursion_profile, source = (
        _choose_loiter_motion_profile(distance, motion_type="loiter_excursion")
        if LOITER_PRESERVE_SPEED_OVER_SCHEDULE
        else _choose_transition_motion_profile(
            distance,
            max_duration=TRANSITION_MAX_DURATION,
        )
    )

    # V11 deliberately does NOT squeeze a long excursion into whatever time
    # remains in the source loiter. Doing so preserves the clock at the expense
    # of velocity and was the cause of very fast relocations. Endpoint/button
    # correctness is the higher-priority invariant.
    elapsed = time.monotonic() - loiter_start
    remaining = max(0.0, total_duration - elapsed)
    if remaining <= 0.0 and not LOITER_PRESERVE_SPEED_OVER_SCHEDULE:
        return True

    if PLAYBACK_VERBOSE_DEBUG:
        pass  # V34: playback print suppressed

    if distance > 0.0:
        if not generated_move_to(
            loiter_x,
            loiter_y,
            excursion_duration,
            excursion_profile,
            0,
            arc_length_timing=True,
            motion_type="loiter_excursion",
            reconstruct_profile_pauses=False,
        ):
            return False

    # 3) If the excursion finished before the original stationary interval
    #    expires, wait out the remainder at the random point. If the excursion
    #    itself ran longer, continue immediately rather than speeding it up.
    elapsed = time.monotonic() - loiter_start
    remaining = max(0.0, total_duration - elapsed)

    if remaining > 0.0:
        hid.write(bytes([0, 0, 0, 0]))
        capture_emitted_report(0, 0, 0, 0)
        if playback_stop_event.wait(remaining):
            return False

    # Deliberately leave authoritative x/y at the random point. The next normal
    # generated segment starts there and still ends at its exact recorded target.
    return True



def play_recording_generated(filename, offset_x=0, offset_y=0):
    """Generated playback with natural speed profile and drag preservation."""
    start_x, start_y, segments = load_generated_segments(
        filename, offset_x, offset_y
    )

    print(
        f"Generated playback: {os.path.basename(filename)} "
        f"({len(segments)} motion segments)"
    )

    current_buttons = 0

    # V52: Read button-event timestamps DIRECTLY from the unmodified CSV.
    # Generated segmentation, loiter splitting and timing variation are not
    # allowed to change the minimum time between actual button transitions.
    source_button_events = []
    previous_source_buttons = 0
    with open(filename, "r", newline="") as source_file:
        for source_row in csv.DictReader(source_file):
            source_buttons = int(source_row["buttons"])
            if source_buttons != previous_source_buttons:
                source_button_events.append(
                    (int(source_row["time_ns"]) / 1e9, source_buttons)
                )
                previous_source_buttons = source_buttons

    playback_timeline_start = time.monotonic()
    source_timeline_elapsed = 0.0
    next_source_button_index = 0
    last_source_button_event_time = None
    last_emitted_button_event_time = None
    last_emitted_button_state = 0

    def wait_for_source_timeline_floor():
        # Preserve the existing source-segment schedule floor for movement.
        if not PLAYBACK_ENFORCE_SOURCE_TIMELINE_FLOOR:
            return True
        deadline = playback_timeline_start + source_timeline_elapsed
        remaining = deadline - time.monotonic()
        if remaining > 0.0 and playback_stop_event.wait(remaining):
            return False
        return not playback_stop_event.is_set()

    def emit_recorded_button_transition(new_buttons):
        """Wait BEFORE output, using exact original CSV button timestamps.

        For events i-1 and i, require:
          emitted[i] - emitted[i-1] >= source[i] - source[i-1]
        Also keep each event no earlier than its original absolute time.
        """
        nonlocal next_source_button_index
        nonlocal last_source_button_event_time
        nonlocal last_emitted_button_event_time, last_emitted_button_state

        if new_buttons == last_emitted_button_state:
            return True  # A segment boundary without a button transition.

        if next_source_button_index >= len(source_button_events):
            raise RuntimeError(
                "Generated playback produced a button transition absent "
                "from the source CSV"
            )
        source_event_time, expected_buttons = source_button_events[
            next_source_button_index
        ]
        if new_buttons != expected_buttons:
            raise RuntimeError(
                "Generated button transition order differs from source CSV: "
                f"expected 0x{expected_buttons:02X}, "
                f"got 0x{new_buttons:02X} at source event "
                f"{next_source_button_index + 1}"
            )

        deadline = playback_timeline_start + source_event_time
        if (last_source_button_event_time is not None
                and last_emitted_button_event_time is not None):
            original_gap = max(
                0.0, source_event_time - last_source_button_event_time
            )
            deadline = max(
                deadline, last_emitted_button_event_time + original_gap
            )

        remaining = deadline - time.monotonic()
        if remaining > 0.0 and playback_stop_event.wait(remaining):
            return False
        if playback_stop_event.is_set():
            return False

        _set_diagnostic_context(motion_type="button_transition")
        hid.write(bytes([new_buttons, 0, 0, 0]))
        capture_emitted_report(0, 0, 0, new_buttons)
        last_emitted_button_event_time = time.monotonic()
        last_source_button_event_time = source_event_time
        last_emitted_button_state = new_buttons
        next_source_button_index += 1
        return True

    for segment in segments:
        source_timeline_elapsed += max(
            0.0, float(segment.get("duration", 0.0))
        )
        if playback_stop_event.is_set():
            return False

        current_buttons = segment["held_buttons"]

        if _is_loiter_segment(segment):
            if not play_loiter_segment(segment):
                return False

            # A button transition is a spatial anchor. If this stationary
            # interval ends with a press/release, the cursor MUST be at the
            # exact recorded endpoint before that transition is emitted.
            #
            # If there is no button transition here, leave the cursor at the
            # random loiter point; the next ordinary motion segment will travel
            # directly from there to its own exact endpoint.
            if segment["end_buttons"] != segment["held_buttons"]:
                with position_lock:
                    anchor_start_x, anchor_start_y = x, y

                anchor_distance = math.hypot(
                    segment["x"] - anchor_start_x,
                    segment["y"] - anchor_start_y,
                )

                if anchor_distance > 0.0:
                    anchor_duration, anchor_profile, anchor_source = (
                        _choose_loiter_motion_profile(
                            anchor_distance,
                            motion_type="button_anchor_return",
                        )
                    )
                    print(
                        f"Button anchor return: "
                        f"({anchor_start_x}, {anchor_start_y}) -> "
                        f"({segment['x']}, {segment['y']}), "
                        f"{anchor_duration:.3f}s, profile={anchor_source}"
                    )
                    if not generated_move_to(
                        segment["x"],
                        segment["y"],
                        anchor_duration,
                        anchor_profile,
                        segment["held_buttons"],
                        arc_length_timing=True,
                        motion_type="button_anchor_return",
                        reconstruct_profile_pauses=False,
                    ):
                        return False

            current_buttons = segment["end_buttons"]
            if not emit_recorded_button_transition(current_buttons):
                return False

            if not wait_for_source_timeline_floor():
                return False
            continue

        if _is_button_dwell_segment(segment):
            # Ordinary click/short hold: vary the stationary dwell only.
            # Do NOT also apply generated movement timing variation here.
            varied_duration = _vary_button_dwell(segment["duration"])
            dwell_change_pct = (
                ((varied_duration / segment["duration"]) - 1.0) * 100.0
                if segment["duration"] > 0.0 else 0.0
            )
            print(
                f"Button dwell: {segment['duration'] * 1000.0:.1f} ms -> "
                f"{varied_duration * 1000.0:.1f} ms "
                f"({dwell_change_pct:+.1f}%) buttons=0x{current_buttons:02X}"
            )
            varied_profile = _scale_normalized_profile(
                _normalize_profile_time(segment["speed_profile"]),
                varied_duration,
            )
        else:
            # V16: vary only the ACTIVE movement interval. Stationary time at
            # the edges remains dwell and is not emitted as repeated (0,0)
            # movement reports.
            active_duration = float(
                segment.get("active_motion_duration", segment["duration"])
            )
            varied_duration, varied_profile = _vary_duration_and_profile(
                active_duration, segment["speed_profile"]
            )

            leading_dwell = float(segment.get("leading_motion_dwell", 0.0))
            if leading_dwell > 0.0:
                if playback_stop_event.wait(leading_dwell):
                    return False

        # V13: after a randomized loiter, the source segment's old
        # duration is no longer valid because its start point changed. Derive a
        # new duration/profile from the ACTUAL distance to the exact endpoint.
        if segment.get("after_loiter", False):
            with position_lock:
                exit_start_x, exit_start_y = x, y
            exit_distance = math.hypot(
                segment["x"] - exit_start_x,
                segment["y"] - exit_start_y,
            )
            if exit_distance > 0.0:
                segment_duration, segment_profile, exit_source = (
                    _choose_loiter_motion_profile(exit_distance, motion_type="loiter_exit")
                )
                varied_duration = segment_duration
                varied_profile = segment_profile
                if PLAYBACK_VERBOSE_DEBUG:
                    print(
                        f"Loiter exit: ({exit_start_x}, {exit_start_y}) -> "
                        f"({segment['x']}, {segment['y']}), "
                        f"{segment_duration:.3f}s, profile={exit_source}"
                    )

        segment_motion_type = (
            "loiter_exit" if segment.get("after_loiter", False) else "normal"
        )
        if not generated_move_to(
            segment["x"],
            segment["y"],
            varied_duration,
            varied_profile,
            current_buttons,
            motion_type=segment_motion_type,
            reconstruct_profile_pauses=(segment_motion_type == "normal"),
        ):
            return False

        # V16: preserve stationary tail timing separately from the movement
        # profile. Waiting here produces no fake native movement reports.
        trailing_dwell = float(segment.get("trailing_motion_dwell", 0.0))
        if trailing_dwell > 0.0 and not segment.get("after_loiter", False):
            if playback_stop_event.wait(trailing_dwell):
                return False

        # Apply the recorded button transition at the exact segment endpoint.
        current_buttons = segment["end_buttons"]
        if not emit_recorded_button_transition(current_buttons):
            return False

        if not wait_for_source_timeline_floor():
            return False

    if next_source_button_index != len(source_button_events):
        raise RuntimeError(
            "Generated playback ended without emitting all recorded button "
            f"transitions: {next_source_button_index}/"
            f"{len(source_button_events)}"
        )

    if current_buttons:
        hid.write(bytes([0, 0, 0, 0]))

    return True


def play_recording(filename):
    """Replay one recording using its original reports and timing.

    dx/dy/wheel/buttons are the values reproduced. x/y from the CSV are not
    used to drive the mouse; they are the recorded reference trajectory.

    The live x/y tracker advances from the movement actually emitted.

    Returns:
        True if the recording completed.
        False if playback was interrupted.
    """
    global x, y

    print(f"Playing: {os.path.basename(filename)}")

    with open(filename, "r", newline="") as file:
        reader = csv.DictReader(file)
        previous_time_ns = 0

        for row in reader:
            if playback_stop_event.is_set():
                return False

            timestamp_ns = int(row["time_ns"])
            delay_ns = timestamp_ns - previous_time_ns

            if delay_ns > 0:
                if playback_stop_event.wait(delay_ns / 1e9):
                    return False

            previous_time_ns = timestamp_ns

            dx_play = clamp(int(row["dx"]))
            dy_play = clamp(int(row["dy"]))
            wheel_play = clamp(int(row["wheel"]))
            buttons_play = int(row["buttons"])

            hid.write(bytes([
                buttons_play,
                signed_byte(dx_play),
                signed_byte(dy_play),
                signed_byte(wheel_play)
            ]))

            with position_lock:
                x += dx_play
                y += dy_play

    return True


def playback_loop():
    """Continuously play all compatible recordings in sequential order.

    Before each recording:
      1. Recover its absolute starting x/y from the first CSV row.
      2. Generate a fresh cubic transition using a randomly borrowed real
         hand-speed profile from the available recordings.
      3. Replay the recording.

    After the last recording, playback starts again at the first recording.
    """
    global playback

    try:
        while not playback_stop_event.is_set():
            files = sorted(
                glob.glob(
                    os.path.join(
                        RECORDING_DIR,
                        "recording_*.csv"
                    )
                )
            )

            if not files:
                pass  # V34: playback print suppressed
                break

            pass  # V34: playback print suppressed
            pass  # V34: playback print suppressed
            for filename in files:
                pass  # V34: playback print suppressed
            pass  # V34: playback print suppressed

            for filename in files:
                if playback_stop_event.is_set():
                    break

                try:
                    stored_start_x, stored_start_y = get_recording_start(filename)
                except ValueError as error:
                    pass  # V34: playback print suppressed
                    pass  # V34: playback print suppressed
                    continue

                # Pick a new point inside the configured circular region for
                # every recording on every playback cycle.
                offset_x, offset_y = choose_random_circular_offset()

                target_x = stored_start_x + offset_x
                target_y = stored_start_y + offset_y
                target_x, target_y = clamp_target_to_screen(
                    target_x, target_y
                )

                pass  # V34: playback print suppressed

                if not generated_transition_to(target_x, target_y):
                    break

                if PLAYBACK_MODE == "recorded":
                    if not play_recording(filename):
                        break
                elif PLAYBACK_MODE == "generated":
                    if not play_recording_generated(
                        filename, offset_x, offset_y
                    ):
                        break
                else:
                    raise ValueError(
                        f"Unknown PLAYBACK_MODE: {PLAYBACK_MODE!r}. "
                        'Use "recorded" or "generated".'
                    )

    finally:
        # Ensure Windows is not left with a held mouse button.
        try:
            hid.write(bytes([0, 0, 0, 0]))
        except Exception:
            pass

        stop_native_output_capture()
        playback = False
        pass  # V34: playback print suppressed


def start_playback():
    """Start sequential playback in a background thread."""
    global playback
    global playback_thread

    if playback:
        return

    if recording:
        pass  # V34: playback print suppressed
        return

    playback = True
    playback_stop_event.clear()
    start_native_output_capture()

    playback_thread = threading.Thread(
        target=playback_loop,
        daemon=True
    )
    playback_thread.start()

    pass  # V34: playback print suppressed


def stop_playback():
    """Request playback to stop and interrupt any active playback wait."""
    if not playback:
        return

    pass  # V34: playback print suppressed
    playback_stop_event.set()


def clear_mouse_recording_data():
    """Stop active work, delete RECORDING_DIR, recreate it, and reset numbering."""
    global recording_number

    print(f"\n*** EXTRA HELD {PLAYBACK_DELETE_HOLD_SECONDS:.1f}s: CLEARING RECORDING DATA ***")

    # Stop producers/consumers before removing files beneath them.
    if recording:
        stop_recording()

    if playback:
        stop_playback()
        thread = playback_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=5.0)

    # In case playback ended between the checks above, make sure the native
    # capture file is closed before deleting its directory.
    stop_native_output_capture()

    try:
        shutil.rmtree(RECORDING_DIR)
    except FileNotFoundError:
        pass

    os.makedirs(RECORDING_DIR, exist_ok=True)
    recording_number = 1

    if DELETE_GESTURE_LIBRARY_ON_RECORDING_RESET:
        try:
            os.remove(GESTURE_LIBRARY_FILE)
            print(f"Gesture library deleted: {GESTURE_LIBRARY_FILE}")
        except FileNotFoundError:
            pass

    print(f"Recording data cleared: {RECORDING_DIR}")
    print("Next physical recording will be recording_001.csv\n")


def _playback_button_hold_worker(generation):
    """Trigger the destructive reset once EXTRA has remained down for 3 seconds."""
    global playback_button_delete_triggered

    time.sleep(PLAYBACK_DELETE_HOLD_SECONDS)

    with playback_button_lock:
        if (
            not playback_button_held
            or generation != playback_button_generation
            or playback_button_delete_triggered
        ):
            return
        playback_button_delete_triggered = True

    clear_mouse_recording_data()


def handle_playback_button(event_value):
    """Short EXTRA press/release toggles playback; a 3 s hold clears recordings."""
    global playback_button_held, playback_button_press_ns
    global playback_button_delete_triggered, playback_button_generation

    if event_value == 1:  # press
        with playback_button_lock:
            if playback_button_held:
                return
            playback_button_held = True
            playback_button_press_ns = time.monotonic_ns()
            playback_button_delete_triggered = False
            playback_button_generation += 1
            generation = playback_button_generation

        threading.Thread(
            target=_playback_button_hold_worker,
            args=(generation,),
            daemon=True,
        ).start()
        return

    if event_value != 0:  # ignore autorepeat/other values
        return

    should_delete = False
    with playback_button_lock:
        if not playback_button_held:
            return

        held_seconds = (time.monotonic_ns() - playback_button_press_ns) / 1e9
        playback_button_held = False

        # Cover the small race where release occurs just after the threshold but
        # before the hold worker gets scheduled.
        if held_seconds >= PLAYBACK_DELETE_HOLD_SECONDS:
            if not playback_button_delete_triggered:
                playback_button_delete_triggered = True
                should_delete = True
            long_press = True
        else:
            long_press = playback_button_delete_triggered

    if should_delete:
        clear_mouse_recording_data()
        return

    if long_press:
        return

    if playback:
        stop_playback()
    else:
        start_playback()


# ============================================================
# Main Event Loop
# ============================================================

pass  # V34: playback print suppressed
pass  # V34: playback print suppressed
pass  # V34: playback print suppressed
pass  # V34: playback print suppressed
pass  # V34: playback print suppressed
pass  # V34: playback print suppressed
pass  # V34: playback print suppressed
pass  # V34: playback print suppressed
pass  # V34: playback print suppressed
pass  # V34: playback print suppressed
pass  # V34: playback print suppressed


try:
    for event in mouse.read_loop():

        # --------------------------------------------------
        # Relative movement
        # --------------------------------------------------
        if event.type == ecodes.EV_REL:
            # During playback, physical mouse movement/scrolling is consumed
            # completely. EXTRA remains available in EV_KEY to stop playback.
            if playback:
                continue

            if event.code == ecodes.REL_X:
                dx += event.value
            elif event.code == ecodes.REL_Y:
                dy += event.value
            elif event.code == ecodes.REL_WHEEL:
                # Scrolling remains completely normal.
                wheel += event.value

        # --------------------------------------------------
        # Buttons
        # --------------------------------------------------
        elif event.type == ecodes.EV_KEY:

            # During playback, consume every physical button except EXTRA.
            # EXTRA is handled on both press and release so a short hold can
            # toggle playback while a 3-second hold can clear recording data.
            if playback and event.code != ecodes.BTN_EXTRA:
                continue

            # Consume release of the final reference click as well.
            if (consume_reference_middle_release
                    and event.code == ecodes.BTN_MIDDLE
                    and event.value == 0):
                consume_reference_middle_release = False
                continue

            # Pressing the physical scroll wheel (middle click) starts
            # calibration while idle. This press is consumed locally.
            if (
                not calibrating
                and event.code == ecodes.BTN_MIDDLE
                and event.value == 1
            ):
                if not recording and not playback:
                    begin_calibration()
                    continue

            # During calibration all buttons are consumed locally.
            # LEFT CLICK captures the next requested boundary.
            if calibrating:
                if calibration_awaiting_reference:
                    if event.code == ecodes.BTN_MIDDLE and event.value == 1:
                        finish_calibration_reference()
                elif event.code == ecodes.BTN_LEFT and event.value == 1:
                    capture_calibration_point()
                continue

            # SIDE is consumed locally as the recording toggle.
            if event.code == ecodes.BTN_SIDE:
                if event.value == 1:
                    if recording:
                        stop_recording()
                    else:
                        start_recording()
                continue

            # EXTRA is consumed locally. Short press/release toggles playback;
            # holding it for PLAYBACK_DELETE_HOLD_SECONDS clears RECORDING_DIR.
            if event.code == ecodes.BTN_EXTRA:
                handle_playback_button(event.value)
                continue

            # Normal mouse buttons are represented as a persistent bitmask.
            if event.code == ecodes.BTN_LEFT:
                mask = 0x01
            elif event.code == ecodes.BTN_RIGHT:
                mask = 0x02
            elif event.code == ecodes.BTN_MIDDLE:
                mask = 0x04
            else:
                mask = 0

            if mask:
                if event.value:
                    buttons |= mask
                else:
                    buttons &= ~mask

        # --------------------------------------------------
        # End of one Linux input frame
        # --------------------------------------------------
        elif (
            event.type == ecodes.EV_SYN
            and event.code == ecodes.SYN_REPORT
        ):

            if not playback:
                # Values actually sent to Windows. Using these same values for
                # tracking keeps the coordinate system tied to HID output.
                out_dx = clamp(dx)
                out_dy = clamp(dy)
                out_wheel = clamp(wheel)

                with position_lock:
                    current_x, current_y = x, y

                out_dx, out_dy = limit_motion(
                    current_x, current_y, out_dx, out_dy
                )

                with position_lock:
                    x += out_dx
                    y += out_dy
                    frame_x = x
                    frame_y = y

                # Record the same movement that Windows receives, together
                # with the resulting persistent x/y position.
                if recording:
                    timestamp = time.monotonic_ns() - record_start_time

                    record_writer.writerow([
                        timestamp,
                        out_dx,
                        out_dy,
                        out_wheel,
                        buttons,
                        frame_x,
                        frame_y
                    ])

                hid.write(bytes([
                    buttons,
                    signed_byte(out_dx),
                    signed_byte(out_dy),
                    signed_byte(out_wheel)
                ]))

            # During playback physical motion is intentionally not forwarded
            # and does not alter x/y. The physical device is still read so the
            # EXTRA button can stop playback immediately.

            dx = 0
            dy = 0
            wheel = 0


except KeyboardInterrupt:
    pass  # V34: playback print suppressed


finally:
    if recording:
        stop_recording()

    if playback:
        playback_stop_event.set()

    hid.close()
