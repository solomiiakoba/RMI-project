"""
line_follower_v4.py

Based on the team's line_follower_v3.py (ground-sensor line following with
FOLLOW / SHARP / CORNER / SEARCH states). This version adds:

  - camera-based checkpoint colour detection (red / blue / yellow)
  - direction check: checkpoints must appear in the order red -> blue -> yellow
  - a first version of the "wrong way" recovery manoeuvre (180 degree turn)

Everything from v3 is kept as-is; new code is grouped under CHECKPOINTS /
DIRECTION so it is easy to review as a diff against v3.
"""

from controller import Robot

# ---------------------------------------------------------------------------
# Initialization
# ---------------------------------------------------------------------------
robot = Robot()
timeStep = int(robot.getBasicTimeStep())

leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# GS0 = left, GS1 = center, GS2 = right
ground_sensors = [robot.getDevice('gs' + str(x)) for x in range(3)]
for gs in ground_sensors:
    gs.enable(timeStep)

camera = robot.getDevice("camera")
camera.enable(timeStep)
CAM_WIDTH = camera.getWidth()
CAM_HEIGHT = camera.getHeight()

# ---------------------------------------------------------------------------
# Calibration (measured: white ~765, line ~340, corners ~300,
# red/yellow checkpoints ~830, blue checkpoint ~575)
# ---------------------------------------------------------------------------
BLACK_VALUE = 340        # typical value on the black line
BLACK_THRESHOLD = 450    # above this = white (blue checkpoint must count as white)

# ---------------------------------------------------------------------------
# TUNING - change ONE value at a time and compare the Final Score
# ---------------------------------------------------------------------------
cruiseVelocity = 6.28            # máximo do e-puck
maxVelocity = 6.28               # e-puck motor limit

KP = 0.035                       # proporcional: maior = mais agressivo na correcção

SHARP_TURN_THRESHOLD = 0.75
SHARP_TURN_OUTER_SPEED = 6.28    # roda exterior ao máximo nas curvas fechadas
SHARP_TURN_INNER_SPEED = -2.0    # roda interior mais agressiva

CORNER_SPEED = 3.5               # era 1.5 - corner mais rápido
SEARCH_SPEED = 2.0               # rotation speed when the line is lost

# --- CHECKPOINTS / DIRECTION (new in v4) ------------------------------------
# NOTE: these RGB thresholds are placeholders - calibrate them the same way
# the ground sensors were calibrated (print camera samples while the robot
# passes over each checkpoint colour, then adjust). A small helper is left
# below (CAMERA_DEBUG) to make this easy.
CAMERA_DEBUG = False              # set True only when calibrating colours

# how many pixels around the center of the image to average (robustness to noise)
SAMPLE_HALF_SIZE = 3

# minimum "dominance" a channel needs over the others to call a colour
COLOR_MARGIN = 30

# how many consecutive steps a colour must be seen before we trust it
# (avoids triggering twice on the same checkpoint, or on camera noise)
CHECKPOINT_CONFIRM_STEPS = 1
CHECKPOINT_COOLDOWN_STEPS = int(1000 / timeStep)  # ignore new checkpoints for ~1s after one is confirmed

# expected order while going the CORRECT (clockwise) way
CHECKPOINT_SEQUENCE = ["red", "blue", "yellow"]

# 180-degree recovery turn parameters.
# TURN_AROUND_STEPS is open-loop: the robot spins for a fixed number of steps.
# To calibrate: run the simulation, trigger a wrong-way, and measure visually
# whether the robot ends up facing the opposite direction.
# - Too short → undershoots (robot still faces roughly same direction)
# - Too long  → overshoots (robot ends up facing ~270° instead of 180°)
# e-puck wheel base ≈ 0.052 m, wheel radius ≈ 0.021 m.
# For a 180° turn at TURN_AROUND_SPEED rad/s each wheel:
#   arc = pi * wheelBase = 0.163 m
#   time = arc / (TURN_AROUND_SPEED * wheelRadius) = 0.163 / (3.0 * 0.021) ≈ 2.6 s
# → at timeStep=32ms: ~81 steps. int(1200/timeStep) ≈ 37 steps which is too few.
# Adjusted to ~2.6 s worth of steps:
TURN_AROUND_STEPS = int(2600 / timeStep)
TURN_AROUND_SPEED = 3.0

# ---------------------------------------------------------------------------
# Other constants
# ---------------------------------------------------------------------------
LINE_PRESENT_THRESHOLD = 0.30
SENSOR_WEIGHTS = [-1.0, 0.0, 1.0]
SKIP_STEPS = 5                   # ignore the first steps (wrong readings while the robot is placed)

DEBUG = False                    # True = print sensors and speeds about once per second
STATS_EVERY = int(10000 / timeStep)   # print state statistics every ~10 s of simulation


def normalize(value):
    """
    Convert a raw sensor value to a weight between 0 (white) and 1 (black).
    Anything above BLACK_THRESHOLD counts as fully white, so the blue
    checkpoint (~575) does not look like part of the line.
    """
    v = (BLACK_THRESHOLD - value) / (BLACK_THRESHOLD - BLACK_VALUE)
    return max(0.0, min(1.0, v))


def clamp(value, low, high):
    return max(low, min(high, value))


# ---------------------------------------------------------------------------
# CHECKPOINTS / DIRECTION - new in v4
# ---------------------------------------------------------------------------
def sample_camera_color():
    """
    Reads a small patch around the center of the camera image and returns
    its average (r, g, b), each 0-255.

    Assumption: checkpoints are wide enough / close enough to the ground that
    a patch near the bottom-center of the image sees mostly the checkpoint
    colour when the robot is on top of / right before one. If in practice the
    checkpoint appears elsewhere in the frame (e.g. it's seen from farther
    away), move SAMPLE_Y down/up - use CAMERA_DEBUG to check.
    """
    image = camera.getImage()
    cx = CAM_WIDTH // 2
    cy = int(CAM_HEIGHT * 0.8)  # lower part of the image = closer to the robot

    r_total = g_total = b_total = 0
    count = 0
    for dx in range(-SAMPLE_HALF_SIZE, SAMPLE_HALF_SIZE + 1):
        for dy in range(-SAMPLE_HALF_SIZE, SAMPLE_HALF_SIZE + 1):
            x = clamp(cx + dx, 0, CAM_WIDTH - 1)
            y = clamp(cy + dy, 0, CAM_HEIGHT - 1)
            r_total += camera.imageGetRed(image, CAM_WIDTH, x, y)
            g_total += camera.imageGetGreen(image, CAM_WIDTH, x, y)
            b_total += camera.imageGetBlue(image, CAM_WIDTH, x, y)
            count += 1

    return r_total / count, g_total / count, b_total / count


def classify_color(r, g, b):
    """
    Returns "red", "blue", "yellow", or None (floor / line / nothing relevant).

    Rules derived from real camera samples in this environment:
      red    ~ (high r, low g, low b)  -> r dominates both g and b
      blue   ~ (low r, mid-high g, mid-high b) / cyan-ish
               The blue checkpoint is NOT pure blue: g is also elevated.
               Key feature: r is clearly lower than both g and b.
               Use b - r > MARGIN (not b - max(r,g)) so that teal/cyan passes.
      yellow ~ (high r, high g, low b) -> r and g both dominate b

    The floor is either near-black (~23,24,30) or near-white (~234,234,237),
    both of which are achromatic (all channels roughly equal) - they will
    produce no margin > COLOR_MARGIN so they fall through to None.
    """
    # Red: r clearly above both g and b
    if r - max(g, b) > COLOR_MARGIN and r > 80:
        return "red"
    # Blue/cyan: r is clearly below b AND (b dominates g OR g is also elevated above r)
    # Samples: ~(33,67,72) -> b-r=39>30, r is the clear minimum channel
    if (b - r) > COLOR_MARGIN and b > 40 and r < g and r < b:
        return "blue"
    # Yellow: r and g both clearly above b
    if (r - b) > COLOR_MARGIN and (g - b) > COLOR_MARGIN and r > 80 and g > 80:
        return "yellow"
    return None


# Set to None so the robot works correctly regardless of where it starts on
# the loop. The first confirmed checkpoint becomes the baseline; direction is
# only judged from the second checkpoint onward. This is safe for any maze.
ASSUMED_START_CHECKPOINT = None

# state for the checkpoint sequence / direction check
last_confirmed_color = ASSUMED_START_CHECKPOINT   # baseline: last checkpoint colour actually seen
pending_color = None             # colour currently being confirmed
pending_count = 0                # for how many steps it's been seen
cooldown = 0                     # steps left before we can trust a new colour
wrong_streak = 0                 # consecutive ambiguous/reverse checkpoints seen
turning_around_steps_left = 0    # >0 while executing the 180-degree manoeuvre

# --- debug-only state (does not affect behaviour) ---------------------------
_last_logged_color = "<start>"   # forces the first sample to always be logged


def log_color_transition(r, g, b, color, step, robot_time):
    """
    Prints a line EVERY TIME the classified colour changes (including
    None -> colour and colour -> None), regardless of CAMERA_DEBUG's periodic
    interval. This matters because a checkpoint may only be visible for a
    handful of simulation steps, so a fixed "print every N steps" can miss it
    completely - an edge-triggered log never does.

    Also prints the raw channel margins (r-g, r-b, b-r, b-g, g-b) so you can
    see exactly how close a sample was to crossing COLOR_MARGIN, which is
    the key info needed to fix a colour that classify_color() is missing
    (e.g. a teal/cyan-ish blue where g is also high).
    """
    global _last_logged_color
    if color == _last_logged_color:
        return
    margins = {
        "r-g": r - g, "r-b": r - b,
        "b-r": b - r, "b-g": b - g,
        "g-r": g - r, "g-b": g - b,
    }
    margins_str = "  ".join(f"{k}={v:+.0f}" for k, v in margins.items())
    print(f"[camera] t={robot_time:.1f}s step={step}  "
          f"rgb=({r:.0f},{g:.0f},{b:.0f})  classified={color!r}  "
          f"was={_last_logged_color!r}  {margins_str}")
    _last_logged_color = color


def update_direction_check(raw_r_g_b):
    """
    Detects checkpoint colours and decides if the robot is going the wrong way.

    With only 3 checkpoints (red, blue, yellow) in a cycle, a single
    "out-of-order" observation is AMBIGUOUS: seeing colour C after baseline B
    where C == previous(B) could mean either:
      (a) the robot is going backwards, OR
      (b) the robot missed one checkpoint (detection failure) and is still
          going forward (skipped one step in the sequence).

    To avoid false wrong-way triggers from missed detections, we require TWO
    consecutive checkpoints that are BOTH consistent with the reverse direction
    before declaring wrong_way. A single anomaly just updates the baseline
    (best-effort) so the next comparison has a fresh reference point.

    State machine per confirmed checkpoint:
      - baseline is None  -> set baseline, no verdict
      - color == baseline -> same colour twice, ignore (spurious re-detection)
      - color == next(baseline) -> correct clockwise, update baseline, reset wrong streak
      - color == prev(baseline) (= next-next, same thing with 3 items) ->
            ambiguous: could be missed detection going forward, OR reverse.
            Increment wrong_streak. If wrong_streak >= 2 -> wrong_way.
            Either way update baseline so the next checkpoint gives fresh info.

    Returns "ok" or "wrong_way".
    """
    global last_confirmed_color, pending_color, pending_count, cooldown
    global wrong_streak

    if cooldown > 0:
        cooldown -= 1
        return "ok"

    color = classify_color(*raw_r_g_b)

    if color is None:
        pending_color = None
        pending_count = 0
        return "ok"

    if color == pending_color:
        pending_count += 1
    else:
        pending_color = color
        pending_count = 1

    if pending_count < CHECKPOINT_CONFIRM_STEPS:
        return "ok"

    # colour confirmed - reset debounce state and start cooldown
    pending_color = None
    pending_count = 0
    cooldown = CHECKPOINT_COOLDOWN_STEPS

    if last_confirmed_color is None:
        last_confirmed_color = color
        wrong_streak = 0
        print(f"[checkpoint] '{color}' confirmed - baseline established, no verdict yet")
        return "ok"

    if color == last_confirmed_color:
        print(f"[checkpoint] '{color}' confirmed again (same as baseline) - ignoring")
        return "ok"

    base_index = CHECKPOINT_SEQUENCE.index(last_confirmed_color)
    next_cw   = CHECKPOINT_SEQUENCE[(base_index + 1) % len(CHECKPOINT_SEQUENCE)]
    next_ccw  = CHECKPOINT_SEQUENCE[(base_index - 1) % len(CHECKPOINT_SEQUENCE)]

    if color == next_cw:
        # Correct clockwise step
        wrong_streak = 0
        last_confirmed_color = color
        print(f"[checkpoint] '{color}' confirmed - clockwise OK "
              f"(after '{CHECKPOINT_SEQUENCE[base_index]}')")
        return "ok"

    # With 3 checkpoints next_ccw == next_cw's next, so this is always the
    # "other" colour - ambiguous between skipped-forward or reversed.
    if color == next_ccw:
        wrong_streak += 1
        last_confirmed_color = color   # update baseline regardless
        print(f"[checkpoint] '{color}' after '{last_confirmed_color}' is ambiguous "
              f"(skipped or reversed) - wrong_streak={wrong_streak}")
        if wrong_streak >= 2:
            wrong_streak = 0
            print(f"[checkpoint] wrong_streak reached 2 -> WRONG WAY confirmed!")
            return "wrong_way"
        return "ok"

    # Should not happen with a 3-colour cycle, but be safe
    print(f"[checkpoint] unexpected colour '{color}' - ignoring")
    return "ok"


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
last_side = 0   # -1 = line last seen on the left, +1 = on the right, 0 = unknown
step = 0
state_count = {"FOLLOW": 0, "SHARP": 0, "CORNER": 0, "SEARCH": 0, "TURN_AROUND": 0}

while robot.step(timeStep) != -1:
    step += 1
    raw = [g.getValue() for g in ground_sensors]
    weights = [normalize(v) for v in raw]

    if step <= SKIP_STEPS:
        continue

    # --- direction check runs every step, independently of the line state ---
    rgb = sample_camera_color()

    # Edge-triggered log: always active while calibrating (cheap - only prints
    # on change), independent of CAMERA_DEBUG. Set to False once colours are
    # confirmed to work reliably, to reduce log spam in the final runs.
    log_color_transition(*rgb, classify_color(*rgb), step, robot.getTime())

    if CAMERA_DEBUG and step % 5 == 0:
        # NOTE: printed unconditionally on RAW rgb values (not on classify_color's
        # result), specifically so we can see what blue actually looks like even
        # though classify_color() currently seems to never recognize it as such
        # (it stays 'None' the whole time the robot is over it, so the
        # edge-triggered log above never fires for it either).
        r, g, b = rgb
        print(f"[camera heartbeat] t={robot.getTime():.1f}s rgb=({r:.0f},{g:.0f},{b:.0f})  "
              f"pending={pending_color}({pending_count}/{CHECKPOINT_CONFIRM_STEPS})  "
              f"cooldown={cooldown}  baseline={last_confirmed_color}")

    if turning_around_steps_left == 0:
        direction_result = update_direction_check(rgb)
        if direction_result == "wrong_way":
            turning_around_steps_left = TURN_AROUND_STEPS
            # After turning around we are now travelling the other way on the
            # loop. Forget the baseline and reset the wrong_streak so the
            # robot re-establishes direction from scratch after the turn.
            last_confirmed_color = None
            wrong_streak = 0

    if turning_around_steps_left > 0:
        # TURN_AROUND: spin in place (open-loop). This is a first version -
        # test the actual angle reached and adjust TURN_AROUND_STEPS /
        # TURN_AROUND_SPEED accordingly.
        state = "TURN_AROUND"
        left_speed, right_speed = TURN_AROUND_SPEED, -TURN_AROUND_SPEED
        turning_around_steps_left -= 1

    elif max(weights) < LINE_PRESENT_THRESHOLD:
        # LINE LOST: rotate toward the side where the line was last seen
        state = "SEARCH"
        if last_side < 0:
            left_speed, right_speed = -SEARCH_SPEED, SEARCH_SPEED
        elif last_side > 0:
            left_speed, right_speed = SEARCH_SPEED, -SEARCH_SPEED
        else:
            left_speed, right_speed = SEARCH_SPEED, SEARCH_SPEED

    elif min(weights) > 0.7:
        # CORNER: all three sensors on black -> slow down
        state = "CORNER"
        left_speed = right_speed = CORNER_SPEED

    else:
        # NORMAL LINE FOLLOWING
        error = sum(w * sw for w, sw in zip(weights, SENSOR_WEIGHTS)) / sum(weights)

        # remember on which side the line was last seen
        if error < -0.1:
            last_side = -1
        elif error > 0.1:
            last_side = 1

        if abs(error) >= SHARP_TURN_THRESHOLD:
            # large error: pivot almost in place
            state = "SHARP"
            if error < 0:
                left_speed, right_speed = SHARP_TURN_INNER_SPEED, SHARP_TURN_OUTER_SPEED
            else:
                left_speed, right_speed = SHARP_TURN_OUTER_SPEED, SHARP_TURN_INNER_SPEED
        else:
            # small error: proportional correction while driving
            state = "FOLLOW"
            correction = KP * error * 100
            left_speed = cruiseVelocity + correction
            right_speed = cruiseVelocity - correction

    left_speed = clamp(left_speed, -maxVelocity, maxVelocity)
    right_speed = clamp(right_speed, -maxVelocity, maxVelocity)
    leftMotor.setVelocity(left_speed)
    rightMotor.setVelocity(right_speed)

    # statistics: share of time spent in each state
    state_count[state] += 1
    if step % STATS_EVERY == 0:
        total = sum(state_count.values())
        stats = "  ".join(f"{k}={100 * v / total:.0f}%" for k, v in state_count.items())
        print(f"t={robot.getTime():.0f}s  {stats}")

    if DEBUG and step % 30 == 0:
        print(f"{state:7s} gs={[int(v) for v in raw]}  L={left_speed:.2f}  R={right_speed:.2f}")
