"""
line_follower_v3.py

Line follower based on v2, with:
  - all tunable parameters grouped at the top (TUNING section)
  - state statistics printed every 10 s (to see where time is lost)
  - optional debug print (DEBUG flag)
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

# ---------------------------------------------------------------------------
# Calibration (measured: white ~765, line ~340, corners ~300,
# red/yellow checkpoints ~830, blue checkpoint ~575)
# ---------------------------------------------------------------------------
BLACK_VALUE = 340        # typical value on the black line
BLACK_THRESHOLD = 450    # above this = white (blue checkpoint must count as white)

# ---------------------------------------------------------------------------
# TUNING - change ONE value at a time and compare the Final Score
# ---------------------------------------------------------------------------
cruiseVelocity = 5.9
maxVelocity = 6.28               # e-puck motor limit

KP = 0.02                        # experiment 2: try 0.03, 0.04

SHARP_TURN_THRESHOLD = 0.75     # experiment 1: try 0.75
SHARP_TURN_OUTER_SPEED = 4.0     # experiment 3: try 6.0
SHARP_TURN_INNER_SPEED = -1.0    # experiment 3: try -2.0

CORNER_SPEED = 1.5               # experiment 4: try 3.0
SEARCH_SPEED = 2.0               # rotation speed when the line is lost

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
# Main loop
# ---------------------------------------------------------------------------
last_side = 0   # -1 = line last seen on the left, +1 = on the right, 0 = unknown
step = 0
state_count = {"FOLLOW": 0, "SHARP": 0, "CORNER": 0, "SEARCH": 0}

while robot.step(timeStep) != -1:
    step += 1
    raw = [g.getValue() for g in ground_sensors]
    weights = [normalize(v) for v in raw]

    if step <= SKIP_STEPS:
        continue

    if max(weights) < LINE_PRESENT_THRESHOLD:
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