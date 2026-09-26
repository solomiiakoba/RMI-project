from controller import Robot

robot = Robot()
timeStep = int(robot.getBasicTimeStep())

leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

gs = [robot.getDevice('gs' + str(i)) for i in range(3)]
for s in gs:
    s.enable(timeStep)

speed = 1.5          # slow speed so the sensors measure accurately
mins = [9999, 9999, 9999]
maxs = [0, 0, 0]
step = 0

while robot.step(timeStep) != -1:
    v = [s.getValue() for s in gs]

    # keep track of the minimum and maximum value for each sensor
    for i in range(3):
        mins[i] = min(mins[i], v[i])
        maxs[i] = max(maxs[i], v[i])

    # professor's line-following logic, just slower
    if v[2] > 500:
        leftMotor.setVelocity(0.1 * speed)
        rightMotor.setVelocity(1.2 * speed)
    elif v[0] > 500:
        leftMotor.setVelocity(1.2 * speed)
        rightMotor.setVelocity(0.1 * speed)
    else:
        leftMotor.setVelocity(speed)
        rightMotor.setVelocity(speed)

    # print roughly once per second
    step += 1
    if step % 30 == 0:
        print(f"now: {[int(x) for x in v]}   min: {[int(x) for x in mins]}   max: {[int(x) for x in maxs]}")
