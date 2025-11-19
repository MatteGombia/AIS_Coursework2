import cozmo
import time
from math import sqrt
from cozmo.util import degrees

def cozmo_program(robot: cozmo.robot.Robot):
    accel_sum = 0
    dt = 0.1
    duration = 1.0
    robot.drive_wheels(50, 50)
    t_start = time.time()
    elapsed = 0
    while elapsed < duration:
        accel = robot.accelerometer
        mag = sqrt(accel.x**2 + accel.y**2 + accel.z**2)
        accel_sum += mag * dt
        time.sleep(dt)
        elapsed = time.time() - t_start
    robot.drive_wheels(0, 0)
    print("Integrated acceleration (Σa·dt):", accel_sum)
    if accel_sum < 1000:
        print("Didn't move (low integrated acceleration)")
        robot.drive_wheels(-50, -50, duration=1.0)
        robot.turn_in_place(degrees(90)).wait_for_completed()
        time.sleep(0.1)
    else:
        print("Moved (by accelerometer sum)")
cozmo.run_program(cozmo_program)
