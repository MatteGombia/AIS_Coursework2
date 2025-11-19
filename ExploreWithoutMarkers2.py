import cozmo
import time
from math import sqrt
from cozmo.util import degrees

def cozmo_program(robot: cozmo.robot.Robot):
    baseline_readings = []
    for _ in range(10):
        accel = robot.accelerometer
        mag = sqrt(accel.x**2 + accel.y**2 + accel.z**2)
        baseline_readings.append(mag)
        time.sleep(0.05)
    baseline = sum(baseline_readings) / len(baseline_readings)
    
    accel_changes = 0
    dt = 0.1
    duration = 1.0
    
    robot.drive_wheels(50, 50)
    t_start = time.time()
    
    while time.time() - t_start < duration:
        accel = robot.accelerometer
        mag = sqrt(accel.x**2 + accel.y**2 + accel.z**2)
        accel_changes += abs(mag - baseline) * dt
        time.sleep(dt)
    
    robot.drive_wheels(0, 0)
    
    print(f"Acceleration changes: {accel_changes:.2f}")
    
    if accel_changes < 50:
        print("Low acceleration changes - robot may be stuck")
        robot.drive_wheels(-50, -50)
        time.sleep(1.0)
        robot.drive_wheels(0, 0)
        robot.turn_in_place(degrees(90)).wait_for_completed()
    else:
        print("Normal acceleration detected")

cozmo.run_program(cozmo_program)
