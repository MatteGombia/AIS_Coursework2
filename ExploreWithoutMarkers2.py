import cozmo
import time
from math import sqrt
from cozmo.util import degrees

def cozmo_program(robot: cozmo.robot.Robot):
    baseline = []
    for _ in range(10):
        accel = robot.accelerometer
        baseline.append((accel.x, accel.y, accel.z))
        time.sleep(0.05)
    
    avg_x = sum(a[0] for a in baseline) / len(baseline)
    avg_y = sum(a[1] for a in baseline) / len(baseline)
    avg_z = sum(a[2] for a in baseline) / len(baseline)
    
    variance_sum = 0
    dt = 0.1
    duration = 1.0
    
    robot.drive_wheels(50, 50)
    t_start = time.time()
    
    while time.time() - t_start < duration:
        accel = robot.accelerometer
        var_x = (accel.x - avg_x) ** 2
        var_y = (accel.y - avg_y) ** 2
        var_z = (accel.z - avg_z) ** 2
        variance_sum += sqrt(var_x + var_y + var_z)
        time.sleep(dt)
    
    robot.drive_wheels(0, 0)
    
    print(f"Movement variance: {variance_sum:.2f}")
    
    if variance_sum < 100:
        print("Low movement detected - likely stuck")
        robot.drive_wheels(-50, -50)
        time.sleep(1.0)
        robot.drive_wheels(0, 0)
        robot.turn_in_place(degrees(90)).wait_for_completed()
    else:
        print("Movement detected")

cozmo.run_program(cozmo_program)
