import cozmo
import time
from math import sqrt
from cozmo.util import degrees

def cozmo_program(robot: cozmo.robot.Robot):
    start_pose = robot.pose
    
    robot.drive_wheels(50, 50)
    time.sleep(1.0)
    robot.drive_wheels(0, 0)
    
    end_pose = robot.pose
    
    dx = end_pose.position.x - start_pose.position.x
    dy = end_pose.position.y - start_pose.position.y
    distance_moved = sqrt(dx**2 + dy**2)
    
    print(f"Distance moved: {distance_moved:.2f} mm")
    
    if distance_moved < 10:
        print("Robot didn't move - likely stuck!")
        robot.drive_wheels(-50, -50)
        time.sleep(1.0)
        robot.drive_wheels(0, 0)
        robot.turn_in_place(degrees(90)).wait_for_completed()
    else:
        print("Robot moved successfully")

cozmo.run_program(cozmo_program)
