import cozmo
import time
from cozmo.util import degrees

def cozmo_program(robot: cozmo.robot.Robot):
    # Method 1: Use pose tracking (most reliable)
    start_pose = robot.pose
    
    # Drive forward
    robot.drive_wheels(50, 50)
    time.sleep(1.0)
    robot.drive_wheels(0, 0)
    
    # Check if robot moved by comparing poses
    end_pose = robot.pose
    distance_moved = start_pose.position.distance_to(end_pose.position)
    
    print(f"Distance moved: {distance_moved:.2f} mm")
    
    if distance_moved < 10:  # Less than 10mm means stuck
        print("Robot didn't move - likely stuck!")
        # Back up and turn
        robot.drive_wheels(-50, -50)
        time.sleep(1.0)
        robot.drive_wheels(0, 0)
        robot.turn_in_place(degrees(90)).wait_for_completed()
    else:
        print("Robot moved successfully")

cozmo.run_program(cozmo_program)
