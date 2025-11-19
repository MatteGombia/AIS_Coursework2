import cozmo
import time
from cozmo.util import degrees

def cozmo_program(robot: cozmo.robot.Robot):
    while True:
        start_pose = robot.pose
        robot.drive_wheels(50, 50)
        time.sleep(0.5)
        end_pose = robot.pose
        dx = end_pose.position.x - start_pose.position.x
        dy = end_pose.position.y - start_pose.position.y
        distance_moved = (dx**2 + dy**2)**0.5
        if distance_moved < 10:
            robot.drive_wheels(0, 0)
            robot.drive_wheels(-50, -50, duration=1.0)
            robot.turn_in_place(degrees(90)).wait_for_completed()
            time.sleep(0.1)
        else:
            pass
cozmo.run_program(cozmo_program)
