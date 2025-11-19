import cozmo
import time
from cozmo.util import degrees

def cozmo_program(robot: cozmo.robot.Robot):
    while True:
        start_pose = robot.pose
        start_left_speed = robot.left_wheel_speed_mmps
        start_right_speed = robot.right_wheel_speed_mmps
        robot.drive_wheels(50, 50)
        time.sleep(0.5)
        end_pose = robot.pose
        end_left_speed = robot.left_wheel_speed_mmps
        end_right_speed = robot.right_wheel_speed_mmps
        dx = end_pose.position.x - start_pose.position.x
        dy = end_pose.position.y - start_pose.position.y
        distance_moved = (dx**2 + dy**2)**0.5
        avg_wheel_speed = (abs(end_left_speed) + abs(end_right_speed)) / 2.0
        if avg_wheel_speed > 20 and distance_moved < 10:
            robot.drive_wheels(0, 0)
            robot.drive_wheels(-50, -50, duration=1.0)
            robot.turn_in_place(degrees(90)).wait_for_completed()
            time.sleep(0.1)
cozmo.run_program(cozmo_program)
