import cozmo
import time
from cozmo.util import degrees
def cozmo_program(robot: cozmo.robot.Robot):
    while True:
        robot.drive_wheels(50, 50)
        time.sleep(0.5)
        if robot.is_moving:
            continue
        else:
            robot.drive_wheels(-50, -50, duration=0.5)
            robot.turn_in_place(degrees(90)).wait_for_completed()
            time.sleep(0.1)
cozmo.run_program(cozmo_program)
