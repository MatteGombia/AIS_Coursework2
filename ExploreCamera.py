import cozmo
import time
from cozmo.util import degrees
import numpy as np

def explorewithcamera(robot: cozmo.robot.Robot):
    robot.camera.image_stream_enabled = True
    while True:
        img1 = robot.world.latest_image
        if img1 is None:
            time.sleep(0.1)
            continue
        raw1 = img1.raw_image
        gray1 = np.array(raw1.convert('L'))
        robot.drive_wheels(150, 150)
        time.sleep(0.5)
        robot.drive_wheels(0, 0)
        time.sleep(0.1)
        img2 = robot.world.latest_image
        if img2 is None:
            continue 
        raw2 = img2.raw_image
        gray2 = np.array(raw2.convert('L'))
        diff = np.abs(gray1.astype(float) - gray2.astype(float))
        movement_score = np.mean(diff)
        print(f"Image difference: {movement_score:.2f}")
        if movement_score <5:
            print("No visual movement detected")
            robot.drive_wheels(-150, -150)
            time.sleep(1.0)
            robot.drive_wheels(0, 0)
            robot.turn_in_place(degrees(90)).wait_for_completed()
        else:
            print("Visual movement detected")
cozmo.run_program(explorewithcamera)
