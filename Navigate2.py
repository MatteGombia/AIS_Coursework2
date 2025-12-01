import cozmo
from cozmo.util import degrees, radians
import math
import time
import numpy as np

WHEEL_SPEED = 150
DISTANCE_PER_MOVE = 100
CALIBRATION_TIME = 1.0
IMAGE_DIFF_THRESHOLD = 8

def calibrate(robot):
    print(f"Calibrating: driving at {WHEEL_SPEED}mm/s for {CALIBRATION_TIME}s")
    print("Measure the distance and update DISTANCE_PER_MOVE")
    robot.drive_wheels(WHEEL_SPEED, WHEEL_SPEED, duration=CALIBRATION_TIME)
    time.sleep(CALIBRATION_TIME + 0.5)

def check_for_wall(robot, move_duration):
    img1 = robot.world.latest_image
    if img1 is None:
        time.sleep(0.1)
        return False
    
    gray1 = np.array(img1.raw_image.convert('L'))
    
    robot.drive_wheels(WHEEL_SPEED, WHEEL_SPEED, duration=move_duration)
    time.sleep(move_duration)
    
    img2 = robot.world.latest_image
    if img2 is None:
        return False
    
    gray2 = np.array(img2.raw_image.convert('L'))
    difference = np.mean(np.abs(gray1.astype(float) - gray2.astype(float)))
    
    if difference < IMAGE_DIFF_THRESHOLD:
        robot.drive_wheels(0, 0)
        print(f"Wall detected! Difference: {difference:.2f}")
        
        robot.drive_wheels(-WHEEL_SPEED, -WHEEL_SPEED, duration=move_duration)
        time.sleep(move_duration)
        robot.drive_wheels(0, 0)
        time.sleep(0.2)
        return True
    
    return False

def navigate_with_avoidance(robot, x_target, y_target, x_current=0, y_current=0):
    tolerance = 50
    max_attempts = 40
    attempts = 0
    
    print(f"Navigating from ({x_current:.0f}, {y_current:.0f}) to ({x_target:.0f}, {y_target:.0f})")
    
    dx = x_target - x_current
    dy = y_target - y_current
    distance = math.hypot(dx, dy)
    theta = math.atan2(dy, dx)
    
    print(f"Initial: Distance {distance:.0f}mm, Angle {math.degrees(theta):.0f}°")
    robot.turn_in_place(radians(theta)).wait_for_completed()
    time.sleep(0.2)
    
    while attempts < max_attempts:
        dx = x_target - x_current
        dy = y_target - y_current
        distance = math.hypot(dx, dy)
        
        if distance < tolerance:
            print("Target reached!")
            return True
        
        move_distance = min(DISTANCE_PER_MOVE, distance)
        move_duration = move_distance / WHEEL_SPEED
        
        wall_hit = check_for_wall(robot, move_duration)
        
        if wall_hit:
            print("Wall hit, attempting to go around...")
            
            current_angle = math.atan2(dy, dx)
            
            robot.turn_in_place(degrees(90)).wait_for_completed()
            time.sleep(0.2)
            
            avoidance_duration = DISTANCE_PER_MOVE / WHEEL_SPEED
            side_hit = check_for_wall(robot, avoidance_duration)
            
            if side_hit:
                print("Right blocked, trying left...")
                robot.turn_in_place(degrees(-180)).wait_for_completed()
                time.sleep(0.2)
                
                left_hit = check_for_wall(robot, avoidance_duration)
                
                if not left_hit:
                    x_current += DISTANCE_PER_MOVE * math.cos(current_angle - math.pi/2)
