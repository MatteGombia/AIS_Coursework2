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
    max_attempts = 30
    attempts = 0
    consecutive_walls = 0
    
    print(f"Navigating from ({x_current:.0f}, {y_current:.0f}) to ({x_target:.0f}, {y_target:.0f})")
    
    while attempts < max_attempts:
        dx = x_target - x_current
        dy = y_target - y_current
        distance = math.hypot(dx, dy)
        
        if distance < tolerance:
            print(f"Target reached! Distance: {distance:.0f}mm")
            return True
        
        theta = math.atan2(dy, dx)
        print(f"Distance: {distance:.0f}mm, Angle: {math.degrees(theta):.0f}°")
        
        robot.turn_in_place(radians(theta)).wait_for_completed()
        time.sleep(0.2)
        
        move_distance = min(DISTANCE_PER_MOVE, distance)
        move_duration = move_distance / WHEEL_SPEED
        
        wall_hit = check_for_wall(robot, move_duration)
        
        if wall_hit:
            consecutive_walls += 1
            print(f"Wall hit {consecutive_walls} times, trying alternate path")
            
            if consecutive_walls % 2 == 1:
                robot.turn_in_place(degrees(90)).wait_for_completed()
            else:
                robot.turn_in_place(degrees(-90)).wait_for_completed()
            
            time.sleep(0.2)
            
            test_duration = DISTANCE_PER_MOVE / WHEEL_SPEED
            test_hit = check_for_wall(robot, test_duration)
            
            if not test_hit:
                x_current += DISTANCE_PER_MOVE * math.cos(theta + math.radians(90 if consecutive_walls % 2 == 1 else -90))
                y_current += DISTANCE_PER_MOVE * math.sin(theta + math.radians(90 if consecutive_walls % 2 == 1 else -90))
                consecutive_walls = 0
        else:
            x_current += move_distance * math.cos(theta)
            y_current += move_distance * math.sin(theta)
            consecutive_walls = 0
        
        attempts += 1
        time.sleep(0.2)
    
    print(f"Could not reach target after {max_attempts} attempts")
    return False

def navigate_rectilinear(robot, x_target, y_target, x_current=0, y_current=0):
    print(f"X-axis movement: {x_target - x_current:.0f}mm")
    if abs(x_target - x_current) > 50:
        navigate_with_avoidance(robot, x_target, y_current, x_current, y_current)
    
    time.sleep(0.5)
    
    print(f"Y-axis movement: {y_target - y_current:.0f}mm")
    if abs(y_target - y_current) > 50:
        navigate_with_avoidance(robot, x_target, y_target, x_target, y_current)

def navigate_to_cube(robot):
    print("Searching for cube...")
    robot.set_head_angle(degrees(0)).wait_for_completed()
    
    look_around = robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
    cube = None
    
    try:
        cube = robot.world.wait_for_observed_light_cube(timeout=30)
        print(f"Found cube!")
    except:
        print("No cube found")
    finally:
        look_around.stop()
    
    if cube:
        dx = cube.pose.position.x
        dy = cube.pose.position.y
        print(f"Cube at ({dx:.0f}, {dy:.0f})")
        navigate_with_avoidance(robot, dx, dy)
    else:
        print("Cannot navigate without cube")

def show_menu():
    print("\n" + "="*50)
    print("COZMO NAVIGATION")
    print("="*50)
    print("1. Diagonal Navigation")
    print("2. Rectilinear Navigation")
    print("3. Navigate to Cube")
    print("4. Calibrate")
    print("5. Exit")
    print("="*50)

def get_coords():
    try:
        x = float(input("X coordinate (mm): "))
        y = float(input("Y coordinate (mm): "))
        return x, y
    except ValueError:
        print("Invalid input")
        return None, None

def get_current_pos():
    use_origin = input("Use (0,0) as current position? (y/n): ").lower()
    if use_origin == 'y':
        return 0, 0
    
    try:
        x = float(input("Current X (mm): "))
        y = float(input("Current Y (mm): "))
        return x, y
    except ValueError:
        print("Using (0,0)")
        return 0, 0

def cozmo_program(robot: cozmo.robot.Robot):
    robot.camera.image_stream_enabled = True
    time.sleep(0.5)
    print("Robot ready!")
    
    while True:
        show_menu()
        choice = input("\nChoice (1-5): ")
        
        try:
            if choice == '1':
                print("\nDIAGONAL NAVIGATION")
                x_curr, y_curr = get_current_pos()
                x_targ, y_targ = get_coords()
                
                if x_targ is not None:
                    navigate_with_avoidance(robot, x_targ, y_targ, x_curr, y_curr)
            
            elif choice == '2':
                print("\nRECTILINEAR NAVIGATION")
                x_curr, y_curr = get_current_pos()
                x_targ, y_targ = get_coords()
                
                if x_targ is not None:
                    navigate_rectilinear(robot, x_targ, y_targ, x_curr, y_curr)
            
            elif choice == '3':
                print("\nNAVIGATE TO CUBE")
                navigate_to_cube(robot)
            
            elif choice == '4':
                print("\nCALIBRATION")
                if input("Start? (y/n): ").lower() == 'y':
                    calibrate(robot)
            
            elif choice == '5':
                print("\nGoodbye!")
                break
            
            else:
                print("Invalid choice")
        
        except KeyboardInterrupt:
            print("\nInterrupted")
            break
        except Exception as e:
            print(f"Error: {e}")
    
    robot.drive_wheels(0, 0)

if __name__ == '__main__':
    cozmo.run_program(cozmo_program)
