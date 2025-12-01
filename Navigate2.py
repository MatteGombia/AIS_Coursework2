import cozmo
from cozmo.util import degrees, radians
import math
import time
import numpy as np

WHEEL_SPEED = 250
DISTANCE_PER_MOVE = 100
CALIBRATION_TIME = 1.0
IMAGE_DIFF_THRESHOLD = 8
CALIBRATED_CONSTANT = 2.25

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

def normalise_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def navigate_with_avoidance(robot, x_target, y_target, x_current=0, y_current=0):
    tolerance = 50
    max_attempts = 40
    attempts = 0
    
    
    print(f"Navigating from ({x_current:.0f}, {y_current:.0f}) to ({x_target:.0f}, {y_target:.0f})")
    
    dx = x_target - x_current
    dy = y_target - y_current
    distance = math.hypot(dx, dy)
    robot_heading = math.atan2(dy, dx)
    
    print(f"Initial: Distance {distance:.0f}mm, Angle {math.degrees(robot_heading):.0f}°")
    robot.turn_in_place(radians(robot_heading)).wait_for_completed()
    time.sleep(0.2)
    
    while attempts < max_attempts:
        print(f"\n--- Attempt {attempts + 1} ---")
        print(f"Current position: ({x_current:.0f}, {y_current:.0f})")
        print(f"Current heading: {math.degrees(robot_heading):.0f}°")
        
        dx = x_target - x_current
        dy = y_target - y_current
        distance = math.hypot(dx, dy)
        
        print(f"Distance to target: {distance:.0f}mm")
        
        if distance < tolerance:
            print("Target reached!")
            add_position(x_current, y_current)
            return True
        
        move_distance = min(DISTANCE_PER_MOVE, distance)
        move_duration = move_distance / WHEEL_SPEED * CALIBRATED_CONSTANT
        
        print(f"Checking for wall ahead...")
        wall_hit = check_for_wall(robot, move_duration)
        
        if wall_hit:
            print(f">>> WALL HIT at attempt {attempts + 1} <<<")
            add_wall_detection(x_current, y_current)
            
            # Turn right 90°
            print("Turning right 90°")
            robot.turn_in_place(degrees(90)).wait_for_completed()
            robot_heading += math.pi/2
            robot_heading = normalise_angle(robot_heading)  # Keep angle in range
            print(f"New heading after right turn: {math.degrees(robot_heading):.0f}°")
            time.sleep(0.2)
            
            # Check right side
            avoidance_duration = DISTANCE_PER_MOVE / WHEEL_SPEED * CALIBRATED_CONSTANT
            print("Checking right side...")
            side_hit = check_for_wall(robot, avoidance_duration)
            
            if side_hit:
                print("Right side blocked, turning left 180°")
                add_wall_detection(x_current, y_current)
                robot.turn_in_place(degrees(-180)).wait_for_completed()
                robot_heading -= math.pi
                robot_heading = normalise_angle(robot_heading)
                print(f"New heading after left turn: {math.degrees(robot_heading):.0f}°")
                time.sleep(0.2)
                
                # Check left side
                print("Checking left side...")
                left_hit = check_for_wall(robot, avoidance_duration)
                
                if not left_hit:
                    print("Left side clear, moving")
                    x_current += DISTANCE_PER_MOVE * math.cos(robot_heading)
                    y_current += DISTANCE_PER_MOVE * math.sin(robot_heading)
                    add_position(x_current, y_current)
                    print(f"Moved left to ({x_current:.0f}, {y_current:.0f})")
                else:
                    add_wall_detection(x_current, y_current)
                    print("Both sides blocked! Staying in place")
            else:
                print("Right side clear, moving")
                x_current += DISTANCE_PER_MOVE * math.cos(robot_heading)
                y_current += DISTANCE_PER_MOVE * math.sin(robot_heading)
                add_position(x_current, y_current)
                print(f"Moved right to ({x_current:.0f}, {y_current:.0f})")
            
            # REORIENT TOWARDS TARGET
            dx = x_target - x_current
            dy = y_target - y_current
            target_angle = math.atan2(dy, dx)
            
            turn_amount = normalise_angle(target_angle - robot_heading)
            
            print(f">>> REORIENTING <<<")
            print(f"Target angle: {math.degrees(target_angle):.0f}°")
            print(f"Current heading: {math.degrees(robot_heading):.0f}°")
            print(f"Turn amount: {math.degrees(turn_amount):.0f}°")
            
            robot.turn_in_place(radians(turn_amount)).wait_for_completed()
            robot_heading = target_angle
            print(f"New heading after reorientation: {math.degrees(robot_heading):.0f}°")
            time.sleep(0.2)
            
        else:
            print("No wall detected, moving forward")
            x_current += move_distance * math.cos(robot_heading)
            y_current += move_distance * math.sin(robot_heading)
            add_position(x_current, y_current)
            print(f"Moved forward to ({x_current:.0f}, {y_current:.0f})")
        
        attempts += 1
        time.sleep(0.2)
    
    print("\n!!! Could not reach target !!!")
    add_position(x_current, y_current)
    return False

def move_straight_with_avoidance(robot, target_distance, target_angle):
    tolerance = 50
    max_attempts = 30
    attempts = 0
    x_current = 0
    y_current = 0
    x_target = target_distance * math.cos(target_angle)
    y_target = target_distance * math.sin(target_angle)
    robot_heading = target_angle
    
    print(f"Moving {target_distance:.0f}mm at {math.degrees(target_angle):.0f}°")
    robot.turn_in_place(radians(target_angle)).wait_for_completed()
    time.sleep(0.2)
    
    while attempts < max_attempts:
        dx = x_target - x_current
        dy = y_target - y_current
        remaining = math.hypot(dx, dy)
        
        if remaining < tolerance:
            print("Target reached!")
            return True
        
        move_distance = min(DISTANCE_PER_MOVE, remaining)
        move_duration = move_distance / WHEEL_SPEED * CALIBRATED_CONSTANT
        
        # CHECK for wall WITHOUT moving
        wall_hit = check_for_wall(robot, move_duration)
        
        if wall_hit:
            add_wall_detection(x_current, y_current)
            print(f"Wall detected at attempt {attempts}, avoiding...")
            
            # Turn right 90°
            robot.turn_in_place(degrees(90)).wait_for_completed()
            robot_heading += math.pi/2
            time.sleep(0.2)
            
            # Check side
            avoidance_duration = DISTANCE_PER_MOVE / WHEEL_SPEED * CALIBRATED_CONSTANT
            side_hit = check_for_wall(robot, avoidance_duration)
            
            if side_hit:
                # Turn left 180° (now facing left from original obstacle)
                add_wall_detection(x_current, y_current)
                robot.turn_in_place(degrees(-180)).wait_for_completed()
                robot_heading -= math.pi
                time.sleep(0.2)
                
                # Try left side
                left_hit = check_for_wall(robot, avoidance_duration)
                
                if not left_hit:
                    # Move left successfully
                    x_current += DISTANCE_PER_MOVE * math.cos(robot_heading)
                    y_current += DISTANCE_PER_MOVE * math.sin(robot_heading)
                    add_position(x_current, y_current)
                else:
                    print("Completely blocked, staying in place")
            else:
                # Move right successfully
                x_current += DISTANCE_PER_MOVE * math.cos(robot_heading)
                y_current += DISTANCE_PER_MOVE * math.sin(robot_heading)
                add_position(x_current, y_current)
            
            # ALWAYS reorient after avoidance
            dx = x_target - x_current
            dy = y_target - y_current
            target_angle_new = math.atan2(dy, dx)
            turn_amount = normalise_angle(target_angle_new - robot_heading)
            
            print(f"Reorienting: turning {math.degrees(turn_amount):.0f}° towards target")
            robot.turn_in_place(radians(turn_amount)).wait_for_completed()
            robot_heading = target_angle_new
            time.sleep(0.2)
            
        else:
            # No wall detected - move forward
            x_current += move_distance * math.cos(robot_heading)
            y_current += move_distance * math.sin(robot_heading)
            add_position(x_current, y_current)
        
        attempts += 1
        time.sleep(0.2)
    
    print("Could not reach target")
    return False

def navigate_rectilinear(robot, x_target, y_target, x_current=0, y_current=0):
    dx = x_target - x_current
    dy = y_target - y_current
    
    if abs(dx) > 50:
        print(f"Moving along X-axis: {dx:.0f}mm")
        angle = 0 if dx > 0 else math.pi
        move_straight_with_avoidance(robot, abs(dx), angle)
        time.sleep(0.5)
    
    if abs(dy) > 50:
        print(f"Moving along Y-axis: {dy:.0f}mm")
        angle = math.pi/2 if dy > 0 else -math.pi/2
        move_straight_with_avoidance(robot, abs(dy), angle)

def navigate_to_cube(robot):
    print("Searching for cube")
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
                print("\nClosing")
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
