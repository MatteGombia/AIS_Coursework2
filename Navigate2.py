import cozmo
from cozmo.util import degrees, radians
import math
import time
import numpy as np

WHEEL_SPEED = 150  # mm/s (reduced for better control)
DISTANCE_PER_MOVE = 100  # mm 
CALIBRATION_TIME = 1.0  # seconds
IMAGE_DIFF_THRESHOLD = 5  # Threshold for detecting walls

def calibrate(robot):
    print(f"Calibrating: Robot will drive at speed {WHEEL_SPEED} for {CALIBRATION_TIME}s")
    print("Measure the distance traveled with a ruler")
    robot.drive_wheels(WHEEL_SPEED, WHEEL_SPEED, duration=CALIBRATION_TIME)
    time.sleep(CALIBRATION_TIME + 0.5)
    print(f"Update DISTANCE_PER_MOVE constant with the measured distance")

def check_for_wall(robot, move_duration):
    """
    Move forward and check if we hit a wall by comparing images.
    Returns True if wall detected, False otherwise.
    """
    # Get first image
    img1 = robot.world.latest_image
    if img1 is None:
        time.sleep(0.1)
        return False
    
    gray1 = np.array(img1.raw_image.convert('L'))
    
    # Move forward
    robot.drive_wheels(WHEEL_SPEED, WHEEL_SPEED, duration=move_duration)
    time.sleep(move_duration)
    
    # Get second image
    img2 = robot.world.latest_image
    if img2 is None:
        return False
    
    gray2 = np.array(img2.raw_image.convert('L'))
    
    # Compare images - if not much changed, we hit a wall
    difference = np.mean(np.abs(gray1.astype(float) - gray2.astype(float)))
    
    if difference < IMAGE_DIFF_THRESHOLD:
        # Hit a wall!
        robot.drive_wheels(0, 0)
        print(f"Wall detected! Image difference: {difference:.2f}")
        
        # Back up
        robot.drive_wheels(-WHEEL_SPEED, -WHEEL_SPEED, duration=move_duration)
        time.sleep(move_duration)
        robot.drive_wheels(0, 0)
        time.sleep(0.2)
        
        return True
    
    return False

def navigate_diagonal_with_avoidance(robot, x_target, y_target, x_current=0, y_current=0, max_attempts=20):
    """
    Navigate to target with obstacle avoidance.
    """
    attempts = 0
    tolerance = 50  # mm - how close we need to get to the target
    
    print(f"Navigating from ({x_current:.1f}, {y_current:.1f}) to ({x_target:.1f}, {y_target:.1f})")
    
    while attempts < max_attempts:
        # Calculate distance and angle to target
        dx = x_target - x_current
        dy = y_target - y_current
        distance = math.hypot(dx, dy)
        
        # Check if we've reached the target
        if distance < tolerance:
            print(f"Target reached! Final distance: {distance:.1f}mm")
            return True
        
        theta = math.atan2(dy, dx)
        
        print(f"Attempt {attempts + 1}: Distance to target: {distance:.1f}mm, Angle: {math.degrees(theta):.1f}°")
        
        # Turn to face target
        robot.turn_in_place(radians(theta)).wait_for_completed()
        time.sleep(0.2)
        
        # Move in steps with wall detection
        move_distance = min(DISTANCE_PER_MOVE, distance)
        move_duration = move_distance / WHEEL_SPEED
        
        # Check for wall while moving
        wall_hit = check_for_wall(robot, move_duration)
        
        if wall_hit:
            print("Wall encountered, attempting to go around...")
            
            # Try turning 90 degrees right first
            robot.turn_in_place(degrees(90)).wait_for_completed()
            time.sleep(0.2)
            
            # Try a short move
            test_duration = DISTANCE_PER_MOVE / WHEEL_SPEED
            wall_hit_again = check_for_wall(robot, test_duration)
            
            if wall_hit_again:
                # Right didn't work, try left instead
                print("Right blocked, trying left...")
                robot.turn_in_place(degrees(-180)).wait_for_completed()  # Turn 180 left to face 90 left from original
                time.sleep(0.2)
                
                wall_hit_third = check_for_wall(robot, test_duration)
                
                if wall_hit_third:
                    # Both sides blocked, turn around
                    print("Both sides blocked, turning 90 degrees...")
                    robot.turn_in_place(degrees(90)).wait_for_completed()
                    time.sleep(0.2)
            
            # Update approximate current position (rough estimate)
            x_current += DISTANCE_PER_MOVE * math.cos(theta)
            y_current += DISTANCE_PER_MOVE * math.sin(theta)
        else:
            # Successfully moved forward
            x_current += move_distance * math.cos(theta)
            y_current += move_distance * math.sin(theta)
        
        attempts += 1
        time.sleep(0.3)
    
    print(f"Could not reach target after {max_attempts} attempts")
    return False

def navigate_rectilinear_with_avoidance(robot, x_target, y_target, x_current=0, y_current=0):
    """
    Navigate using rectilinear path with obstacle avoidance.
    """
    # Move along X axis first
    print(f"Moving along X axis: {x_target - x_current:.1f}mm")
    if abs(x_target - x_current) > 50:
        navigate_diagonal_with_avoidance(robot, x_target, y_current, x_current, y_current)
    
    time.sleep(0.5)
    
    # Then move along Y axis
    print(f"Moving along Y axis: {y_target - y_current:.1f}mm")
    if abs(y_target - y_current) > 50:
        navigate_diagonal_with_avoidance(robot, x_target, y_target, x_target, y_current)
    
    print("Rectilinear navigation complete!")

def navigate_to_cube(robot):
    """
    Navigate to a detected cube.
    """
    print("\nSearching for cube...")
    robot.set_head_angle(degrees(0)).wait_for_completed()
    cube = None
    look_around = robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
    try:
        cube = robot.world.wait_for_observed_light_cube(timeout=30)
        print(f"Found cube: {cube}")
    except:
        print("No cube found within timeout")
    finally:
        look_around.stop()
    
    if cube:
        cube_pose = cube.pose
        dx = cube_pose.position.x
        dy = cube_pose.position.y
        print(f"Navigating to cube at ({dx:.2f}, {dy:.2f})")
        navigate_diagonal_with_avoidance(robot, dx, dy)
    else:
        print("Cannot navigate - no cube detected!")

def display_menu():
    """Display the navigation menu."""
    print("\n" + "="*50)
    print("COZMO NAVIGATION WITH OBSTACLE AVOIDANCE")
    print("="*50)
    print("1. Diagonal Navigation (direct path)")
    print("2. Rectilinear Navigation (X then Y)")
    print("3. Navigate to Cube")
    print("4. Calibrate Robot")
    print("5. Exit")
    print("="*50)

def get_coordinates():
    """Get target coordinates from user."""
    try:
        x = float(input("Enter X coordinate (mm): "))
        y = float(input("Enter Y coordinate (mm): "))
        return x, y
    except ValueError:
        print("Invalid input! Please enter numbers.")
        return None, None

def get_current_position():
    """Get current position from user."""
    use_current = input("Use current position as (0, 0)? (y/n): ").lower()
    if use_current == 'y':
        return 0, 0
    else:
        try:
            x = float(input("Enter current X coordinate (mm): "))
            y = float(input("Enter current Y coordinate (mm): "))
            return x, y
        except ValueError:
            print("Invalid input! Using (0, 0) as default.")
            return 0, 0

def cozmo_program(robot: cozmo.robot.Robot):
    # Enable camera for wall detection
    robot.camera.image_stream_enabled = True
    time.sleep(0.5)  # Give camera time to initialize
    
    print("\nCamera initialized. Robot ready!")
    
    while True:
        display_menu()
        
        try:
            choice = input("\nEnter your choice (1-5): ")
            
            if choice == '1':
                # Diagonal Navigation
                print("\n--- DIAGONAL NAVIGATION ---")
                x_current, y_current = get_current_position()
                x_target, y_target = get_coordinates()
                
                if x_target is not None and y_target is not None:
                    print(f"\nStarting diagonal navigation to ({x_target}, {y_target})...")
                    navigate_diagonal_with_avoidance(robot, x_target, y_target, x_current, y_current)
                    print("Navigation complete!")
            
            elif choice == '2':
                # Rectilinear Navigation
                print("\n--- RECTILINEAR NAVIGATION ---")
                x_current, y_current = get_current_position()
                x_target, y_target = get_coordinates()
                
                if x_target is not None and y_target is not None:
                    print(f"\nStarting rectilinear navigation to ({x_target}, {y_target})...")
                    navigate_rectilinear_with_avoidance(robot, x_target, y_target, x_current, y_current)
                    print("Navigation complete!")
            
            elif choice == '3':
                # Navigate to Cube
                print("\n--- NAVIGATE TO CUBE ---")
                navigate_to_cube(robot)
            
            elif choice == '4':
                # Calibrate
                print("\n--- CALIBRATION ---")
                confirm = input("Start calibration? Robot will move forward. (y/n): ")
                if confirm.lower() == 'y':
                    calibrate(robot)
            
            elif choice == '5':
                # Exit
                print("\nExiting navigation program. Goodbye!")
                break
            
            else:
                print("Invalid choice! Please enter a number between 1 and 5.")
        
        except KeyboardInterrupt:
            print("\n\nProgram interrupted by user.")
            break
        except Exception as e:
            print(f"\nError occurred: {e}")
            print("Returning to menu...")
    
    # Clean up
    robot.drive_wheels(0, 0)
    print("Robot stopped.")

if __name__ == '__main__':
    cozmo.run_program(cozmo_program)
```

**Key changes:**

1. **Interactive Menu System**: 
   - Choice 1: Diagonal navigation with custom coordinates
   - Choice 2: Rectilinear navigation (X then Y) with custom coordinates
   - Choice 3: Navigate to a detected cube
   - Choice 4: Run calibration
   - Choice 5: Exit program

2. **User Input Functions**:
   - `get_coordinates()`: Get target X, Y from user
   - `get_current_position()`: Option to use (0,0) or specify current position

3. **90-Degree Turns**: Changed all avoidance turns to 90 degrees:
   - First tries 90° right
   - Then tries 180° turn (to face 90° left)
   - If both blocked, turns another 90°

4. **Loop Structure**: Program keeps running until user chooses to exit

**Usage example:**
```
COZMO NAVIGATION WITH OBSTACLE AVOIDANCE
==================================================
1. Diagonal Navigation (direct path)
2. Rectilinear Navigation (X then Y)
3. Navigate to Cube
4. Calibrate Robot
5. Exit
==================================================

Enter your choice (1-5): 1

--- DIAGONAL NAVIGATION ---
Use current position as (0, 0)? (y/n): y
Enter X coordinate (mm): 300
Enter Y coordinate (mm): 200

Starting diagonal navigation to (300.0, 200.0)...
