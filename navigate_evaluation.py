import cozmo
from cozmo.util import degrees, radians
import math
import time
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

WHEEL_SPEED = 250
DISTANCE_PER_MOVE = 100
CALIBRATION_TIME = 1.0
IMAGE_DIFF_THRESHOLD = 8
CALIBRATED_CONSTANT = 2.25

# Global tracking for visualization
navigation_data = {
    'path': [],  # List of (x, y) positions
    'start': (0, 0),
    'target': (0, 0),
    'walls_detected': [],  # List of (x, y) where walls were detected
    'success': False,
    'distance_error': 0,
    'time_taken': 0,
    'attempts': 0
}

def reset_navigation_data(x_start, y_start, x_target, y_target):
    """Reset tracking data for new navigation"""
    global navigation_data
    navigation_data = {
        'path': [(x_start, y_start)],
        'start': (x_start, y_start),
        'target': (x_target, y_target),
        'walls_detected': [],
        'success': False,
        'distance_error': 0,
        'time_taken': 0,
        'attempts': 0
    }

def add_position(x, y):
    """Add current position to path"""
    navigation_data['path'].append((x, y))

def add_wall_detection(x, y):
    """Mark where a wall was detected"""
    navigation_data['walls_detected'].append((x, y))

def plot_navigation_results():
    """Create visualization of navigation results"""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    
    # Extract path data
    if len(navigation_data['path']) > 0:
        path_x = [p[0] for p in navigation_data['path']]
        path_y = [p[1] for p in navigation_data['path']]
    else:
        path_x, path_y = [], []
    
    start_x, start_y = navigation_data['start']
    target_x, target_y = navigation_data['target']
    
    # Plot 1: Path Visualization
    ax1.plot(path_x, path_y, 'b-o', linewidth=2, markersize=4, label='Actual Path', alpha=0.7)
    ax1.plot([start_x, target_x], [start_y, target_y], 'g--', linewidth=2, label='Direct Path', alpha=0.5)
    ax1.plot(start_x, start_y, 'go', markersize=15, label='Start', markeredgecolor='darkgreen', markeredgewidth=2)
    ax1.plot(target_x, target_y, 'r^', markersize=15, label='Target', markeredgecolor='darkred', markeredgewidth=2)
    
    if len(navigation_data['path']) > 0:
        final_x, final_y = navigation_data['path'][-1]
        ax1.plot(final_x, final_y, 'bs', markersize=12, label='Final Position', markeredgecolor='darkblue', markeredgewidth=2)
    
    # Mark wall detections
    if navigation_data['walls_detected']:
        wall_x = [w[0] for w in navigation_data['walls_detected']]
        wall_y = [w[1] for w in navigation_data['walls_detected']]
        ax1.plot(wall_x, wall_y, 'rx', markersize=12, markeredgewidth=3, label='Wall Detected')
    
    ax1.set_xlabel('X Position (mm)', fontsize=12)
    ax1.set_ylabel('Y Position (mm)', fontsize=12)
    ax1.set_title('Navigation Path', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='best')
    ax1.axis('equal')
    
    # Plot 2: Performance Metrics
    metrics_text = []
    
    # Calculate metrics
    target_distance = math.hypot(target_x - start_x, target_y - start_y)
    
    if len(navigation_data['path']) > 0:
        final_x, final_y = navigation_data['path'][-1]
        actual_distance = math.hypot(final_x - start_x, final_y - start_y)
        distance_error = math.hypot(target_x - final_x, target_y - final_y)
        
        # Calculate path length
        path_length = 0
        for i in range(1, len(navigation_data['path'])):
            dx = navigation_data['path'][i][0] - navigation_data['path'][i-1][0]
            dy = navigation_data['path'][i][1] - navigation_data['path'][i-1][1]
            path_length += math.hypot(dx, dy)
        
        efficiency = (target_distance / path_length * 100) if path_length > 0 else 0
        
        metrics_text = [
            f"Target Distance: {target_distance:.1f} mm",
            f"Path Length: {path_length:.1f} mm",
            f"Final Distance Error: {distance_error:.1f} mm",
            f"Path Efficiency: {efficiency:.1f}%",
            f"Attempts: {navigation_data['attempts']}",
            f"Time Taken: {navigation_data['time_taken']:.2f} s",
            f"Walls Detected: {len(navigation_data['walls_detected'])}",
            f"Success: {'YES' if navigation_data['success'] else 'NO'}",
            f"",
            f"Start: ({start_x:.0f}, {start_y:.0f})",
            f"Target: ({target_x:.0f}, {target_y:.0f})",
            f"Final: ({final_x:.0f}, {final_y:.0f})"
        ]
    else:
        metrics_text = ["No navigation data recorded"]
    
    ax2.axis('off')
    ax2.text(0.1, 0.9, 'Performance Metrics', fontsize=16, fontweight='bold', 
             transform=ax2.transAxes, verticalalignment='top')
    
    y_pos = 0.8
    for line in metrics_text:
        ax2.text(0.1, y_pos, line, fontsize=11, transform=ax2.transAxes, 
                verticalalignment='top', family='monospace')
        y_pos -= 0.06
    
    plt.tight_layout()
    
    # Save the figure
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"cozmo_navigation_{timestamp}.png"
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"\nVisualization saved as: {filename}")
    
    plt.show()

def save_navigation_data_csv():
    """Save navigation data to CSV for further analysis"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"cozmo_navigation_data_{timestamp}.csv"
    
    with open(filename, 'w') as f:
        f.write("Step,X,Y,Event\n")
        for i, (x, y) in enumerate(navigation_data['path']):
            event = "Start" if i == 0 else "Move"
            if i == len(navigation_data['path']) - 1:
                event = "End"
            f.write(f"{i},{x:.2f},{y:.2f},{event}\n")
        
        for i, (x, y) in enumerate(navigation_data['walls_detected']):
            f.write(f"-,{x:.2f},{y:.2f},Wall\n")
    
    print(f"Data saved as: {filename}")

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
    
    # Initialize tracking
    reset_navigation_data(x_current, y_current, x_target, y_target)
    start_time = time.time()
    
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
            navigation_data['success'] = True
            navigation_data['distance_error'] = distance
            navigation_data['time_taken'] = time.time() - start_time
            navigation_data['attempts'] = attempts
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
    navigation_data['success'] = False
    navigation_data['distance_error'] = distance
    navigation_data['time_taken'] = time.time() - start_time
    navigation_data['attempts'] = attempts
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
    # Initialize tracking
    reset_navigation_data(x_current, y_current, x_target, y_target)
    start_time = time.time()
    
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
    
    navigation_data['time_taken'] = time.time() - start_time
    # Check if we reached target
    if len(navigation_data['path']) > 0:
        final_x, final_y = navigation_data['path'][-1]
        distance_error = math.hypot(x_target - final_x, y_target - final_y)
        navigation_data['distance_error'] = distance_error
        navigation_data['success'] = distance_error < 50

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
    print("COZMO NAVIGATION WITH EVALUATION")
    print("="*50)
    print("1. Diagonal Navigation")
    print("2. Rectilinear Navigation")
    print("3. Navigate to Cube")
    print("4. Calibrate")
    print("5. Show Last Navigation Graph")
    print("6. Save Last Navigation Data (CSV)")
    print("7. Exit")
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
        choice = input("\nChoice (1-7): ")
        
        try:
            if choice == '1':
                print("\nDIAGONAL NAVIGATION")
                x_curr, y_curr = get_current_pos()
                x_targ, y_targ = get_coords()
                
                if x_targ is not None:
                    navigate_with_avoidance(robot, x_targ, y_targ, x_curr, y_curr)
                    print("\nGenerating visualization...")
                    plot_navigation_results()
            
            elif choice == '2':
                print("\nRECTILINEAR NAVIGATION")
                x_curr, y_curr = get_current_pos()
                x_targ, y_targ = get_coords()
                
                if x_targ is not None:
                    navigate_rectilinear(robot, x_targ, y_targ, x_curr, y_curr)
                    print("\nGenerating visualization...")
                    plot_navigation_results()
            
            elif choice == '3':
                print("\nNAVIGATE TO CUBE")
                navigate_to_cube(robot)
                print("\nGenerating visualization...")
                plot_navigation_results()
            
            elif choice == '4':
                print("\nCALIBRATION")
                if input("Start? (y/n): ").lower() == 'y':
                    calibrate(robot)
            
            elif choice == '5':
                print("\nSHOWING LAST NAVIGATION GRAPH")
                if len(navigation_data['path']) > 0:
                    plot_navigation_results()
                else:
                    print("No navigation data available. Run a navigation task first.")
            
            elif choice == '6':
                print("\nSAVING NAVIGATION DATA")
                if len(navigation_data['path']) > 0:
                    save_navigation_data_csv()
                else:
                    print("No navigation data available. Run a navigation task first.")
            
            elif choice == '7':
                print("\nClosing")
                break
            
            else:
                print("Invalid choice")
        
        except KeyboardInterrupt:
            print("\nInterrupted")
            break
        except Exception as e:
            print(f"Error: {e}")
            import traceback
            traceback.print_exc()
    
    robot.drive_wheels(0, 0)

if __name__ == '__main__':
    cozmo.run_program(cozmo_program)
