import cozmo
from cozmo.util import degrees, radians, Angle
import math
import time
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from frame2d import Frame2D

WHEEL_SPEED = 250
DISTANCE_PER_MOVE = 100
CALIBRATION_TIME = 1.0
IMAGE_DIFF_THRESHOLD = 8
CALIBRATED_CONSTANT = 2.25

WALL_RADIUS=100
WALL_THRESHOLD = 50

# Global tracking for visualisation
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

def normalise_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def plot_navigation_results():
    """Create visualisation of navigation results"""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    
    # Extract path data
    if len(navigation_data['path']) > 0:
        path_x = [p[0] for p in navigation_data['path']]
        path_y = [p[1] for p in navigation_data['path']]
    else:
        path_x, path_y = [], []
    
    start_x, start_y = navigation_data['start']
    target_x, target_y = navigation_data['target']
    
    # Plot 1: Path Visualisation
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
    print(f"\nVisualisation saved as: {filename}")
    
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

        add_wall_detection(get_current_pos())
        
        robot.drive_wheels(-WHEEL_SPEED, -WHEEL_SPEED, duration=move_duration)
        time.sleep(move_duration)
        robot.drive_wheels(0, 0)
        time.sleep(0.2)
        return True
    
    return False

def get_angle_math(x1, y1, x2, y2, x3, y3):
    """
    Calculates the angle in degrees between three points (p1, p2, p3) using the math module.
    The angle is at the vertex p2.
    """

    # Create vectors
    v1_x, v1_y = x1 - x2, y1 - y2
    v2_x, v2_y = x3 - x2, y3 - y2

    # Calculate dot product
    dot_product = v1_x * v2_x + v1_y * v2_y

    # Calculate magnitudes
    norm_v1 = math.sqrt(v1_x**2 + v1_y**2)
    norm_v2 = math.sqrt(v2_x**2 + v2_y**2)

    # Calculate the cosine of the angle
    if norm_v1 == 0 or norm_v2 == 0:
        return 0.0
    cos_theta = dot_product / (norm_v1 * norm_v2)
    cos_theta = max(-1.0, min(1.0, cos_theta)) # Clip for floating point errors

    angle_radians = math.acos(cos_theta)
    return angle_radians

def calculate_next_target(px, py, x1, y1, x2, y2):
    """Return shortest distance from point (px,py) to segment (x1,y1)-(x2,y2).
    All units are assumed to be mm.
    """
    vx = x2 - x1
    vy = y2 - y1
    wx = px - x1
    wy = py - y1
    seg_len_sq = vx*vx + vy*vy
    if seg_len_sq == 0:
        return False, None, None
    t = (wx*vx + wy*vy) / seg_len_sq
    if t <= 0:
        print("Wall before initial point")
        return False, None, None
    elif t >= 1:
        print("Wall after target point")
        return False, None, None
    
    cx = x1 + t * vx
    cy = y1 + t * vy

    if math.hypot(px - cx, py - cy) > WALL_RADIUS:
        print("Wall too far from segment " + str(math.hypot(px - cx, py - cy)))
        return False, None, None
    
    absolute_angle = get_angle_math(0, 1e10, x1, y1, x2, y2)
    print(get_angle_math(0, 1e10, x1, y1, x2, y2))

    sign = 1
    if get_angle_math(x1, y1, x2, y2, px, py) < math.pi:
        sign = -1

    print(get_angle_math(x1, y1, x2, y2, px, py))

    target_x = px + sign * (WALL_RADIUS+WALL_THRESHOLD) * math.sin(absolute_angle+math.pi/2)
    target_y = py + sign * (WALL_RADIUS+WALL_THRESHOLD) * math.cos(absolute_angle+math.pi/2)
    
    return True, target_x, target_y

def navigate_with_avoidance(robot, x_target, y_target, x_current=0, y_current=0):
    tolerance = 50
    max_attempts = 40
    attempts = 0
    
    for wall in navigation_data['walls_detected']: 
        obstructed, new_target_x, new_target_y = calculate_next_target(wall[0], wall[1], x_current, y_current, x_target, y_target)
        if obstructed and math.hypot(new_target_x-x_current, new_target_y-y_target)>10:
            print("Obtacle found at %f, %f", wall[0], wall[1])
            print("New partial target at %f, %f", new_target_x, new_target_y)
            navigate_with_avoidance(robot, new_target_x, new_target_y, x_current, y_current)
            x_current, y_current = new_target_x, new_target_y


    # Initialise tracking
    #reset_navigation_data(x_current, y_current, x_target, y_target)
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

            x_current, y_current = get_current_pos(robot)
            
            #WRONG, If you came back, the wall is not there. Probably to add inside the function and use navigation data[-1]
            #TO DO: Change wall and target 

            #Find new way 
            obstructed, new_target_x, new_target_y = calculate_next_target(navigation_data['walls_detected'][-1][0], navigation_data['walls_detected'][-1][1], x_current, y_current, x_target, y_target)
            if obstructed and math.hypot(new_target_x-x_current, new_target_y-y_target)>10:
                print("Obtacle found at %f, %f", wall[0], wall[1])
                print("New partial target at %f, %f", new_target_x, new_target_y)
                navigate_with_avoidance(robot, new_target_x, new_target_y, x_current, y_current)
                x_current, y_current = new_target_x, new_target_y

            
        else:
            print("No wall detected, moving forward")
            x_current, y_current = get_current_pos(robot)
            #x_current += move_distance * math.cos(robot_heading)
            #y_current += move_distance * math.sin(robot_heading)
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

def get_current_pos(robot: cozmo.robot.Robot):
    robotPose = Frame2D.fromPose(robot.pose)
    return robotPose.x(), robotPose.y()

def cozmo_program(robot: cozmo.robot.Robot):
    robot.camera.image_stream_enabled = True
    time.sleep(0.5)
    print("Robot ready!")
    
    while True:
        print("\nDIAGONAL NAVIGATION")
        x_curr, y_curr = get_current_pos(robot)
        x_targ, y_targ = (400,10)
        reset_navigation_data(x_curr, y_curr, x_targ, y_targ)
        add_wall_detection(200, 0)
        
        if x_targ is not None:
            navigate_with_avoidance(robot, x_targ, y_targ, x_curr, y_curr)
            print("\nGenerating visualisation...")
            plot_navigation_results()
            
           
    
    robot.drive_wheels(0, 0)

if __name__ == '__main__':
    cozmo.run_program(cozmo_program)
