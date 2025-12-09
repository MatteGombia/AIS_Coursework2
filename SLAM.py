import cozmo
import math
import time
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from cozmo.util import degrees, Angle, Pose
from cozmo.objects import CustomObject, CustomObjectMarkers, CustomObjectTypes

#Configuration
WALL_POSITIONS = {
    'wall1': (0, 122),
    'wall2': (122, 122)
}

MARKER_TO_WALL = {
    'CustomType01': 'wall1',
    'CustomType02': 'wall2'
}

MOVE_DISTANCE = 400
WALL_RADIUS = 100
POSITION_TOLERANCE = 50

#Global tracking variables
walls = []
marked_walls_seen = []
walls_angles = []
wall_assignments = {}
explored_positions = []
current_position = (0, 0)
current_heading = 0
path_history = []
cube_positions = []


def create_cozmo_walls(robot: cozmo.robot.Robot):
    #Creates custom wall objects that Cozmo can detect
    types = [CustomObjectTypes.CustomType01,
             CustomObjectTypes.CustomType02,
             CustomObjectTypes.CustomType03,
             CustomObjectTypes.CustomType04,
             CustomObjectTypes.CustomType05,
             CustomObjectTypes.CustomType06,
             CustomObjectTypes.CustomType07,
             CustomObjectTypes.CustomType08,
             CustomObjectTypes.CustomType09,
             CustomObjectTypes.CustomType10,
             CustomObjectTypes.CustomType11,
             CustomObjectTypes.CustomType12,
             CustomObjectTypes.CustomType13,
             CustomObjectTypes.CustomType14,
             CustomObjectTypes.CustomType15,
             CustomObjectTypes.CustomType16]
    
    markers = [CustomObjectMarkers.Circles2,
             CustomObjectMarkers.Diamonds2,
             CustomObjectMarkers.Hexagons2,
             CustomObjectMarkers.Triangles2,
             CustomObjectMarkers.Circles3,
             CustomObjectMarkers.Diamonds3,
             CustomObjectMarkers.Hexagons3,
             CustomObjectMarkers.Triangles3,
             CustomObjectMarkers.Circles4,
             CustomObjectMarkers.Diamonds4,
             CustomObjectMarkers.Hexagons4,
             CustomObjectMarkers.Triangles4,
             CustomObjectMarkers.Circles5,
             CustomObjectMarkers.Diamonds5,
             CustomObjectMarkers.Hexagons5,
             CustomObjectMarkers.Triangles5]
    
    cozmo_walls = []
    
    print("Creating custom walls")
    
    for i in range(0, 8):
        wall = robot.world.define_custom_wall(types[i], markers[i], 200, 60, 50, 50, True)
        cozmo_walls.append(wall)
    
    for i in range(8, 16):
        wall = robot.world.define_custom_wall(types[i], markers[i], 300, 60, 50, 50, True)
        cozmo_walls.append(wall)
    
    return cozmo_walls


def add_wall(wall_x, wall_y, wall_angle):
    #Adds a wall to the map
    walls.append((wall_x, wall_y))
    walls_angles.append(wall_angle)
    print(f"Added wall at ({wall_x:.0f}, {wall_y:.0f})")


def handle_object_observed(evt, **kw):
    #Gets called when Cozmo sees a wall marker
    global walls, marked_walls_seen, walls_angles, wall_assignments
    
    if isinstance(evt.obj, CustomObject):
        if evt.obj not in marked_walls_seen:
            full_type = str(evt.obj.object_type)
            marker_type = full_type.split('.')[-1]
            
            if marker_type in MARKER_TO_WALL:
                #This is a known wall marker
                wall_key = MARKER_TO_WALL[marker_type]
                fixed_position = WALL_POSITIONS[wall_key]
                
                marked_walls_seen.append(evt.obj)
                wall_assignments[evt.obj] = (wall_key, fixed_position)
                
                wall_angle = evt.obj.pose.rotation.angle_z.radians + math.pi/2
                add_wall(fixed_position[0], fixed_position[1], wall_angle)
                
                print(f"Found known wall: {marker_type} at {fixed_position}")
            else:
                #Unknown marker, add it as a new wall
                wall_x = evt.obj.pose.position.x
                wall_y = evt.obj.pose.position.y
                wall_angle = evt.obj.pose.rotation.angle_z.radians + math.pi/2
                
                marked_walls_seen.append(evt.obj)
                add_wall(wall_x, wall_y, wall_angle)
                
                print(f"Found unknown wall at ({wall_x:.0f}, {wall_y:.0f})")


def fix_angle(angle):
    #Keeps angle between -pi and pi
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def find_robot_position(c1, c2, d1, d2):
    #Finds robot position using two wall distances (trilateration)
    x1, y1 = c1
    x2, y2 = c2

    dx = x2 - x1
    dy = y2 - y1
    wall_distance = math.sqrt(dx*dx + dy*dy)

    if wall_distance == 0:
        return None, None
    
    #Checks if circles can intersect
    if wall_distance > d1 + d2 or wall_distance < abs(d1 - d2):
        return None, None

    a = (d1*d1 - d2*d2 + wall_distance*wall_distance) / (2 * wall_distance)
    h_squared = d1*d1 - a*a
    if h_squared < 0:
        return None, None
    h = math.sqrt(h_squared)

    x3 = x1 + a * dx / wall_distance
    y3 = y1 + a * dy / wall_distance

    #Two possible positions
    rx1 = x3 + h * (dy / wall_distance)
    ry1 = y3 - h * (dx / wall_distance)

    rx2 = x3 - h * (dy / wall_distance)
    ry2 = y3 + h * (dx / wall_distance)

    return (rx1, ry1), (rx2, ry2)


def pick_right_position(c1, c2, posA, posB, bearing1, bearing2):
    #Checks which position matches the bearings better
    xr, yr = posA
    angle1 = math.atan2(c1[1] - yr, c1[0] - xr)
    angle2 = math.atan2(c2[1] - yr, c2[0] - xr)
    Error1 = abs(fix_angle(angle1 - bearing1)) + abs(fix_angle(angle2 - bearing2))

    xr, yr = posB
    angle1 = math.atan2(c1[1] - yr, c1[0] - xr)
    angle2 = math.atan2(c2[1] - yr, c2[0] - xr)
    Error2 = abs(fix_angle(angle1 - bearing1)) + abs(fix_angle(angle2 - bearing2))

    if Error1 < Error2:
        return posA
    else:
        return posB


def figure_out_position(c1, c2, d1, d2, b1, b2):
    #Calculates robot position from two wall measurements
    x1, y1 = c1
    x2, y2 = c2
    wall_dist = math.sqrt((x2-x1)**2 + (y2-y1)**2)
    
    print(f"  Walls are {wall_dist:.0f}mm apart")
    print(f"  Measured {d1:.0f}mm and {d2:.0f}mm from robot")
    
    #Tries normal calculation first
    result = find_robot_position(c1, c2, d1, d2)
    if result[0] is not None:
        posA, posB = result
        xr, yr = pick_right_position(c1, c2, posA, posB, b1, b2)
        heading = (b1 + b2) / 2.0
        return xr, yr, heading
    
    #If that didn't work, adjust the distances
    if d1 + d2 < wall_dist:
        scale = (wall_dist * 1.1) / (d1 + d2)
        d1 = d1 * scale
        d2 = d2 * scale
        
        result = find_robot_position(c1, c2, d1, d2)
        if result[0] is not None:
            posA, posB = result
            xr, yr = pick_right_position(c1, c2, posA, posB, b1, b2)
            heading = (b1 + b2) / 2.0
            return xr, yr, heading
    
    elif abs(d1 - d2) > wall_dist:
        if d1 > d2:
            d1 = wall_dist * 0.9 + d2
        else:
            d2 = wall_dist * 0.9 + d1
        
        result = find_robot_position(c1, c2, d1, d2)
        if result[0] is not None:
            posA, posB = result
            xr, yr = pick_right_position(c1, c2, posA, posB, b1, b2)
            heading = (b1 + b2) / 2.0
            return xr, yr, heading
    
    #Estimates from bearings if all else fails
    heading = (b1 + b2) / 2.0
    
    x1_est = c1[0] - d1 * math.cos(b1)
    y1_est = c1[1] - d1 * math.sin(b1)
    
    x2_est = c2[0] - d2 * math.cos(b2)
    y2_est = c2[1] - d2 * math.sin(b2)
    
    xr = (x1_est + x2_est) / 2
    yr = (y1_est + y2_est) / 2
    
    return xr, yr, heading


def get_marker_measurements(robot, marker_obj):
    #Gets distance and bearing to a wall marker
    dx = marker_obj.pose.position.x - robot.pose.position.x
    dy = marker_obj.pose.position.y - robot.pose.position.y

    dist = math.sqrt(dx*dx + dy*dy)
    global_bearing = math.atan2(dy, dx)
    robot_heading = robot.pose.rotation.angle_z.radians

    rel_bearing = fix_angle(global_bearing - robot_heading)
    return dist, rel_bearing


def clean_up_data(data):
    #Removes outliers from measurements
    if len(data) < 4:
        return data
    
    sorted_data = sorted(data)
    n = len(sorted_data)

    #Gets quartiles
    q1 = sorted_data[n // 4]
    q3 = sorted_data[3 * n // 4]
    iqr = q3 - q1

    #Removes outliers
    lower = q1 - 1.5 * iqr
    upper = q3 + 1.5 * iqr
    filtered = [x for x in data if lower <= x <= upper]

    #Doesn't filter too much
    if len(filtered) < len(data) * 0.5:
        return data
    
    return filtered if len(filtered) > 0 else data


def measure_wall(robot, marker_obj, num_samples=10):
    #Takes multiple measurements to a wall and averages them
    distances = []
    bearings = []
    
    print(f"    Taking {num_samples} measurements")
    
    for i in range(num_samples):
        d, b = get_marker_measurements(robot, marker_obj)
        distances.append(d)
        bearings.append(b)
        time.sleep(0.05)
    
    #Cleans up distance outliers
    distances = clean_up_data(distances)
    
    #Gets the average of the bearings using trig to handle the wraparound
    avg_x = sum(math.cos(b) for b in bearings) / len(bearings)
    avg_y = sum(math.sin(b) for b in bearings) / len(bearings)
    avg_bearing = math.atan2(avg_y, avg_x)
    
    #Removes bearing outliers
    diffs = [abs(fix_angle(b - avg_bearing)) for b in bearings]
    threshold = max(sorted(diffs)[len(diffs)//2] * 2.5, math.radians(10))
    good_bearings = [b for b, diff in zip(bearings, diffs) if diff <= threshold]
    
    #Gets median distance
    final_dist = sorted(distances)[len(distances) // 2]
    
    #Average of the good bearings
    final_x = sum(math.cos(b) for b in good_bearings) / len(good_bearings)
    final_y = sum(math.sin(b) for b in good_bearings) / len(good_bearings)
    final_bearing = math.atan2(final_y, final_x)
    
    print(f"    Distance: {final_dist:.0f}mm, Bearing: {math.degrees(final_bearing):.0f}°")
    
    return final_dist, final_bearing


def scan_for_walls(robot: cozmo.robot.Robot, num_needed=2):
    #Does a 360 scan looking for walls
    print("\nScanning for walls")
    
    initial_walls = len(marked_walls_seen)
    
    for step in range(18):
        if len(marked_walls_seen) >= num_needed:
            print(f"Found {len(marked_walls_seen)} walls, stopping scan")
            break
        
        print(f"  Step {step+1}/18 - found {len(marked_walls_seen)} so far")
        
        if step < 17:
            robot.turn_in_place(degrees(20)).wait_for_completed()
            time.sleep(0.3)
    
    new_walls = len(marked_walls_seen) - initial_walls
    print(f"Scan complete - found {new_walls} new walls\n")
    
    return len(marked_walls_seen) >= num_needed


def scan_for_cubes(robot: cozmo.robot.Robot):
    #Does a 360 scan looking for cubes
    print("\nScanning for cubes")
    
    cubes_found = []
    
    for step in range(18):
        #Checks for visible cubes
        cube1 = robot.world.get_light_cube(cozmo.objects.LightCube1Id)
        cube2 = robot.world.get_light_cube(cozmo.objects.LightCube2Id)
        cube3 = robot.world.get_light_cube(cozmo.objects.LightCube3Id)
        
        for cube in [cube1, cube2, cube3]:
            if cube is not None and cube.is_visible:
                cube_pos = (cube.pose.position.x, cube.pose.position.y)
                
                #Checks if we already found this cube
                already_found = False
                for found_pos in cube_positions:
                    dist = math.sqrt((cube_pos[0]-found_pos[0])**2 + (cube_pos[1]-found_pos[1])**2)
                    if dist < 50:
                        already_found = True
                        break
                
                if not already_found:
                    cube_positions.append(cube_pos)
                    cubes_found.append(cube_pos)
                    print(f"  Found cube at ({cube_pos[0]:.0f}, {cube_pos[1]:.0f})")
        
        if step < 17:
            robot.turn_in_place(degrees(20)).wait_for_completed()
            time.sleep(0.2)
    
    print(f"Scan complete - found {len(cubes_found)} new cubes\n")
    return cubes_found


def localise_robot(robot):
    #Figures out where the robot is using wall measurements
    global current_position, current_heading
    
    if len(marked_walls_seen) < 2:
        print("Not enough walls to localise")
        return False
    
    print("\nLocalizing robot")
    
    marker1 = marked_walls_seen[0]
    marker2 = marked_walls_seen[1]
    c1 = walls[0]
    c2 = walls[1]
    
    print(f"Measuring wall 1:")
    d1, b1 = measure_wall(robot, marker1)
    
    print(f"Measuring wall 2:")
    d2, b2 = measure_wall(robot, marker2)
    
    xr, yr, heading = figure_out_position(c1, c2, d1, d2, b1, b2)
    
    current_position = (xr, yr)
    current_heading = heading
    
    print(f"\nRobot position: ({xr:.0f}, {yr:.0f})mm")
    print(f"Robot heading: {math.degrees(heading):.0f}°")
    
    return True


def is_position_explored(x, y):
    #Checks if we've already been near this position
    for pos in explored_positions:
        dist = math.sqrt((x - pos[0])**2 + (y - pos[1])**2)
        if dist < POSITION_TOLERANCE:
            return True
    return False


def is_path_blocked(start_x, start_y, end_x, end_y):
    #Checks if there's a wall blocking the path
    for i in range(len(walls)):
        wall_x, wall_y = walls[i]
        
        #Calculates distance from wall to the line segment
        dx = end_x - start_x
        dy = end_y - start_y
        
        if dx == 0 and dy == 0:
            continue
        
        t = ((wall_x - start_x) * dx + (wall_y - start_y) * dy) / (dx*dx + dy*dy)
        t = max(0, min(1, t))
        
        closest_x = start_x + t * dx
        closest_y = start_y + t * dy
        
        dist = math.sqrt((wall_x - closest_x)**2 + (wall_y - closest_y)**2)
        
        if dist < WALL_RADIUS:
            return True
    
    return False


def choose_next_position():
    #Picks where to explore next
    global current_position
    
    x, y = current_position
    
    #Possible positions to explore
    candidates = [
        (x + MOVE_DISTANCE, y),
        (x - MOVE_DISTANCE, y),
        (x, y + MOVE_DISTANCE),
        (x, y - MOVE_DISTANCE)
    ]
    
    #Finds unexplored positions that aren't blocked
    for pos in candidates:
        if not is_position_explored(pos[0], pos[1]):
            if not is_path_blocked(x, y, pos[0], pos[1]):
                return pos
    
    #If all nearby positions explored, pick the closest unexplored one
    best_pos = None
    best_dist = float('inf')
    
    for explored in explored_positions:
        for offset in [(MOVE_DISTANCE, 0), (-MOVE_DISTANCE, 0), (0, MOVE_DISTANCE), (0, -MOVE_DISTANCE)]:
            new_pos = (explored[0] + offset[0], explored[1] + offset[1])
            
            if not is_position_explored(new_pos[0], new_pos[1]):
                dist = math.sqrt((new_pos[0] - x)**2 + (new_pos[1] - y)**2)
                if dist < best_dist and not is_path_blocked(x, y, new_pos[0], new_pos[1]):
                    best_dist = dist
                    best_pos = new_pos
    
    return best_pos


def move_to_position(robot, target_x, target_y):
    #Moves robot to target position
    global current_position, current_heading, path_history
    
    print(f"\nMoving to ({target_x:.0f}, {target_y:.0f})")
    
    #Calculates heading to target
    dx = target_x - current_position[0]
    dy = target_y - current_position[1]
    target_heading = math.atan2(dy, dx)
    
    #Turns to face target
    turn_angle = fix_angle(target_heading - current_heading)
    print(f"Turning {math.degrees(turn_angle):.0f}°")
    robot.turn_in_place(degrees(math.degrees(turn_angle))).wait_for_completed()
    time.sleep(0.2)
    
    #Moves forward
    distance = math.sqrt(dx*dx + dy*dy)
    print(f"Moving forward {distance:.0f}mm")
    
    try:
        robot.go_to_pose(Pose(target_x, target_y, 0, angle_z=degrees(0))).wait_for_completed()
        time.sleep(0.5)
        
        #Updates position
        current_position = (target_x, target_y)
        current_heading = target_heading
        path_history.append(current_position)
        
        print(f"Reached ({current_position[0]:.0f}, {current_position[1]:.0f})")
        return True
        
    except Exception as e:
        print(f"Failed to reach target: {e}")
        return False


def plot_map():
    #Draws the map of explored area
    print("\nGenerating map")
    
    fig, ax = plt.subplots(figsize=(12, 10))
    
    #Draws the path
    if len(path_history) > 1:
        path_x = [p[0] for p in path_history]
        path_y = [p[1] for p in path_history]
        ax.plot(path_x, path_y, 'b-', linewidth=2, alpha=0.5, label='Path')
    
    #Draws explored positions
    if len(explored_positions) > 0:
        exp_x = [p[0] for p in explored_positions]
        exp_y = [p[1] for p in explored_positions]
        ax.plot(exp_x, exp_y, 'go', markersize=8, label='Explored positions')
    
    #Draws walls
    for i in range(len(walls)):
        wall_x, wall_y = walls[i]
        angle = walls_angles[i]
        
        #Draws wall as a line
        wall_x1 = wall_x - WALL_RADIUS * math.cos(angle)
        wall_y1 = wall_y - WALL_RADIUS * math.sin(angle)
        wall_x2 = wall_x + WALL_RADIUS * math.cos(angle)
        wall_y2 = wall_y + WALL_RADIUS * math.sin(angle)
        
        ax.plot([wall_x1, wall_x2], [wall_y1, wall_y2], 'r-', linewidth=4, label='Wall' if i == 0 else '')
        ax.plot(wall_x, wall_y, 'rs', markersize=10)
    
    #Draws cubes
    if len(cube_positions) > 0:
        cube_x = [c[0] for c in cube_positions]
        cube_y = [c[1] for c in cube_positions]
        ax.plot(cube_x, cube_y, 'ys', markersize=15, label='Cubes', markeredgecolor='orange', markeredgewidth=2)
    
    #Draws current position
    ax.plot(current_position[0], current_position[1], 'bo', markersize=15, label='Current position', markeredgecolor='darkblue', markeredgewidth=2)
    
    #Draws heading arrow
    arrow_len = 100
    dx = arrow_len * math.cos(current_heading)
    dy = arrow_len * math.sin(current_heading)
    ax.arrow(current_position[0], current_position[1], dx, dy, head_width=30, head_length=40, fc='blue', ec='blue')
    
    ax.set_xlabel('X (mm)', fontsize=12)
    ax.set_ylabel('Y (mm)', fontsize=12)
    ax.set_title('SLAM Map', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    ax.legend(loc='upper right')
    
    plt.tight_layout()
    plt.savefig('slam_map.png', dpi=300)
    print("Map saved as slam_map.png")
    plt.show()


def main(robot: cozmo.robot.Robot):
    global current_position, current_heading, explored_positions
    
    print("Starting SLAM")
    
    robot.camera.image_stream_enabled = True
    create_cozmo_walls(robot)
    robot.add_event_handler(cozmo.objects.EvtObjectObserved, handle_object_observed)
    
    time.sleep(1)
    robot.set_head_angle(Angle(0)).wait_for_completed()
    time.sleep(1)
    
    #Initial scan for walls
    scan_for_walls(robot, num_needed=2)
    
    if len(marked_walls_seen) < 2:
        print("Error: Need at least 2 walls for localization")
        return
    
    #Initial localization
    if not localise_robot(robot):
        print("Error: Could not localise robot")
        return
    
    #Main exploration loop
    max_positions = 10
    positions_explored = 0
    
    try:
        while positions_explored < max_positions:
            print(f"\n{'='*50}")
            print(f"Exploration step {positions_explored + 1}/{max_positions}")
            print(f"{'='*50}")
            
            #Marks current position as explored
            if not is_position_explored(current_position[0], current_position[1]):
                explored_positions.append(current_position)
                print(f"Marked position ({current_position[0]:.0f}, {current_position[1]:.0f}) as explored")
            
            #Scans for new things
            scan_for_walls(robot, num_needed=2)
            scan_for_cubes(robot)
            
            #Re-localises if we found new walls
            if len(marked_walls_seen) >= 2:
                localise_robot(robot)
            
            #Picks next position
            next_pos = choose_next_position()
            
            if next_pos is None:
                print("\nNo more positions to explore")
                break
            
            #Moves to next position
            if move_to_position(robot, next_pos[0], next_pos[1]):
                positions_explored += 1
            else:
                print("Failed to reach position, trying another")
            
            time.sleep(0.5)
        
        print("\n" + "="*50)
        print("Exploration complete")
        print("="*50)
        print(f"Explored {len(explored_positions)} positions")
        print(f"Found {len(walls)} walls")
        print(f"Found {len(cube_positions)} cubes")
        
        plot_map()
        
    except KeyboardInterrupt:
        print("\n\nStopping exploration")
        robot.drive_wheels(0, 0)
        plot_map()


if __name__ == "__main__":
    cozmo.run_program(main)
