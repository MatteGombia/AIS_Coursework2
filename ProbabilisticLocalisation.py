import cozmo
import math
import time
from statistics import mean
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from cozmo.util import degrees, Angle
from cozmo.objects import CustomObject, CustomObjectMarkers, CustomObjectTypes

WALL_POSITIONS = {
    'wall1': (0, 122),
    'wall2': (122, 122)
}

MARKER_TO_WALL = {
    'CustomType01': 'wall1',
    'CustomType02': 'wall2'
}

walls = []
marked_walls_seen = []
walls_angles = []
wall_assignments = {}

# Track stuff for plotting
scan_positions = []
wall_detections = []
final_position = None
final_heading = None
internal_position = None


def create_cozmo_walls(robot: cozmo.robot.Robot):
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


def add_wall(wall_x, wall_y):
    walls.append((wall_x, wall_y))


def handle_object_observed(evt, **kw):
    global walls, marked_walls_seen, walls_angles, wall_assignments, wall_detections
    
    if isinstance(evt.obj, CustomObject):
        if evt.obj not in marked_walls_seen:
            full_type = str(evt.obj.object_type)
            marker_type = full_type.split('.')[-1]
            
            if marker_type in MARKER_TO_WALL:
                wall_key = MARKER_TO_WALL[marker_type]
                fixed_position = WALL_POSITIONS[wall_key]
                
                marked_walls_seen.append(evt.obj)
                wall_assignments[evt.obj] = (wall_key, fixed_position)
                
                print(f"Found wall: {marker_type} at {fixed_position}")
                add_wall(fixed_position[0], fixed_position[1])
                wall_detections.append(fixed_position)
            else:
                #Assigns it to the next available wall
                if len(marked_walls_seen) < len(WALL_POSITIONS):
                    wall_keys = list(WALL_POSITIONS.keys())
                    wall_key = wall_keys[len(marked_walls_seen)]
                    fixed_position = WALL_POSITIONS[wall_key]
                    
                    marked_walls_seen.append(evt.obj)
                    wall_assignments[evt.obj] = (wall_key, fixed_position)
                    add_wall(fixed_position[0], fixed_position[1])
                    wall_detections.append(fixed_position)


def fix_angle(angle):
    #Keeps angle between -pi and pi
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def find_robot_position(c1, c2, d1, d2):
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
    error1 = abs(fix_angle(angle1 - bearing1)) + abs(fix_angle(angle2 - bearing2))

    xr, yr = posB
    angle1 = math.atan2(c1[1] - yr, c1[0] - xr)
    angle2 = math.atan2(c2[1] - yr, c2[0] - xr)
    error2 = abs(fix_angle(angle1 - bearing1)) + abs(fix_angle(angle2 - bearing2))

    if error1 < error2:
        return posA
    else:
        return posB


def figure_out_position(c1, c2, d1, d2, b1, b2):
    x1, y1 = c1
    x2, y2 = c2
    wall_dist = math.sqrt((x2-x1)**2 + (y2-y1)**2)
    
    print(f"\nTrying to find position")
    print(f"  Walls are {wall_dist:.0f}mm apart")
    print(f"  Measured {d1:.0f}mm and {d2:.0f}mm from robot")
    
    #Tries normal calculation first
    result = find_robot_position(c1, c2, d1, d2)
    if result[0] is not None:
        posA, posB = result
        xr, yr = pick_right_position(c1, c2, posA, posB, b1, b2)
        heading = (b1 + b2) / 2.0
        print(f"  Got position directly")
        return xr, yr, heading
    
    print(f"  Direct calculation didn't work, adjusting")
    
    #If distances are too small, it makes them bigger
    if d1 + d2 < wall_dist:
        scale = (wall_dist * 1.1) / (d1 + d2)
        d1 = d1 * scale
        d2 = d2 * scale
        print(f"  Scaled up distances by {scale:.2f}x")
        
        result = find_robot_position(c1, c2, d1, d2)
        if result[0] is not None:
            posA, posB = result
            xr, yr = pick_right_position(c1, c2, posA, posB, b1, b2)
            heading = (b1 + b2) / 2.0
            return xr, yr, heading
    
    #If one distance is way bigger than the other, it fixes it
    elif abs(d1 - d2) > wall_dist:
        if d1 > d2:
            d1 = wall_dist * 0.9 + d2
        else:
            d2 = wall_dist * 0.9 + d1
        print(f"  Adjusted distance difference")
        
        result = find_robot_position(c1, c2, d1, d2)
        if result[0] is not None:
            posA, posB = result
            xr, yr = pick_right_position(c1, c2, posA, posB, b1, b2)
            heading = (b1 + b2) / 2.0
            return xr, yr, heading
    
    #Estimates from bearings
    print(f"  Using bearing estimate")
    heading = (b1 + b2) / 2.0
    
    x1_est = c1[0] - d1 * math.cos(b1)
    y1_est = c1[1] - d1 * math.sin(b1)
    
    x2_est = c2[0] - d2 * math.cos(b2)
    y2_est = c2[1] - d2 * math.sin(b2)
    
    xr = (x1_est + x2_est) / 2
    yr = (y1_est + y2_est) / 2
    
    return xr, yr, heading


def localise_robot(c1, c2, d1, d2, b1, b2):
    return figure_out_position(c1, c2, d1, d2, b1, b2)


def get_marker_measurements(robot, marker_obj):
    dx = marker_obj.pose.position.x - robot.pose.position.x
    dy = marker_obj.pose.position.y - robot.pose.position.y

    dist = math.sqrt(dx*dx + dy*dy)
    global_bearing = math.atan2(dy, dx)
    robot_heading = robot.pose.rotation.angle_z.radians

    rel_bearing = fix_angle(global_bearing - robot_heading)
    return dist, rel_bearing


def clean_up_data(data):
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
    distances = []
    bearings = []
    
    print(f"    Taking {num_samples} measurements", end='', flush=True)
    
    for i in range(num_samples):
        d, b = get_marker_measurements(robot, marker_obj)
        distances.append(d)
        bearings.append(b)
        time.sleep(0.05)
    
    print("\nDone")
    
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
    
    #Gest median distance
    final_dist = sorted(distances)[len(distances) // 2]
    
    #Average of the good bearings
    final_x = sum(math.cos(b) for b in good_bearings) / len(good_bearings)
    final_y = sum(math.sin(b) for b in good_bearings) / len(good_bearings)
    final_bearing = math.atan2(final_y, final_x)
    
    print(f"    Distance: {final_dist:.0f}mm, Bearing: {math.degrees(final_bearing):.0f}°")
    
    return final_dist, final_bearing


def scan_for_walls(robot: cozmo.robot.Robot, num_needed=2):
    global marked_walls_seen, walls, walls_angles, wall_assignments, scan_positions
    
    marked_walls_seen.clear()
    walls.clear()
    walls_angles.clear()
    wall_assignments.clear()
    
    print("\nScanning for walls")
    
    #Does a 360 scan
    for step in range(18):
        #Records where robot is during scan
        scan_positions.append((
            robot.pose.position.x,
            robot.pose.position.y,
            robot.pose.rotation.angle_z.radians
        ))
        
        if len(marked_walls_seen) >= num_needed:
            print(f"Found {len(marked_walls_seen)} walls, stopping scan")
            break
        
        print(f"  Step {step+1}/18 - found {len(marked_walls_seen)} so far")
        
        if step < 17:
            robot.turn_in_place(degrees(20)).wait_for_completed()
            time.sleep(0.3)
    
    print(f"Scan complete - found {len(marked_walls_seen)} walls\n")
    
    return len(marked_walls_seen) >= num_needed


def plot_everything(c1, c2, d1, d2):
    global scan_positions, wall_detections, final_position, final_heading, internal_position
    
    print("\nMaking plot")
    
    fig, ax = plt.subplots(figsize=(10, 8))
    
    #Draws the walls
    for i, wall_pos in enumerate(WALL_POSITIONS.values()):
        wall_rect = patches.Rectangle((wall_pos[0]-10, wall_pos[1]-5), 20, 10, 
                                      linewidth=2, edgecolor='black', facecolor='gray')
        ax.add_patch(wall_rect)
        ax.text(wall_pos[0], wall_pos[1], f'Wall{i+1}', ha='center', va='center', fontsize=8)
    
    #Shows scan path
    if len(scan_positions) > 0:
        scan_x = [p[0] for p in scan_positions]
        scan_y = [p[1] for p in scan_positions]
        ax.plot(scan_x, scan_y, 'o-', color='lightblue', markersize=3, 
                linewidth=1, label='Scan path', alpha=0.6)
    
    #Shows when walls were found
    for wall_pos in wall_detections:
        ax.plot(wall_pos[0], wall_pos[1], 'r*', markersize=15, label='Wall detected')
    
    #Draws distance circles from walls
    if final_position:
        circle1 = plt.Circle(c1, d1, fill=False, color='orange', linestyle='--', 
                            linewidth=1.5, alpha=0.7)
        circle2 = plt.Circle(c2, d2, fill=False, color='purple', linestyle='--', 
                            linewidth=1.5, alpha=0.7)
        ax.add_patch(circle1)
        ax.add_patch(circle2)
        
        #Shows calculated position
        ax.plot(final_position[0], final_position[1], 'go', markersize=12, 
                label='Calculated position')
        
        #Shows heading
        arrow_len = 30
        dx = arrow_len * math.cos(final_heading)
        dy = arrow_len * math.sin(final_heading)
        ax.arrow(final_position[0], final_position[1], dx, dy, 
                head_width=8, head_length=10, fc='green', ec='green', alpha=0.7)
    
    #Shows cozmo's internal estimate
    if internal_position:
        ax.plot(internal_position[0], internal_position[1], 'bs', markersize=10, 
                label='Cozmo internal')
    
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_title('Robot Localisation Process')
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    #Cleans up duplicate labels
    handles, labels = ax.get_legend_handles_labels()
    unique = {}
    for h, l in zip(handles, labels):
        if l not in unique:
            unique[l] = h
    ax.legend(unique.values(), unique.keys(), loc='upper right')
    
    plt.tight_layout()
    plt.show()
    
    print("Plot shown")


def main(robot: cozmo.robot.Robot):
    global final_position, final_heading, internal_position
    
    print("Robot Localisation Using Wall Markers")
    print(f"\nWall positions:")
    for wall_key, pos in WALL_POSITIONS.items():
        print(f"  {wall_key}: {pos}")
    
    robot.camera.image_stream_enabled = True
    create_cozmo_walls(robot)
    robot.add_event_handler(cozmo.objects.EvtObjectObserved, handle_object_observed)
    
    time.sleep(1)
    robot.set_head_angle(Angle(0)).wait_for_completed()
    time.sleep(1)
    
    #Looks for walls
    found_walls = scan_for_walls(robot, num_needed=2)
    
    if not found_walls or len(marked_walls_seen) < 2:
        print(f"ERROR: Only found {len(marked_walls_seen)} walls, need 2")
        return
    
    print("Measuring walls")
    
    marker1 = marked_walls_seen[0]
    marker2 = marked_walls_seen[1]
    c1 = walls[0]
    c2 = walls[1]
    
    wall1_key, wall1_pos = wall_assignments[marker1]
    wall2_key, wall2_pos = wall_assignments[marker2]
    
    print(f"\nUsing:")
    print(f"  {wall1_key} at {c1}")
    print(f"  {wall2_key} at {c2}")
    
    print(f"\nMeasuring {wall1_key}:")
    d1, b1 = measure_wall(robot, marker1)
    
    print(f"\nMeasuring {wall2_key}:")
    d2, b2 = measure_wall(robot, marker2)
    
    xr, yr, heading = figure_out_position(c1, c2, d1, d2, b1, b2)
    
    print("\nResults:")
    print(f"Robot position: ({xr:.0f}, {yr:.0f})mm")
    print(f"Robot heading: {math.degrees(heading):.0f}°")
    
    print("\nCozmo's internal estimate:")
    print(f"Position: ({robot.pose.position.x:.0f}, {robot.pose.position.y:.0f})mm")
    print(f"Heading: {robot.pose.rotation.angle_z.degrees:.0f}°")
    
    #Calculates error
    error_x = abs(xr - robot.pose.position.x)
    error_y = abs(yr - robot.pose.position.y)
    error_dist = math.sqrt(error_x*error_x + error_y*error_y)
    error_heading = abs(fix_angle(heading - robot.pose.rotation.angle_z.radians))
    
    print(f"\nPosition error: {error_dist:.0f}mm")
    print(f"Heading error: {math.degrees(error_heading):.0f}°\n")
    
    #Saves stuff for plotting
    final_position = (xr, yr)
    final_heading = heading
    internal_position = (robot.pose.position.x, robot.pose.position.y)
    
    #Makes the plot
    plot_everything(c1, c2, d1, d2)


if __name__ == "__main__":
    cozmo.run_program(main)
