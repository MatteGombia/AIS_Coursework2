import cozmo
import math
import time
import numpy as np
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
RANGE_MAP_KEY = 20
RADIUS_CIRCLES = 300

#Global tracking variables
walls = []
wall_uncertainties = []  #Stores uncertainty for each wall position
marked_walls_seen = []
walls_angles = []
wall_assignments = {}
map = dict()  #Stores positions and their reachable neighbours
explored_positions = []
current_position = (0, 0)
current_heading = 0
path_history = []
cube_positions = []
cube_uncertainties = []  #Stores uncertainty for each cube
start_time = time.time()

#Particle filter stuff
particles = None
weights = None
num_particles = 500


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


def init_particles():
    #Makes random particles spread out in the environment
    global particles, weights, num_particles
    
    print(f"Initializing {num_particles} particles")
    
    #Random positions in a reasonable range
    particles = np.random.uniform(
        low=[-100, -100, -np.pi],
        high=[300, 300, np.pi],
        size=(num_particles, 3)
    )
    
    #All particles start with equal weight
    weights = np.ones(num_particles) / num_particles
    
    print("Particles initialized")


def move_particles(delta_x, delta_y, delta_theta):
    #Moves all particles based on robot movement with some noise added
    global particles
    
    #How much noise to add (bigger = more uncertain)
    noise_x = 15  #mm
    noise_y = 15  #mm
    noise_theta = 0.15  #radians
    
    for i in range(num_particles):
        #Add movement plus random noise
        particles[i, 0] += delta_x + np.random.normal(0, noise_x)
        particles[i, 1] += delta_y + np.random.normal(0, noise_y)
        particles[i, 2] += delta_theta + np.random.normal(0, noise_theta)
        
        #Keep angle wrapped
        particles[i, 2] = fix_angle(particles[i, 2])


def update_particle_weights(measurements, wall_positions):
    #Updates how likely each particle is based on what we measured
    global particles, weights
    
    #How much noise we expect in our sensors
    sigma_dist = 25  #mm
    sigma_bearing = 0.2  #radians
    
    for i in range(num_particles):
        x, y, theta = particles[i]
        likelihood = 1.0
        
        #Check how well this particle matches each wall measurement
        for (measured_dist, measured_bearing), (wall_x, wall_y) in zip(measurements, wall_positions):
            #What we'd expect to measure from this particle's position
            dx = wall_x - x
            dy = wall_y - y
            expected_dist = np.sqrt(dx**2 + dy**2)
            expected_bearing = fix_angle(np.arctan2(dy, dx) - theta)
            
            #How far off are we
            dist_error = measured_dist - expected_dist
            bearing_error = fix_angle(measured_bearing - expected_bearing)
            
            #Calculate probability using gaussian
            likelihood *= np.exp(-0.5 * (dist_error / sigma_dist)**2)
            likelihood *= np.exp(-0.5 * (bearing_error / sigma_bearing)**2)
        
        weights[i] = likelihood
    
    #Normalize so weights sum to 1
    total_weight = np.sum(weights)
    if total_weight > 0:
        weights = weights / total_weight
    else:
        #If all weights are zero, reset them
        weights = np.ones(num_particles) / num_particles


def resample_particles():
    #Keeps good particles and drops bad ones
    global particles, weights
    
    #Pick particles based on their weights
    indices = np.random.choice(
        num_particles,
        size=num_particles,
        p=weights
    )
    
    particles = particles[indices]
    
    #Reset weights to equal after resampling
    weights = np.ones(num_particles) / num_particles


def get_particle_estimate():
    #Gets best position guess from all particles
    global particles, weights
    
    #Weighted average of x and y
    x = np.sum(weights * particles[:, 0])
    y = np.sum(weights * particles[:, 1])
    
    #For heading, need circular mean
    cos_sum = np.sum(weights * np.cos(particles[:, 2]))
    sin_sum = np.sum(weights * np.sin(particles[:, 2]))
    theta = np.arctan2(sin_sum, cos_sum)
    
    return x, y, theta


def get_particle_uncertainty():
    #Calculates how uncertain we are about position
    global particles, weights
    
    mean_x, mean_y, mean_theta = get_particle_estimate()
    
    #Calculate variance in x and y
    var_x = np.sum(weights * (particles[:, 0] - mean_x)**2)
    var_y = np.sum(weights * (particles[:, 1] - mean_y)**2)
    
    #Calculate variance in heading (circular)
    angle_diffs = np.array([fix_angle(particles[i, 2] - mean_theta) for i in range(num_particles)])
    var_theta = np.sum(weights * angle_diffs**2)
    
    #Return standard deviations
    return np.sqrt(var_x), np.sqrt(var_y), np.sqrt(var_theta)


def add_wall_with_uncertainty(wall_x, wall_y, wall_angle, uncertainty=30):
    #Adds a wall to the map with uncertainty estimate
    walls.append((wall_x, wall_y))
    walls_angles.append(wall_angle)
    wall_uncertainties.append(uncertainty)
    print(f"Added wall at ({wall_x:.0f}, {wall_y:.0f}) ±{uncertainty}mm")


def update_wall_estimate(wall_index, new_x, new_y, new_uncertainty):
    #Updates wall position estimate by fusing old and new measurements
    old_x, old_y = walls[wall_index]
    old_uncertainty = wall_uncertainties[wall_index]
    
    #Kalman-style update - weight by inverse uncertainty
    weight_old = 1.0 / (old_uncertainty**2)
    weight_new = 1.0 / (new_uncertainty**2)
    total_weight = weight_old + weight_new
    
    fused_x = (old_x * weight_old + new_x * weight_new) / total_weight
    fused_y = (old_y * weight_old + new_y * weight_new) / total_weight
    fused_uncertainty = np.sqrt(1.0 / total_weight)
    
    walls[wall_index] = (fused_x, fused_y)
    wall_uncertainties[wall_index] = fused_uncertainty
    
    print(f"Updated wall {wall_index}: uncertainty {old_uncertainty:.0f} -> {fused_uncertainty:.0f}mm")


def add_cube_with_uncertainty(cube_x, cube_y, uncertainty=40):
    #Adds cube position with uncertainty
    cube_positions.append((cube_x, cube_y))
    cube_uncertainties.append(uncertainty)


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
                add_wall_with_uncertainty(fixed_position[0], fixed_position[1], wall_angle, uncertainty=10)
                
                print(f"Found known wall: {marker_type} at {fixed_position}")
            else:
                #Unknown marker, estimate position from current robot pose
                wall_x = evt.obj.pose.position.x
                wall_y = evt.obj.pose.position.y
                wall_angle = evt.obj.pose.rotation.angle_z.radians + math.pi/2
                
                marked_walls_seen.append(evt.obj)
                
                #Higher uncertainty for walls not at fixed positions
                add_wall_with_uncertainty(wall_x, wall_y, wall_angle, uncertainty=50)
                
                print(f"Found unknown wall at ({wall_x:.0f}, {wall_y:.0f})")


def fix_angle(angle):
    #Keeps angle between -pi and pi
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


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
            
            #Update particles after each rotation
            move_particles(0, 0, math.radians(20))
    
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
                    #Add cube with uncertainty estimate based on distance
                    dist_to_cube = math.sqrt(cube_pos[0]**2 + cube_pos[1]**2)
                    uncertainty = 30 + dist_to_cube * 0.05  #More uncertain for far cubes
                    add_cube_with_uncertainty(cube_pos[0], cube_pos[1], uncertainty)
                    cubes_found.append(cube_pos)
                    print(f"  Found cube at ({cube_pos[0]:.0f}, {cube_pos[1]:.0f}) ±{uncertainty:.0f}mm")
        
        if step < 17:
            robot.turn_in_place(degrees(20)).wait_for_completed()
            time.sleep(0.2)
            
            #Update particles
            move_particles(0, 0, math.radians(20))
    
    print(f"Scan complete - found {len(cubes_found)} new cubes\n")
    return cubes_found


def localise_robot(robot):
    #Figures out where the robot is using particle filter
    global current_position, current_heading
    
    if len(marked_walls_seen) < 2:
        print("Not enough walls to localise")
        return False
    
    print("\nlocalising robot using particle filter")
    
    marker1 = marked_walls_seen[0]
    marker2 = marked_walls_seen[1]
    c1 = walls[0]
    c2 = walls[1]
    
    print(f"Measuring wall 1:")
    d1, b1 = measure_wall(robot, marker1)
    
    print(f"Measuring wall 2:")
    d2, b2 = measure_wall(robot, marker2)
    
    #Update particle filter with measurements
    measurements = [(d1, b1), (d2, b2)]
    wall_positions = [c1, c2]
    
    update_particle_weights(measurements, wall_positions)
    resample_particles()
    
    #Get estimate from particles
    xr, yr, heading = get_particle_estimate()
    std_x, std_y, std_theta = get_particle_uncertainty()
    
    current_position = (xr, yr)
    current_heading = heading
    
    print(f"\nRobot position: ({xr:.0f}, {yr:.0f})mm ±({std_x:.0f}, {std_y:.0f})mm")
    print(f"Robot heading: {math.degrees(heading):.0f}° ±{math.degrees(std_theta):.0f}°")
    
    return True


def is_position_explored(x, y):
    #Checks if we've already been near this position using the map
    for map_key in map.keys():
        if x > map_key[0] - RANGE_MAP_KEY and x < map_key[0] + RANGE_MAP_KEY and y > map_key[1] - RANGE_MAP_KEY and y < map_key[1] + RANGE_MAP_KEY:
            return True, map_key
    return False, None


def is_path_blocked(start_x, start_y, end_x, end_y):
    #Checks if there's a wall blocking the path and returns distance info
    max_distance = 0
    blocked = False
    
    for i in range(len(walls)):
        wall_x, wall_y = walls[i]
        wall_uncertainty = wall_uncertainties[i]
        
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
        
        #Add uncertainty buffer to wall radius
        effective_radius = WALL_RADIUS + wall_uncertainty
        
        #Track maximum safe distance even if path is blocked
        if t > 0:  #Wall is ahead, not behind
            distance_along_path = t * math.sqrt(dx*dx + dy*dy)
            if dist < effective_radius:
                blocked = True
                max_distance = max(max_distance, distance_along_path - effective_radius)
            else:
                max_distance = max(max_distance, distance_along_path)
    
    return blocked, max_distance


def choose_next_position():
    #Picks where to explore next using map structure
    global current_position
    
    x, y = current_position
    
    #Check if current position is already in map
    is_seen, map_key = is_position_explored(x, y)
    
    if is_seen and map_key in map:
        #We have cached reachable positions for this location
        candidates = map[map_key]
    else:
        #Generate fresh candidates
        candidates = [
            (x + MOVE_DISTANCE, y),
            (x - MOVE_DISTANCE, y),
            (x, y + MOVE_DISTANCE),
            (x, y - MOVE_DISTANCE)
        ]
    
    #Track best fallback position (furthest reachable even if blocked)
    best_fallback = None
    max_fallback_dist = 0
    
    #Finds unexplored positions that aren't blocked
    valid_positions = []
    for pos in candidates:
        is_explored, _ = is_position_explored(pos[0], pos[1])
        
        if not is_explored:
            blocked, safe_distance = is_path_blocked(x, y, pos[0], pos[1])
            
            if not blocked:
                valid_positions.append(pos)
            elif safe_distance > max_fallback_dist:
                #Track furthest safe point even on blocked paths
                max_fallback_dist = safe_distance
                direction_x = (pos[0] - x) / MOVE_DISTANCE
                direction_y = (pos[1] - y) / MOVE_DISTANCE
                best_fallback = (x + safe_distance * direction_x, 
                               y + safe_distance * direction_y)
    
    #Return first valid unexplored position
    if valid_positions:
        return valid_positions[0]
    
    #Use fallback if we have one
    if best_fallback and max_fallback_dist > 50:  #Minimum useful distance
        return best_fallback
    
    #If all nearby positions explored, search globally
    best_pos = None
    best_dist = float('inf')
    
    for explored_key in map.keys():
        for offset in [(MOVE_DISTANCE, 0), (-MOVE_DISTANCE, 0), 
                       (0, MOVE_DISTANCE), (0, -MOVE_DISTANCE)]:
            new_pos = (explored_key[0] + offset[0], explored_key[1] + offset[1])
            
            is_explored, _ = is_position_explored(new_pos[0], new_pos[1])
            if not is_explored:
                dist = math.sqrt((new_pos[0] - x)**2 + (new_pos[1] - y)**2)
                blocked, _ = is_path_blocked(x, y, new_pos[0], new_pos[1])
                
                if not blocked and dist < best_dist:
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
    
    #Update particles for rotation
    move_particles(0, 0, turn_angle)
    
    #Moves forward
    distance = math.sqrt(dx*dx + dy*dy)
    print(f"Moving forward {distance:.0f}mm")
    
    try:
        robot.go_to_pose(Pose(target_x, target_y, 0, angle_z=degrees(0))).wait_for_completed()
        time.sleep(0.5)
        
        #Update particles for translation
        move_particles(dx, dy, 0)
        
        #Updates position estimate from particles
        xr, yr, heading = get_particle_estimate()
        current_position = (xr, yr)
        current_heading = heading
        path_history.append(current_position)
        
        print(f"Reached ({current_position[0]:.0f}, {current_position[1]:.0f})")
        return True
        
    except Exception as e:
        print(f"Failed to reach target: {e}")
        return False


def plot_map():
    #Draws the map of explored area with uncertainty visualization
    global particles, weights
    
    print("\nGenerating probabilistic map")
    
    fig, ax = plt.subplots(figsize=(14, 12))
    
    #Draws particles (shows belief distribution)
    if particles is not None:
        top_indices = np.argsort(weights)[-200:]
        top_particles = particles[top_indices]
        top_weights = weights[top_indices]
        
        norm_weights = (top_weights - top_weights.min()) / (top_weights.max() - top_weights.min() + 1e-10)
        
        scatter = ax.scatter(top_particles[:, 0], top_particles[:, 1], 
                           c=norm_weights, cmap='YlOrRd', s=15, alpha=0.4, 
                           label='Belief particles')
    
    #Draws the path
    if len(path_history) > 1:
        path_x = [p[0] for p in path_history]
        path_y = [p[1] for p in path_history]
        ax.plot(path_x, path_y, 'b-', linewidth=2, alpha=0.5, label='Path')
    
    #Draws explored positions with visibility circles
    for pos in map.keys():
        #Visibility circle
        circle = plt.Circle(pos, RADIUS_CIRCLES, color='g', alpha=0.1, linewidth=1, edgecolor='g', linestyle='--')
        ax.add_patch(circle)
        #Position marker
        ax.plot(pos[0], pos[1], 'go', markersize=8)
    
    #Draws walls with uncertainty ellipses
    for i in range(len(walls)):
        wall_x, wall_y = walls[i]
        angle = walls_angles[i]
        uncertainty = wall_uncertainties[i]
        
        #Draws wall as a line
        wall_x1 = wall_x - WALL_RADIUS * math.cos(angle)
        wall_y1 = wall_y - WALL_RADIUS * math.sin(angle)
        wall_x2 = wall_x + WALL_RADIUS * math.cos(angle)
        wall_y2 = wall_y + WALL_RADIUS * math.sin(angle)
        
        ax.plot([wall_x1, wall_x2], [wall_y1, wall_y2], 'r-', linewidth=4, label='Wall' if i == 0 else '')
        ax.plot(wall_x, wall_y, 'rs', markersize=10, markeredgecolor='darkred', markeredgewidth=2)
        
        #Uncertainty circle
        uncertainty_circle = plt.Circle((wall_x, wall_y), uncertainty, 
                                       color='red', alpha=0.15, linestyle='--', fill=True)
        ax.add_patch(uncertainty_circle)
    
    #Draws cubes with uncertainty
    for i, cube_pos in enumerate(cube_positions):
        uncertainty = cube_uncertainties[i]
        
        ax.plot(cube_pos[0], cube_pos[1], 'ys', markersize=15, 
                label='Cubes' if i == 0 else '',
                markeredgecolor='orange', markeredgewidth=2)
        
        #Uncertainty circle
        cube_uncertainty = plt.Circle(cube_pos, uncertainty,
                                     color='yellow', alpha=0.1, linestyle=':', fill=True)
        ax.add_patch(cube_uncertainty)
    
    #Draws current position estimate
    ax.plot(current_position[0], current_position[1], 'bo', markersize=15, 
            label='Current position', markeredgecolor='darkblue', markeredgewidth=2)
    
    #Draws heading arrow
    arrow_len = 100
    dx = arrow_len * math.cos(current_heading)
    dy = arrow_len * math.sin(current_heading)
    ax.arrow(current_position[0], current_position[1], dx, dy, 
             head_width=30, head_length=40, fc='blue', ec='blue')
    
    #Calculate and display statistics
    elapsed = time.time() - start_time
    total_dist = 0
    for i in range(1, len(path_history)):
        dx_path = path_history[i][0] - path_history[i-1][0]
        dy_path = path_history[i][1] - path_history[i-1][1]
        total_dist += math.sqrt(dx_path*dx_path + dy_path*dy_path)
    
    std_x, std_y, std_theta = get_particle_uncertainty()
    
    title = f'Probabilistic SLAM | Time: {elapsed:.0f}s | Positions: {len(map)} | '
    title += f'Walls: {len(walls)} | Cubes: {len(cube_positions)} | '
    title += f'Distance: {total_dist:.0f}mm\n'
    title += f'Position uncertainty: ±{std_x:.0f}mm x ±{std_y:.0f}mm | '
    title += f'Heading uncertainty: ±{math.degrees(std_theta):.1f}°'
    
    ax.set_xlabel('X (mm)', fontsize=12)
    ax.set_ylabel('Y (mm)', fontsize=12)
    ax.set_title(title, fontsize=11, fontweight='bold')
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
    plt.savefig('probabilistic_slam_map.png', dpi=300)
    print("Map saved as probabilistic_slam_map.png")
    
    print(f"\nStatistics:")
    print(f"  Time: {elapsed:.1f}s")
    print(f"  Distance travelled: {total_dist:.0f}mm")
    print(f"  Positions explored: {len(map)}")
    print(f"  Walls found: {len(walls)}")
    print(f"  Cubes found: {len(cube_positions)}")
    print(f"  Position uncertainty: ±{std_x:.0f}mm (x), ±{std_y:.0f}mm (y)")
    print(f"  Heading uncertainty: ±{math.degrees(std_theta):.1f}°")
    
    #Print wall uncertainties
    print(f"\nWall uncertainties:")
    for i, uncertainty in enumerate(wall_uncertainties):
        print(f"  Wall {i}: ±{uncertainty:.1f}mm")
    
    plt.show()


def main(robot: cozmo.robot.Robot):
    global current_position, current_heading, explored_positions
    
    print("Starting Probabilistic SLAM")
    print("===========================\n")
    
    robot.camera.image_stream_enabled = True
    create_cozmo_walls(robot)
    robot.add_event_handler(cozmo.objects.EvtObjectObserved, handle_object_observed)
    
    #Initialize particle filter
    init_particles()
    
    time.sleep(1)
    robot.set_head_angle(Angle(0)).wait_for_completed()
    time.sleep(1)
    
    #Initial scan for walls
    scan_for_walls(robot, num_needed=2)
    
    if len(marked_walls_seen) < 2:
        print("Error: Need at least 2 walls for localisation")
        return
    
    #Initial localisation using particle filter
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
            
            #Check if current position is already in map
            is_seen, map_key = is_position_explored(current_position[0], current_position[1])
            
            if not is_seen:
                #New position - mark as explored and scan
                print(f"New position ({current_position[0]:.0f}, {current_position[1]:.0f})")
                
                #Generate candidate positions for this location
                x, y = current_position
                candidates = [
                    (x + MOVE_DISTANCE, y),
                    (x - MOVE_DISTANCE, y),
                    (x, y + MOVE_DISTANCE),
                    (x, y - MOVE_DISTANCE)
                ]
                
                #Filter candidates by checking for obstacles
                valid_candidates = []
                for pos in candidates:
                    blocked, _ = is_path_blocked(x, y, pos[0], pos[1])
                    if not blocked:
                        valid_candidates.append(pos)
                
                #Store in map
                map[(current_position[0], current_position[1])] = valid_candidates
                positions_explored += 1
                
                #Scans for new things
                scan_for_walls(robot, num_needed=2)
                scan_for_cubes(robot)
                
                #Re-localises if we can (loop closure)
                if len(marked_walls_seen) >= 2:
                    print("\n--- Loop closure: Re-localising ---")
                    localise_robot(robot)
            else:
                #Already been here, use cached candidates
                print(f"Revisiting known position near ({map_key[0]:.0f}, {map_key[1]:.0f})")
            
            #Picks next position
            next_pos = choose_next_position()
            
            if next_pos is None:
                print("\nNo more positions to explore")
                break
            
            #Moves to next position
            if move_to_position(robot, next_pos[0], next_pos[1]):
                path_history.append(current_position)
            else:
                print("Failed to reach position, trying another")
            
            time.sleep(0.5)
        
        print("\n" + "="*50)
        print("Exploration complete")
        print("="*50)
        
        plot_map()
        
    except KeyboardInterrupt:
        print("\n\nStopping exploration")
        robot.drive_wheels(0, 0)
        plot_map()


if __name__ == "__main__":
    cozmo.run_program(main)
