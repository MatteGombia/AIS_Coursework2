import cozmo
import math
import time
import numpy as np
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

#Track stuff for plotting
scan_positions = []
wall_detections = []
final_position = None
final_heading = None
internal_position = None

#Particle filter values
particles = None
weights = None
num_particles = 500


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


def init_particles():
    #Makes random particles spread out in the environment
    global particles, weights, num_particles
    
    print(f"Making {num_particles} particles")
    
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
    
    print(" - Done")
    
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
            
            #Update particles after each rotation
            move_particles(0, 0, math.radians(20))
    
    print(f"Scan complete - found {len(marked_walls_seen)} walls\n")
    
    return len(marked_walls_seen) >= num_needed


def plot_everything(c1, c2, d1, d2):
    global scan_positions, wall_detections, final_position, final_heading, internal_position
    global particles, weights
    
    print("\nMaking plot")
    
    fig, ax = plt.subplots(figsize=(12, 10))
    
    #Draws the walls
    for i, wall_pos in enumerate(WALL_POSITIONS.values()):
        wall_rect = patches.Rectangle((wall_pos[0]-10, wall_pos[1]-5), 20, 10, 
                                      linewidth=2, edgecolor='black', facecolor='gray')
        ax.add_patch(wall_rect)
        ax.text(wall_pos[0], wall_pos[1], f'Wall{i+1}', ha='center', va='center', fontsize=8)
    
    #Shows particles (colored by weight)
    if particles is not None:
        #Only show top particles to keep plot clean
        top_indices = np.argsort(weights)[-100:]
        top_particles = particles[top_indices]
        top_weights = weights[top_indices]
        
        #Normalize weights for coloring
        norm_weights = (top_weights - top_weights.min()) / (top_weights.max() - top_weights.min() + 1e-10)
        
        scatter = ax.scatter(top_particles[:, 0], top_particles[:, 1], 
                           c=norm_weights, cmap='YlOrRd', s=20, alpha=0.6, 
                           label='Particles (top 100)')
        
        #Show particle headings for best ones
        best_indices = np.argsort(weights)[-10:]
        for idx in best_indices:
            x, y, theta = particles[idx]
            dx = 15 * np.cos(theta)
            dy = 15 * np.sin(theta)
            ax.arrow(x, y, dx, dy, head_width=5, head_length=7, 
                    fc='red', ec='red', alpha=0.3, linewidth=0.5)
    
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
                            linewidth=1.5, alpha=0.7, label='Distance circles')
        circle2 = plt.Circle(c2, d2, fill=False, color='purple', linestyle='--', 
                            linewidth=1.5, alpha=0.7)
        ax.add_patch(circle1)
        ax.add_patch(circle2)
        
        #Shows particle filter estimate
        ax.plot(final_position[0], final_position[1], 'go', markersize=15, 
                markeredgewidth=2, markeredgecolor='darkgreen',
                label='Particle filter estimate')
        
        #Shows heading
        arrow_len = 40
        dx = arrow_len * math.cos(final_heading)
        dy = arrow_len * math.sin(final_heading)
        ax.arrow(final_position[0], final_position[1], dx, dy, 
                head_width=10, head_length=12, fc='green', ec='darkgreen', 
                linewidth=2, alpha=0.8)
    
    #Shows cozmo's internal estimate
    if internal_position:
        ax.plot(internal_position[0], internal_position[1], 'bs', markersize=12, 
                markeredgewidth=2, markeredgecolor='darkblue',
                label='Cozmo internal')
    
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_title('Particle Filter Localisation')
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
    
    print("Particle Filter Localisation Using Wall Markers")
    print(f"\nWall positions:")
    for wall_key, pos in WALL_POSITIONS.items():
        print(f"  {wall_key}: {pos}")
    
    robot.camera.image_stream_enabled = True
    create_cozmo_walls(robot)
    robot.add_event_handler(cozmo.objects.EvtObjectObserved, handle_object_observed)
    
    #Initialize particle filter
    init_particles()
    
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
    
    print("\nUpdating particle filter with measurements")
    
    #Put measurements and wall positions together
    measurements = [(d1, b1), (d2, b2)]
    wall_positions = [c1, c2]
    
    #Update particle weights based on how well they match measurements
    update_particle_weights(measurements, wall_positions)
    
    #Get rid of bad particles and keep good ones
    resample_particles()
    
    #Get best estimate from particles
    xr, yr, heading = get_particle_estimate()
    
    #Calculate how uncertain we are
    std_x, std_y, std_theta = get_particle_uncertainty()
    
    print("\nParticle Filter Results:")
    print(f"Robot position: ({xr:.0f}, {yr:.0f})mm")
    print(f"Robot heading: {math.degrees(heading):.0f}°")
    print(f"Position uncertainty: ±{std_x:.0f}mm (x), ±{std_y:.0f}mm (y)")
    print(f"Heading uncertainty: ±{math.degrees(std_theta):.1f}°")
    
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
