import cozmo
import math
import time
import numpy as np
from statistics import mean
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from cozmo.util import degrees, Angle
from cozmo.objects import CustomObject, CustomObjectMarkers, CustomObjectTypes

walls = []
marked_walls_seen = []
walls_angles = []

NUM_PARTICLES = 500
MOTION_NOISE_TRANS = 30.0
MOTION_NOISE_ROT = 0.15
SENSOR_NOISE_DIST = 50.0
SENSOR_NOISE_BEARING = 0.2


class Particle:
    def __init__(self, x, y, theta, weight=1.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight


class ParticleFilter:
    def __init__(self, num_particles, map_walls, map_bounds):
        self.num_particles = num_particles
        self.map_walls = map_walls
        self.map_bounds = map_bounds
        self.particles = []
        self.initialise_particles_uniform()
        
    def initialise_particles_uniform(self):
        x_min, x_max, y_min, y_max = self.map_bounds
        self.particles = []
        for _ in range(self.num_particles):
            x = np.random.uniform(x_min, x_max)
            y = np.random.uniform(y_min, y_max)
            theta = np.random.uniform(-math.pi, math.pi)
            self.particles.append(Particle(x, y, theta, 1.0 / self.num_particles))
    
    def initialise_particles_gaussian(self, mean_x, mean_y, mean_theta, std_x, std_y, std_theta):
        self.particles = []
        for _ in range(self.num_particles):
            x = np.random.normal(mean_x, std_x)
            y = np.random.normal(mean_y, std_y)
            theta = self.normalise_angle(np.random.normal(mean_theta, std_theta))
            self.particles.append(Particle(x, y, theta, 1.0 / self.num_particles))
    
    def normalise_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def motion_update(self, delta_x, delta_y, delta_theta):
        for particle in self.particles:
            noisy_dx = delta_x + np.random.normal(0, MOTION_NOISE_TRANS)
            noisy_dy = delta_y + np.random.normal(0, MOTION_NOISE_TRANS)
            noisy_dtheta = delta_theta + np.random.normal(0, MOTION_NOISE_ROT)
            
            cos_theta = math.cos(particle.theta)
            sin_theta = math.sin(particle.theta)
            
            particle.x += noisy_dx * cos_theta - noisy_dy * sin_theta
            particle.y += noisy_dx * sin_theta + noisy_dy * cos_theta
            particle.theta = self.normalise_angle(particle.theta + noisy_dtheta)
    
    def measurement_update(self, observed_walls):
        if len(observed_walls) == 0:
            return
        
        for particle in self.particles:
            particle.weight = self.compute_particle_weight(particle, observed_walls)
        
        total_weight = sum(p.weight for p in self.particles)
        if total_weight > 0:
            for particle in self.particles:
                particle.weight /= total_weight
        else:
            for particle in self.particles:
                particle.weight = 1.0 / self.num_particles
    
    def compute_particle_weight(self, particle, observed_walls):
        weight = 1.0
        
        for obs_wall_pos, obs_dist, obs_bearing in observed_walls:
            best_match_prob = 0.0
            
            for map_wall_pos in self.map_walls:
                expected_dist, expected_bearing = self.expected_measurement(
                    particle, map_wall_pos
                )
                
                dist_prob = self.gaussian_probability(
                    obs_dist, expected_dist, SENSOR_NOISE_DIST
                )
                bearing_prob = self.gaussian_probability(
                    self.normalise_angle(obs_bearing - expected_bearing), 0.0, SENSOR_NOISE_BEARING
                )
                
                match_prob = dist_prob * bearing_prob
                best_match_prob = max(best_match_prob, match_prob)
            
            weight *= (best_match_prob + 1e-10)
        
        return weight
    
    def expected_measurement(self, particle, wall_pos):
        dx = wall_pos[0] - particle.x
        dy = wall_pos[1] - particle.y
        
        dist = math.hypot(dx, dy)
        global_bearing = math.atan2(dy, dx)
        relative_bearing = self.normalise_angle(global_bearing - particle.theta)
        
        return dist, relative_bearing
    
    def gaussian_probability(self, value, mean, std):
        exponent = -0.5 * ((value - mean) / std) ** 2
        return (1.0 / (std * math.sqrt(2 * math.pi))) * math.exp(exponent)
    
    def resample(self):
        new_particles = []
        weights = [p.weight for p in self.particles]
        
        cumsum = np.cumsum(weights)
        
        step = 1.0 / self.num_particles
        u = np.random.uniform(0, step)
        
        j = 0
        for i in range(self.num_particles):
            while u > cumsum[j]:
                j += 1
            
            old_particle = self.particles[j]
            new_particle = Particle(old_particle.x, old_particle.y, old_particle.theta)
            new_particles.append(new_particle)
            
            u += step
        
        self.particles = new_particles
        for particle in self.particles:
            particle.weight = 1.0 / self.num_particles
    
    def get_estimated_pose(self):
        if len(self.particles) == 0:
            return None, None, None
        
        x_weighted = sum(p.x * p.weight for p in self.particles)
        y_weighted = sum(p.y * p.weight for p in self.particles)
        
        sin_sum = sum(math.sin(p.theta) * p.weight for p in self.particles)
        cos_sum = sum(math.cos(p.theta) * p.weight for p in self.particles)
        theta_weighted = math.atan2(sin_sum, cos_sum)
        
        return x_weighted, y_weighted, theta_weighted
    
    def get_particle_spread(self):
        xs = [p.x for p in self.particles]
        ys = [p.y for p in self.particles]
        std_x = np.std(xs)
        std_y = np.std(ys)
        return math.sqrt(std_x**2 + std_y**2)


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
    global walls, marked_walls_seen, walls_angles
    
    if isinstance(evt.obj, CustomObject):
        if evt.obj not in marked_walls_seen:
            marked_walls_seen.append(evt.obj)
            print(f"Wall detected: {evt.obj.object_type} at ({evt.obj.pose.position.x:.1f}, {evt.obj.pose.position.y:.1f})")
            add_wall(evt.obj.pose.position.x, evt.obj.pose.position.y)
            walls_angles.append(evt.obj.pose.rotation.angle_z.radians + math.pi / 2)


def normalise_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def get_marker_measurements(robot, marker_obj):
    dx = marker_obj.pose.position.x - robot.pose.position.x
    dy = marker_obj.pose.position.y - robot.pose.position.y
    
    dist = math.hypot(dx, dy)
    global_bearing = math.atan2(dy, dx)
    robot_heading = robot.pose.rotation.angle_z.radians
    
    rel_bearing = normalise_angle(global_bearing - robot_heading)
    return dist, rel_bearing


def scan_for_walls(robot: cozmo.robot.Robot):
    global marked_walls_seen, walls, walls_angles
    
    marked_walls_seen.clear()
    walls.clear()
    walls_angles.clear()
    
    steps = 18
    step_angle = 20
    
    print("\nScanning for walls (360° rotation)...")
    
    for step in range(steps):
        print(f"  Step {step+1}/{steps} - Walls found: {len(marked_walls_seen)}")
        
        if step < steps - 1:
            robot.turn_in_place(degrees(step_angle)).wait_for_completed()
            time.sleep(0.3)
    
    print(f"Scan complete - found {len(marked_walls_seen)} walls\n")
    
    return len(marked_walls_seen) >= 2


def visualise_particles(particle_filter, robot, iteration):
    fig, ax = plt.subplots(figsize=(10, 10))
    
    for particle in particle_filter.particles:
        alpha = min(particle.weight * particle_filter.num_particles * 5, 1.0)
        ax.plot(particle.x, particle.y, 'g.', alpha=alpha, markersize=3)
    
    for wall_pos in particle_filter.map_walls:
        ax.plot(wall_pos[0], wall_pos[1], 'rs', markersize=15, markeredgewidth=2)
    
    est_x, est_y, est_theta = particle_filter.get_estimated_pose()
    ax.plot(est_x, est_y, 'bo', markersize=15, label='Estimated pose')
    
    arrow_len = 100
    arrow_x = est_x + arrow_len * math.cos(est_theta)
    arrow_y = est_y + arrow_len * math.sin(est_theta)
    ax.arrow(est_x, est_y, arrow_x - est_x, arrow_y - est_y,
             head_width=30, head_length=30, fc='blue', ec='blue')
    
    robot_x = robot.pose.position.x
    robot_y = robot.pose.position.y
    robot_theta = robot.pose.rotation.angle_z.radians
    ax.plot(robot_x, robot_y, 'ro', markersize=15, label='Actual pose')
    
    arrow_x = robot_x + arrow_len * math.cos(robot_theta)
    arrow_y = robot_y + arrow_len * math.sin(robot_theta)
    ax.arrow(robot_x, robot_y, arrow_x - robot_x, arrow_y - robot_y,
             head_width=30, head_length=30, fc='red', ec='red')
    
    spread = particle_filter.get_particle_spread()
    
    ax.set_xlim(est_x - 1000, est_x + 1000)
    ax.set_ylim(est_y - 1000, est_y + 1000)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_title(f'Iteration {iteration} | Particles: {NUM_PARTICLES} | Spread: {spread:.1f}mm')
    
    plt.savefig(f'localisation_step_{iteration:03d}.png', dpi=100, bbox_inches='tight')
    plt.close()


def main(robot: cozmo.robot.Robot):
    print("="*70)
    print("MONTE CARLO LOCALISATION (PARTICLE FILTER)")
    print("="*70)
    
    robot.camera.image_stream_enabled = True
    print("✓ Camera enabled")
    
    create_cozmo_walls(robot)
    print("✓ Walls defined")
    
    robot.add_event_handler(cozmo.objects.EvtObjectObserved, handle_object_observed)
    print("✓ Event handler registered")
    
    time.sleep(1)
    robot.set_head_angle(Angle(0)).wait_for_completed()
    time.sleep(1)
    
    success = scan_for_walls(robot)
    
    if not success or len(marked_walls_seen) < 2:
        print(f"ERROR: Insufficient walls detected ({len(marked_walls_seen)})")
        return
    
    print(f"Map walls: {len(walls)} walls detected")
    for i, (wx, wy) in enumerate(walls):
        print(f"  Wall {i+1}: ({wx:.1f}, {wy:.1f})")
    
    map_bounds = (-1000, 1000, -1000, 1000)
    
    particle_filter = ParticleFilter(NUM_PARTICLES, walls, map_bounds)
    print(f"\n✓ Particle filter initialised with {NUM_PARTICLES} particles")
    
    print("\n" + "="*70)
    print("PROBABILISTIC LOCALISATION")
    print("="*70)
    
    prev_x = robot.pose.position.x
    prev_y = robot.pose.position.y
    prev_theta = robot.pose.rotation.angle_z.radians
    
    for iteration in range(10):
        print(f"\n--- Iteration {iteration + 1} ---")
        
        curr_x = robot.pose.position.x
        curr_y = robot.pose.position.y
        curr_theta = robot.pose.rotation.angle_z.radians
        
        delta_x = curr_x - prev_x
        delta_y = curr_y - prev_y
        delta_theta = normalise_angle(curr_theta - prev_theta)
        
        if abs(delta_x) > 1 or abs(delta_y) > 1 or abs(delta_theta) > 0.01:
            print(f"Motion detected: Δx={delta_x:.1f}, Δy={delta_y:.1f}, Δθ={math.degrees(delta_theta):.1f}°")
            particle_filter.motion_update(delta_x, delta_y, delta_theta)
        
        observed_walls = []
        for marker_obj in marked_walls_seen:
            if marker_obj.is_visible:
                dist, bearing = get_marker_measurements(robot, marker_obj)
                wall_pos = (marker_obj.pose.position.x, marker_obj.pose.position.y)
                observed_walls.append((wall_pos, dist, bearing))
        
        print(f"Observed {len(observed_walls)} visible walls")
        
        if len(observed_walls) > 0:
            particle_filter.measurement_update(observed_walls)
            particle_filter.resample()
        
        est_x, est_y, est_theta = particle_filter.get_estimated_pose()
        spread = particle_filter.get_particle_spread()
        
        actual_x = robot.pose.position.x
        actual_y = robot.pose.position.y
        actual_theta = robot.pose.rotation.angle_z.radians
        
        error_x = est_x - actual_x
        error_y = est_y - actual_y
        error_dist = math.hypot(error_x, error_y)
        error_theta = normalise_angle(est_theta - actual_theta)
        
        print(f"Estimated pose: ({est_x:.1f}, {est_y:.1f}, {math.degrees(est_theta):.1f}°)")
        print(f"Actual pose:    ({actual_x:.1f}, {actual_y:.1f}, {math.degrees(actual_theta):.1f}°)")
        print(f"Error: {error_dist:.1f}mm position, {math.degrees(error_theta):.1f}° heading")
        print(f"Particle spread: {spread:.1f}mm")
        
        visualise_particles(particle_filter, robot, iteration + 1)
        
        prev_x = curr_x
        prev_y = curr_y
        prev_theta = curr_theta
        
        if iteration < 9:
            robot.turn_in_place(degrees(36)).wait_for_completed()
            time.sleep(0.5)
    
    print("\n" + "="*70)
    print("FINAL RESULTS")
    print("="*70)
    est_x, est_y, est_theta = particle_filter.get_estimated_pose()
    spread = particle_filter.get_particle_spread()
    
    actual_x = robot.pose.position.x
    actual_y = robot.pose.position.y
    actual_theta = robot.pose.rotation.angle_z.radians
    
    error_dist = math.hypot(est_x - actual_x, est_y - actual_y)
    error_theta = normalise_angle(est_theta - actual_theta)
    
    print(f"Estimated position: ({est_x:.1f}, {est_y:.1f}) mm")
    print(f"Estimated heading: {math.degrees(est_theta):.1f}°")
    print(f"Actual position: ({actual_x:.1f}, {actual_y:.1f}) mm")
    print(f"Actual heading: {math.degrees(actual_theta):.1f}°")
    print(f"Position error: {error_dist:.1f}mm")
    print(f"Heading error: {math.degrees(error_theta):.1f}°")
    print(f"Convergence (spread): {spread:.1f}mm")
    print("="*70)


if __name__ == "__main__":
    cozmo.run_program(main)
