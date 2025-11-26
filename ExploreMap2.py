import cozmo
import time
from cozmo.util import degrees, distance_mm
from cozmo.objects import LightCube
import numpy as np
import matplotlib.pyplot as plt
import math

#Tracking variables
robot_x = 0
robot_y = 0  
robot_angle = 0
path = []
walls = []
cubes_found = []
cubes_rescued = []
rescued_ids = []
start_time = time.time()
visited_positions = set()
GRID_CELL_SIZE = 150
MOVE_DISTANCE = 150

#Setup the plot
fig, ax = plt.subplots(figsize=(10, 10))
plt.ion()
plt.show()

def update_position(left_speed, right_speed, duration):
    global robot_x, robot_y, robot_angle, path, visited_positions
    
    #Wheel movement
    left_dist = left_speed * duration
    right_dist = right_speed * duration
    
    #If both wheels are at the same speed, then go straight
    if abs(left_dist - right_dist) < 0.1:
        distance = (left_dist + right_dist) / 2
        robot_x = robot_x + distance * math.cos(robot_angle)
        robot_y = robot_y + distance * math.sin(robot_angle)
    else:
        #Turning
        wheelbase = 45
        radius = wheelbase * (left_dist + right_dist) / (2 * (right_dist - left_dist))
        angle_change = (right_dist - left_dist) / wheelbase
        
        robot_x = robot_x + radius * (math.sin(robot_angle + angle_change) - math.sin(robot_angle))
        robot_y = robot_y - radius * (math.cos(robot_angle + angle_change) - math.cos(robot_angle))
        robot_angle = robot_angle + angle_change
    
    path.append((robot_x, robot_y))
    
    #Mark current grid cell as visited
    grid_x = round(robot_x / GRID_CELL_SIZE)
    grid_y = round(robot_y / GRID_CELL_SIZE)
    visited_positions.add((grid_x, grid_y))

def get_grid_cell(x, y):
    #Convert world coordinates to grid cell
    return (round(x / GRID_CELL_SIZE), round(y / GRID_CELL_SIZE))

def add_wall():
    global walls, robot_x, robot_y, robot_angle
    #When it hits something, draw a wall line there
    wall_x = robot_x + 80 * math.cos(robot_angle)
    wall_y = robot_y + 80 * math.sin(robot_angle)
    
    #Make the wall perpendicular to where we're facing
    perp_angle = robot_angle + math.pi / 2
    wall_x1 = wall_x - 100 * math.cos(perp_angle)
    wall_y1 = wall_y - 100 * math.sin(perp_angle)
    wall_x2 = wall_x + 100 * math.cos(perp_angle)
    wall_y2 = wall_y + 100 * math.sin(perp_angle)
    
    walls.append((wall_x1, wall_y1, wall_x2, wall_y2))

def add_cube(distance):
    global cubes_found
    cube_x = robot_x + distance * math.cos(robot_angle)
    cube_y = robot_y + distance * math.sin(robot_angle)
    cubes_found.append((cube_x, cube_y))

def add_rescued(distance):
    global cubes_rescued
    cube_x = robot_x + distance * math.cos(robot_angle)
    cube_y = robot_y + distance * math.sin(robot_angle)
    cubes_rescued.append((cube_x, cube_y))

def draw_map():
    global ax, robot_x, robot_y, robot_angle
    ax.clear()
    
    #Centre view on robot
    ax.set_xlim(robot_x - 600, robot_x + 600)
    ax.set_ylim(robot_y - 600, robot_y + 600)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    
    elapsed = int(time.time() - start_time)
    ax.set_title(f'Time: {elapsed}s | Walls: {len(walls)} | Found: {len(cubes_found)} | Rescued: {len(cubes_rescued)}')
    
    #Draw the path
    if len(path) > 1:
        path_xs = [p[0] for p in path]
        path_ys = [p[1] for p in path]
        ax.plot(path_xs, path_ys, 'b-', alpha=0.5)
    
    #Draw walls
    for wall in walls:
        ax.plot([wall[0], wall[2]], [wall[1], wall[3]], 'r-', linewidth=3)
    
    #Draw cubes (yellow squares)
    for cube in cubes_found:
        ax.plot(cube[0], cube[1], 'ys', markersize=15, markeredgecolor='orange', markeredgewidth=2)
    
    #Draw rescued cubes (cyan triangles)
    for rescued in cubes_rescued:
        ax.plot(rescued[0], rescued[1], 'c^', markersize=15, markeredgecolor='blue', markeredgewidth=2)
    
    #Draw Cozmo
    ax.plot(robot_x, robot_y, 'go', markersize=10)
    
    #Arrow showing which way the Cozmo is facing
    arrow_len = 80
    arrow_x = robot_x + arrow_len * math.cos(robot_angle)
    arrow_y = robot_y + arrow_len * math.sin(robot_angle)
    ax.arrow(robot_x, robot_y, arrow_x - robot_x, arrow_y - robot_y, 
             head_width=30, head_length=30, fc='green', ec='green')
    
    plt.draw()
    plt.pause(0.01)

def print_stats():
    elapsed = time.time() - start_time
    
    #Calculate total distance traveled
    total_dist = 0
    for i in range(1, len(path)):
        dx = path[i][0] - path[i-1][0]
        dy = path[i][1] - path[i-1][1]
        total_dist += math.sqrt(dx*dx + dy*dy)
    
    print("\nStats")
    print(f"Time: {elapsed:.1f}s")
    print(f"Distance: {total_dist:.0f}mm")
    print(f"Walls: {len(walls)}")
    print(f"Cubes found: {len(cubes_found)}")
    print(f"Cubes rescued: {len(cubes_rescued)}")
    if len(cubes_found) > 0:
        print(f"Success: {len(cubes_rescued)}/{len(cubes_found)} ({100*len(cubes_rescued)/len(cubes_found):.0f}%)")

def choose_new_position():
    #Choose the nearest unvisited position to the starting point (0, 0)
    global visited_positions, robot_x, robot_y
    
    #Search radius
    max_radius = 20  #Check up to 20 grid cells away
    
    best_position = None
    best_distance = float('inf')
    
    #Search in expanding squares around origin
    for radius in range(1, max_radius + 1):
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                #Only check the perimeter of each square
                if abs(dx) == radius or abs(dy) == radius:
                    grid_cell = (dx, dy)
                    
                    #Skip if already visited
                    if grid_cell in visited_positions:
                        continue
                    
                    #Convert grid cell to world coordinates
                    world_x = dx * GRID_CELL_SIZE
                    world_y = dy * GRID_CELL_SIZE
                    
                    #Calculate distance from origin (0, 0)
                    dist_from_origin = math.sqrt(world_x**2 + world_y**2)
                    
                    if dist_from_origin < best_distance:
                        best_distance = dist_from_origin
                        best_position = (world_x, world_y)
        
        #If it found something at this radius, return it
        if best_position is not None:
            print(f"Chose position {best_position}, distance from origin: {best_distance:.0f}mm")
            return best_position
    
    #If nothing is found, pick a random unvisited spot near current position
    print("No nearby unvisited positions found, exploring near current position")
    angle = np.random.uniform(0, 2 * math.pi)
    distance = GRID_CELL_SIZE * 2
    return (robot_x + distance * math.cos(angle), robot_y + distance * math.sin(angle))

def go_to_new_position(robot: cozmo.robot.Robot, target_x, target_y):
    #Move toward target position with wall detection. Returns True if reached, False if blocked.
    global robot_x, robot_y, robot_angle
    
    max_attempts = 50  #Maximum movement attempts before giving up
    attempts = 0
    
    while attempts < max_attempts:
        #Calculate distance and angle to target
        dx = target_x - robot_x
        dy = target_y - robot_y
        distance_to_target = math.sqrt(dx**2 + dy**2)
        
        #Check if we've reached the target (within tolerance)
        if distance_to_target < GRID_CELL_SIZE / 2:
            print(f"Reached target position ({target_x:.0f}, {target_y:.0f})")
            return True
        
        #Calculate desired angle to target
        desired_angle = math.atan2(dy, dx)
        angle_diff = desired_angle - robot_angle
        
        #Normalise angle difference to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        #If it needs to turn significantly, turn first
        if abs(angle_diff) > math.radians(15):
            turn_speed = 150
            wheelbase = 45
            turn_time = (abs(angle_diff) * wheelbase) / (2 * turn_speed)
            
            if angle_diff > 0:
                robot.drive_wheels(turn_speed, -turn_speed)
            else:
                robot.drive_wheels(-turn_speed, turn_speed)
            
            time.sleep(turn_time)
            robot.drive_wheels(0, 0)
            robot_angle = desired_angle
            time.sleep(0.1)
            draw_map()
            continue
        
        #Now move forward with wall detection
        #Get first image
        img1 = robot.world.latest_image
        if img1 is None:
            time.sleep(0.1)
            attempts += 1
            continue
        
        gray1 = np.array(img1.raw_image.convert('L'))
        
        #Move forward one step
        move_time = MOVE_DISTANCE / 150
        robot.drive_wheels(150, 150)
        time.sleep(move_time)
        update_position(150, 150, move_time)
        
        #Get second image
        img2 = robot.world.latest_image
        if img2 is None:
            attempts += 1
            continue
        
        gray2 = np.array(img2.raw_image.convert('L'))
        
        #Compare images - if not much changed, it probably hit something
        difference = np.mean(np.abs(gray1.astype(float) - gray2.astype(float)))
        
        if difference < 5:
            #Hit a wall
            robot.drive_wheels(0, 0)
            time.sleep(0.1)
            
            add_wall()
            print("Hit a wall while moving to target")
            
            #Back up
            robot.drive_wheels(-150, -150)
            time.sleep(move_time)
            update_position(-150, -150, move_time)
            
            robot.drive_wheels(0, 0)
            time.sleep(0.2)
            
            turn_speed = 150
            wheelbase = 45
            turn_time = (math.radians(90) * wheelbase) / (2 * turn_speed)
            
            robot.drive_wheels(turn_speed, -turn_speed)
            time.sleep(turn_time)
            robot.drive_wheels(0, 0)
            
            robot_angle = robot_angle + math.radians(90)
            time.sleep(0.3)
            draw_map()
            
            return False
        
        draw_map()
        attempts += 1
    
    print("Max attempts reached, couldn't reach target")
    return False


def scan_for_cubes(robot: cozmo.robot.Robot):
    #Do a 360-degree stepwise scan looking for cubes
    global robot_angle, rescued_ids
    
    steps = 8  45 degrees each
    step_angle = 360 / steps
    
    print("Scanning 360 degrees for cubes...")
    
    for step in range(steps):
        #Check if any cubes are visible at this angle
        for obj in robot.world.visible_objects:
            if isinstance(obj, LightCube) and obj.object_id not in rescued_ids:
                dist = math.sqrt(obj.pose.position.x**2 + obj.pose.position.y**2)
                add_cube(dist)
                print(f"Found a cube at {dist:.0f}mm during scan!")
                
                robot.drive_wheels(0, 0)
                time.sleep(0.2)
                
                #Try to go get it
                try:
                    robot.go_to_object(obj, distance_mm(50.0)).wait_for_completed(timeout=10)
                    pickup = robot.pickup_object(obj, num_retries=2)
                    pickup.wait_for_completed(timeout=15)
                    
                    if pickup.has_succeeded:
                        rescued_ids.append(obj.object_id)
                        add_rescued(dist)
                        print(f"Picked up cube")
                        time.sleep(1)
                        robot.set_lift_height(0.0).wait_for_completed()
                    else:
                        print("Failed to pick up cube")
                except:
                    print("Something went wrong whilst picking up cube")
                
                time.sleep(0.5)
                draw_map()
                return True  #Found and cube, return to main loop
        
        #Turn to next scan position
        if step < steps - 1:  #Don't turn on the last step
            #Manual wheel turn
            turn_speed = 150
            wheelbase = 45
            turn_time = (math.radians(step_angle) * wheelbase) / (2 * turn_speed)
            
            robot.drive_wheels(turn_speed, -turn_speed)
            time.sleep(turn_time)
            robot.drive_wheels(0, 0)
            
            robot_angle = robot_angle + math.radians(step_angle)
            time.sleep(0.2)
            draw_map()
    
    return False  #No cubes found

def explore(robot: cozmo.robot.Robot):
    global robot_angle, rescued_ids, visited_positions
    
    robot.camera.image_stream_enabled = True
    robot.set_lift_height(0.0).wait_for_completed()
    
    #Mark starting position as visited
    visited_positions.add((0, 0))
    
    print("Starting exploration with position-based navigation")
    
    try:
        while True:
            #Choose next position to explore
            target_x, target_y = choose_new_position()
            
            #Try to move to that position
            reached = go_to_new_position(robot, target_x, target_y)
            
            if not reached:
                print("Couldn't reach target, choosing new position")
            
            #360 degree scan for cubes at current position
            robot.drive_wheels(0, 0)
            time.sleep(0.2)
            scan_for_cubes(robot)
            
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\nStopping")
        robot.drive_wheels(0, 0)
        robot.set_lift_height(0.0).wait_for_completed()
        print_stats()
        plt.ioff()
        plt.show()

cozmo.run_program(explore)
