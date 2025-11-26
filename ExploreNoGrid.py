import cozmo
import time
from cozmo.util import degrees, distance_mm, Frame2D
from cozmo.objects import LightCube, LightCube1Id, LightCube2Id, LightCube3Id
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
start_time = time.time()
visited_positions = set()  #Track visited grid cells
GRID_CELL_SIZE = 150  #Size of each grid cell in mm
MOVE_DISTANCE = 150  #Distance to move per step

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
    """Convert world coordinates to grid cell"""
    return (round(x / GRID_CELL_SIZE), round(y / GRID_CELL_SIZE))

def add_wall():
    global walls, robot_x, robot_y, robot_angle
    #Record wall position in front of the robot
    wall_x = robot_x + 80 * math.cos(robot_angle)
    wall_y = robot_y + 80 * math.sin(robot_angle)
    walls.append((wall_x, wall_y))

def add_cube(distance):
    global cubes_found
    cube_x = robot_x + distance * math.cos(robot_angle)
    cube_y = robot_y + distance * math.sin(robot_angle)
    cubes_found.append((cube_x, cube_y))

def draw_map():
    global ax, robot_x, robot_y, robot_angle
    ax.clear()
    
    #Centre view on robot
    ax.set_xlim(robot_x - 600, robot_x + 600)
    ax.set_ylim(robot_y - 600, robot_y + 600)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    
    elapsed = int(time.time() - start_time)
    ax.set_title(f'Time: {elapsed}s | Walls: {len(walls)} | Cubes Found: {len(cubes_found)}')
    
    #Draw the path
    if len(path) > 1:
        path_xs = [p[0] for p in path]
        path_ys = [p[1] for p in path]
        ax.plot(path_xs, path_ys, 'b-', alpha=0.5)
    
    #Draw walls (as points)
    for wall_x, wall_y in walls:
        ax.plot(wall_x, wall_y, 'rs', markersize=10, markeredgecolor='darkred', markeredgewidth=2)
    
    #Draw cubes (yellow squares)
    for cube in cubes_found:
        ax.plot(cube[0], cube[1], 'ys', markersize=15, markeredgecolor='orange', markeredgewidth=2)
    
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

def choose_new_position():
    #Choose the nearest unvisited position to the starting point (0, 0)
    global visited_positions, robot_x, robot_y
    
    #Search radius - how far out to look for unvisited cells
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
        
        #If we found something at this radius, return it
        if best_position is not None:
            print(f"Chose position {best_position}, distance from origin: {best_distance:.0f}mm")
            return best_position
    
    #If nothing found, pick a random unvisited spot near current position
    print("No nearby unvisited positions found, exploring near current position")
    angle = np.random.uniform(0, 2 * math.pi)
    distance = GRID_CELL_SIZE * 2
    return (robot_x + distance * math.cos(angle), robot_y + distance * math.sin(angle))

def go_to_new_position(robot: cozmo.robot.Robot, target_x, target_y):
    """Move toward target position with wall detection. Returns True if reached, False if blocked."""
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
        
        #Normalize angle difference to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        #If we need to turn significantly, turn first
        if abs(angle_diff) > math.radians(15):
            #Convert angle_diff from radians to degrees and turn
            turn_angle = math.degrees(angle_diff)
            robot.turn_in_place(degrees(turn_angle)).wait_for_completed()
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
        move_time = MOVE_DISTANCE / 150  #150 mm/s speed
        robot.drive_wheels(150, 150)
        time.sleep(move_time)
        update_position(150, 150, move_time)
        
        #Get second image
        img2 = robot.world.latest_image
        if img2 is None:
            attempts += 1
            continue
        
        gray2 = np.array(img2.raw_image.convert('L'))
        
        #Compare images - if not much changed, we probably hit something
        difference = np.mean(np.abs(gray1.astype(float) - gray2.astype(float)))
        
        if difference < 5:
            #Hit a wall!
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
            
            #Turn 90 degrees to try to go around
            robot.turn_in_place(degrees(90)).wait_for_completed()
            robot_angle = robot_angle + math.radians(90)
            time.sleep(0.3)
            draw_map()
            
            #Return False to indicate we couldn't reach the target directly
            return False
        
        draw_map()
        attempts += 1
    
    print("Max attempts reached, couldn't reach target")
    return False


def scan_for_cubes(robot: cozmo.robot.Robot):
    """Do a 360-degree stepwise scan looking for cubes"""
    global robot_angle
    
    steps = 18  #18 steps * 20 degrees = 360 degrees
    step_angle = 20
    
    print("Scanning 360 degrees for cubes...")
    
    cubeIDs = (LightCube1Id, LightCube2Id, LightCube3Id)
    
    for step in range(steps):
        #Check each cube ID to see if visible at this angle
        for cubeID in cubeIDs:
            cube = robot.world.get_light_cube(cubeID)
            if cube is not None and cube.is_visible:
                #Get cube's 2D pose
                cubePose2D = Frame2D.fromPose(cube.pose)
                print(f"Found cube {cubeID} - 2D frame: {cubePose2D}")
                
                #Calculate distance to cube
                dist = math.sqrt(cube.pose.position.x**2 + cube.pose.position.y**2)
                add_cube(dist)
                print(f"Found a cube at {dist:.0f}mm during scan!")
                
                robot.drive_wheels(0, 0)
                time.sleep(0.2)
                draw_map()
                #Continue scanning for other cubes
        
        #Turn to next scan position
        if step < steps - 1:  #Don't turn on the last step
            robot.turn_in_place(degrees(step_angle)).wait_for_completed()
            robot_angle = robot_angle + math.radians(step_angle)
            time.sleep(0.1)
            draw_map()
    
    return False  #Scan complete

def explore(robot: cozmo.robot.Robot):
    global robot_angle, visited_positions
    
    robot.camera.image_stream_enabled = True
    robot.set_lift_height(0.0).wait_for_completed()
    
    #Mark starting position as visited
    visited_positions.add((0, 0))
    
    print("Starting exploration with position-based navigation")
    
    try:
        while True:
            #Choose next position to explore (nearest to origin that we haven't visited)
            target_x, target_y = choose_new_position()
            
            #Try to move to that position
            reached = go_to_new_position(robot, target_x, target_y)
            
            if not reached:
                print("Couldn't reach target, choosing new position")
            
            #Do a 360 degree scan for cubes at current position
            robot.drive_wheels(0, 0)
            time.sleep(0.2)
            scan_for_cubes(robot)
            
            #Small pause between navigation cycles
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\nStopping")
        robot.drive_wheels(0, 0)
        robot.set_lift_height(0.0).wait_for_completed()
        print_stats()
        plt.ioff()
        plt.show()

cozmo.run_program(explore)
