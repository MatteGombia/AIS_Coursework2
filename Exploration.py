import cozmo

from cozmo.util import degrees, Pose, distance_mm, speed_mmps

import numpy as np
import sys
from cozmo.objects import CustomObject

from frame2d import Frame2D
from cozmo_interface import wheelDistance, target_pose_to_velocity_linear,velocity_to_track_speed,track_speed_to_pose_change
from cozmo.util import distance_mm, speed_mmps, degrees, Angle
import matplotlib.pyplot as plt
import math

import time

cubes = {cozmo.objects.LightCube1Id: [False, None],
         cozmo.objects.LightCube2Id: [False, None],
         cozmo.objects.LightCube3Id: [False, None]}
obstacles = []
map = dict()
path = []
start_time = time.time()

RANGE_MAP_KEY = 20
MOVE_DISTANCE = 100
TOLERANCE = 20


def _distance_point_to_segment(px, py, x1, y1, x2, y2):
    """Return shortest distance from point (px,py) to segment (x1,y1)-(x2,y2).
    All units are assumed to be mm.
    """
    vx = x2 - x1
    vy = y2 - y1
    wx = px - x1
    wy = py - y1
    seg_len_sq = vx*vx + vy*vy
    if seg_len_sq == 0:
        return math.hypot(px - x1, py - y1)
    t = (wx*vx + wy*vy) / seg_len_sq
    if t <= 0:
        cx, cy = x1, y1
    elif t >= 1:
        cx, cy = x2, y2
    else:
        cx = x1 + t * vx
        cy = y1 + t * vy
    return math.hypot(px - cx, py - cy)


def is_path_blocked(start_f: Frame2D, end_f, obstacles_list, clearance_mm=50.0):
    """Check whether any obstacle in obstacles_list lies within clearance_mm
    (plus obstacle radius if available) of the straight-line segment from
    start_f to end_f. Returns (blocked: bool, obstacle, distance_mm).
    """
    sx, sy = start_f.x(), start_f.y()
    ex, ey = end_f[0], end_f[1]
    for obs in obstacles_list:
        # some observed objects may not yet have a pose
        if not hasattr(obs, 'pose') or obs.pose is None:
            continue
        try:
            of = Frame2D.fromPose(obs.pose)
            ox, oy = obs.x(), obs.y()
        except Exception:
            continue
        dist = _distance_point_to_segment(ox, oy, sx, sy, ex, ey)
        
        if dist <= (clearance_mm):
            return True
    return False


def handle_object_observed(evt, **kw):
    global obstacles
    # This will be called whenever an EvtObjectDisappeared is dispatched -
    # whenever an Object goes out of view.
    if isinstance(evt.obj, CustomObject):
        print("Cozmo observed a %s" % str(evt.obj.object_type))
        print(evt.obj)
        for obstacle in obstacles:
            if obstacle.object_id == evt.obj.object_id:
                return
        obstacles.append(evt.obj)
    
def detect_cubes(robot: cozmo.robot.Robot):
    cubeIDs = (cozmo.objects.LightCube1Id,cozmo.objects.LightCube2Id,cozmo.objects.LightCube3Id)
    for cubeID in cubeIDs: 
        cube = robot.world.get_light_cube(cubeID)
        if cube is not None and cube.is_visible:
            cubePose2D = Frame2D.fromPose(cube.pose)
            print("   2D frame: " + str(cubePose2D))
            #robot.go_to_object(cube, distance_mm(50.0)).wait_for_completed()
            cubes[cubeID][1] = cubePose2D
            cubes[cubeID][0] = True
    
def add_reachable_position_to_map(robot: cozmo.robot.Robot):
    robotPose = Frame2D.fromPose(robot.pose)
    possible_map_positions = [
        (robotPose.x() + MOVE_DISTANCE, robotPose.y()),
        (robotPose.x() - MOVE_DISTANCE, robotPose.y()),
        (robotPose.x(), robotPose.y() + MOVE_DISTANCE),
        (robotPose.x(), robotPose.y() - MOVE_DISTANCE)
    ]
    for position in possible_map_positions:
        blocked = is_path_blocked(robotPose, position, obstacles, clearance_mm=80.0)
        if blocked:
            print("Path to position %s is blocked by an obstacle." % str(position))
            possible_map_positions.remove(position)

    return possible_map_positions

def choose_next_position(possible_map_positions):
    for position in possible_map_positions:
        position_seen = False
        print("Possible position to explore: " + str(position))
        for map_key in map.keys():
            if position > map_key[0]-RANGE_MAP_KEY and position < map_key[0]+RANGE_MAP_KEY and position > map_key[1]-RANGE_MAP_KEY and position < map_key[1]+RANGE_MAP_KEY:
                position_seen = True
        if(not position_seen):
            return position
    return possible_map_positions[0]

def draw_map():
    global ax, robot_x, robot_y, robot_angle
    ax.clear()
    
    #Centre view on robot
    ax.set_xlim(robot_x - 600, robot_x + 600)
    ax.set_ylim(robot_y - 600, robot_y + 600)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    
    elapsed = int(time.time() - start_time)
    ax.set_title(f'Time: {elapsed}s | Walls: {len(obstacles)} | Found: {len(cubes)} | Rescued: {len(cubes)}')
    
    #Draw the path
    if len(path) > 1:
        path_xs = [p[0] for p in path]
        path_ys = [p[1] for p in path]
        ax.plot(path_xs, path_ys, 'b-', alpha=0.5)
    
    #Draw walls
    for wall in obstacles:
        ax.plot([wall[0], wall[1]])
    
    #Draw cubes (yellow squares)
    for cube in cubes:
        if cube[0] == True:
            ax.plot(cube[1][0], cube[1][1], 'ys', markersize=15, markeredgecolor='orange', markeredgewidth=2)
    
    #Draw rescued cubes (cyan triangles)
    # for rescued in cubes_rescued:
    #     ax.plot(rescued[0], rescued[1], 'c^', markersize=15, markeredgecolor='blue', markeredgewidth=2)
    
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

def go_to_position(robot: cozmo.robot.Robot, target_x, target_y):
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
        if distance_to_target < TOLERANCE / 2:
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
        #update_position(150, 150, move_time)
        
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
    


def Exploration(robot: cozmo.robot.Robot):
    found_cube = False
    search_steps = 18  #Amount of search cycle repetitions

    robotPose = Frame2D.fromPose(robot.pose)
    path.append((robotPose.x(), robotPose.y()))
    print("Robot Pose: " + str(robotPose))
    print("Map: " + str(map))

    try:
        for _ in range(search_steps):
            detect_cubes(robot)
            robot.turn_in_place(degrees(20)).wait_for_completed()
            time.sleep(0.1)

        #Remove blocked positions from possible map p
        possible_map_positions=add_reachable_position_to_map(robot)

        #Update map with positions
        map[(robotPose.x(), robotPose.y())] = possible_map_positions

        #go to next unseen position
        target = choose_next_position(possible_map_positions)

        print(target)
    
        time.sleep(0.2)
        if not found_cube:
            print("Cube was not found.")
    except Exception as e:
        print("Error:", e)
    finally:
        #robot.stop_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
        pass

def cozmo_drive_to_target(robot: cozmo.robot.Robot):
    robot.add_event_handler(cozmo.objects.EvtObjectObserved, handle_object_observed)
    while True:
        Exploration(robot)








cozmo.run_program(cozmo_drive_to_target)