import cozmo
import time
from cozmo.util import degrees, distance_mm, Angle, Pose
from cozmo.objects import LightCube, LightCube1Id, LightCube2Id, LightCube3Id
import numpy as np
import matplotlib.pyplot as plt
import math
from frame2d import Frame2D
from cozmo.objects import CustomObject, CustomObjectMarkers, CustomObjectTypes

#Tracking variables
robot_x = 0
robot_y = 0  
map = dict()
path = []
walls = []
marked_walls_seen=[]
cubes = {cozmo.objects.LightCube1Id: [False, None],
         cozmo.objects.LightCube2Id: [False, None],
         cozmo.objects.LightCube3Id: [False, None]}
start_time = time.time()
MOVE_DISTANCE = 400  #Distance to move per step
RANGE_MAP_KEY = 20

#Setup the plot
fig, ax = plt.subplots(figsize=(10, 10))
plt.ion()
plt.show()
    
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
    for i in range(0,8):
        cozmo_walls.append(robot.world.define_custom_wall(types[i], markers[i], 200, 60, 50, 50, True))
    
    for i in range(8,16):
        cozmo_walls.append(robot.world.define_custom_wall(types[i], markers[i], 300, 60, 50, 50, True))

def add_wall(wall_x, wall_y):
    walls.append((wall_x, wall_y))


def draw_map(robot: cozmo.robot.Robot):
    global ax
    ax.clear()

    robotPose = Frame2D.fromPose(robot.pose)
    
    #Centre view on robot
    ax.set_xlim(robotPose.x() - 600, robotPose.x() + 600)
    ax.set_ylim(robotPose.y() - 600, robotPose.y() + 600)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    
    elapsed = int(time.time() - start_time)
    ax.set_title(f'Time: {elapsed}s | Walls: {len(walls)} | Cubes Found: {len(cubes)}')
    
    #Draw the path
    if len(path) > 1:
        path_xs = [p[0] for p in path]
        path_ys = [p[1] for p in path]
        ax.plot(path_xs, path_ys, 'b-', alpha=0.5)
    
    #Draw walls (as points)
    for wall_x, wall_y in walls:
        ax.plot(wall_x, wall_y, 'rs', markersize=10, markeredgecolor='darkred', markeredgewidth=2)
    
    #Draw cubes (yellow squares)
    cubeIDs = (cozmo.objects.LightCube1Id,cozmo.objects.LightCube2Id,cozmo.objects.LightCube3Id)
    for cubeID in cubeIDs: 
        if cubes[cubeID][0] == True:
            ax.plot(cubes[cubeID][1].x(), cubes[cubeID][1].y(), 'ys', markersize=15, markeredgecolor='orange', markeredgewidth=2)
    
    #Draw Cozmo
    ax.plot(robotPose.x(), robotPose.y(), 'go', markersize=10)
    
    #Arrow showing which way the Cozmo is facing
    
    robot_angle = robotPose.angle()
    arrow_len = 80
    arrow_x = robotPose.x() + arrow_len * math.cos(robot_angle)
    arrow_y = robotPose.y() + arrow_len * math.sin(robot_angle)
    ax.arrow(robotPose.x(), robotPose.y(), arrow_x - robotPose.x(), arrow_y - robotPose.y(), 
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
    print(f"Cubes found: {len(cubes)}")

def go_to_new_position(robot: cozmo.robot.Robot, target_x, target_y):
    #Test for exploration
    robot.go_to_pose(Pose(target_x, target_y, 0, angle_z=degrees(0)))
    time.sleep(10.0)

def scan_for_cubes(robot: cozmo.robot.Robot):
    #Do a 360-degree stepwise scan looking for cubes
    
    steps = 18  #18 steps * 20 degrees = 360 degrees
    step_angle = 20
    
    print("Scanning 360 degrees for cubes...")
    
    cubeIDs = (LightCube1Id, LightCube2Id, LightCube3Id)
    
    for step in range(steps):
        #Check each cube ID to see if visible at this angle
        cubeIDs = (cozmo.objects.LightCube1Id,cozmo.objects.LightCube2Id,cozmo.objects.LightCube3Id)
        for cubeID in cubeIDs: 
            cube = robot.world.get_light_cube(cubeID)
            if cube is not None and cube.is_visible:
                cubePose2D = Frame2D.fromPose(cube.pose)
                #robot.go_to_object(cube, distance_mm(50.0)).wait_for_completed()
                cubes[cubeID][1] = cubePose2D
                cubes[cubeID][0] = True
                print(f"Found a cube at" + str(cubePose2D) + "mm during scan!")
                
                robot.drive_wheels(0, 0)
                time.sleep(0.2)
                draw_map(robot)
                #Continue scanning for other cubes
        
        #Turn to next scan position
        if step < steps - 1:  #Don't turn on the last step
            robot.turn_in_place(degrees(step_angle)).wait_for_completed()
            time.sleep(0.1)
            draw_map(robot)
    
    return False  #Scan complete

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
        try:
            ox, oy = obs[0], obs[1]
        except Exception:
            continue
        dist = _distance_point_to_segment(ox, oy, sx, sy, ex, ey)
        
        if dist <= (clearance_mm):
            return True
    return False
    
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
        blocked = is_path_blocked(robotPose, position, walls, clearance_mm=210.0)
        if blocked:
            print("Path to position %s is BLOCKED by an obstacle." % str(position))
            possible_map_positions.remove(position)

    return possible_map_positions

def choose_next_position(possible_map_positions):
    for position in possible_map_positions:
        position_seen = False
        print("Possible position to explore: " + str(position))
        for map_key in map.keys():
            if position[0] > map_key[0]-RANGE_MAP_KEY and position[0] < map_key[0]+RANGE_MAP_KEY and position[1] > map_key[1]-RANGE_MAP_KEY and position[1] < map_key[1]+RANGE_MAP_KEY:
                position_seen = True
        if(not position_seen):
            return position
    return possible_map_positions[0]

def handle_object_observed(evt, **kw):
    global walls
    # This will be called whenever an EvtObjectDisappeared is dispatched -
    # whenever an Object goes out of view.
    if isinstance(evt.obj, CustomObject):
        if evt.obj not in marked_walls_seen:
            marked_walls_seen.append(evt.obj)
            print("Cozmo observed a wall at %s" % str(evt.obj.pose.position))
            print(evt.obj)
            add_wall(evt.obj.pose.position.x, evt.obj.pose.position.y)
        
def explore(robot: cozmo.robot.Robot):
    global path

    robot.camera.image_stream_enabled = True
    robot.set_lift_height(0.0).wait_for_completed()
    
    print("Starting exploration with position-based navigation")
    
    try:
        while True:
            #Do a 360 degree scan for cubes at current position
            robotPose = Frame2D.fromPose(robot.pose)
            path.append((robotPose.x(), robotPose.y()))

            scan_for_cubes(robot)
            
            #Small pause between navigation cycles
            time.sleep(0.5)

            #Choose next position to explore (nearest to origin that we haven't visited)
            #Remove blocked positions from possible map p
            possible_map_positions=add_reachable_position_to_map(robot)

            robotPose = Frame2D.fromPose(robot.pose)
            path.append((robotPose.x(), robotPose.y()))
            #Update map with positions
            map[(robotPose.x(), robotPose.y())] = possible_map_positions

            #go to next unseen position
            target = choose_next_position(possible_map_positions)
            
            #Try to move to that position
            reached = go_to_new_position(robot, target[0], target[1])
            
            if not reached:
                print("Couldn't reach target, choosing new position")
            
    except KeyboardInterrupt:
        print("\nStopping")
        robot.drive_wheels(0, 0)
        robot.set_lift_height(0.0).wait_for_completed()
        print_stats()
        plt.ioff()
        plt.show()

def main(robot: cozmo.robot.Robot):
    create_cozmo_walls(robot)
    robot.add_event_handler(cozmo.objects.EvtObjectObserved, handle_object_observed)
    time.sleep(1)
    robot.set_head_angle(Angle(0)).wait_for_completed
    time.sleep(1)
    explore(robot)


cozmo.run_program(main)
