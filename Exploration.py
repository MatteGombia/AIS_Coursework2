import cozmo

from cozmo.util import degrees, Pose, distance_mm, speed_mmps

import numpy as np
import sys
from cozmo.objects import CustomObject

from frame2d import Frame2D
from cozmo_interface import wheelDistance, target_pose_to_velocity_linear,velocity_to_track_speed,track_speed_to_pose_change
from cozmo.util import distance_mm, speed_mmps, degrees, Angle

import time
import math

cubes = {cozmo.objects.LightCube1Id: [False, None],
         cozmo.objects.LightCube2Id: [False, None],
         cozmo.objects.LightCube3Id: [False, None]}
obstacles = []
map = dict()


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

def Exploration(robot: cozmo.robot.Robot):
    found_cube = False
    search_steps = 18  #Amount of search cycle repetitions

    robotPose = Frame2D.fromPose(robot.pose)
    possible_map_positions = [
        (robotPose.x() + 100, robotPose.y()),
        (robotPose.x() - 100, robotPose.y()),
        (robotPose.x(), robotPose.y() + 100),
        (robotPose.x(), robotPose.y() - 100)
    ]
    print("Robot Pose: " + str(robotPose))
    print("Map: " + str(map))

    try:
        for _ in range(search_steps):

            cubeIDs = (cozmo.objects.LightCube1Id,cozmo.objects.LightCube2Id,cozmo.objects.LightCube3Id)
            for cubeID in cubeIDs: 
                cube = robot.world.get_light_cube(cubeID)
                if cube is not None and cube.is_visible:
                    cubePose2D = Frame2D.fromPose(cube.pose)
                    print("   2D frame: " + str(cubePose2D))
                    #robot.go_to_object(cube, distance_mm(50.0)).wait_for_completed()
                    cubes[cubeID][1] = cubePose2D
                    cubes[cubeID][0] = True

            #time.sleep(0.1)

            robot.turn_in_place(degrees(20)).wait_for_completed()
            time.sleep(0.1)

        #Remove blocked positions from possible map positions
        for position in possible_map_positions:
            blocked = is_path_blocked(robotPose, position, obstacles, clearance_mm=80.0)
            if blocked:
                print("Path to position %s is blocked by an obstacle." % str(position))
                possible_map_positions.remove(position)

        #Update map with positions
        map[(robotPose.x(), robotPose.y())] = possible_map_positions

        #go to next unseen position
        for position in possible_map_positions:
            print("Possible position to explore: " + str(position))
            if position not in map.keys():
                target_velocity = target_pose_to_velocity_linear(robotPose, position, max_speed_mmps=100)
                left_speed, right_speed = velocity_to_track_speed(target_velocity, wheelDistance)
                robot.drive_wheels(left_speed, right_speed)
                #Estimate time to reach target
                distance_to_target = math.hypot(position[0] - robotPose.x(), position[1] - robotPose.y())
                estimated_time = distance_to_target / target_velocity
                time.sleep(estimated_time)
                robot.stop_all_motors()
                robotPose = Frame2D.fromPose(robot.pose)
                break
    
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