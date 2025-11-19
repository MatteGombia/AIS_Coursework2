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
        robotPose + Frame2D.fromTranslation(distance_mm(100), 0),
        robotPose + Frame2D.fromTranslation(-distance_mm(100), 0),
        robotPose + Frame2D.fromTranslation(0, distance_mm(100)),
        robotPose + Frame2D.fromTranslation(0, -distance_mm(100)),
    ]
    

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

        
        map[(robotPose.x_mm, robotPose.y_mm)] = possible_map_positions

        robot.drive_straight(distance_mm(100), speed_mmps(50)).wait_for_completed()
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