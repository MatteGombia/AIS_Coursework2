#!/usr/bin/env python3

# Copyright (c) 2019 Matthias Rolf, Oxford Brookes University

'''

'''

import cozmo

from cozmo.util import degrees, Pose, distance_mm, speed_mmps

import numpy as np
import sys

from frame2d import Frame2D
from cozmo_interface import wheelDistance, target_pose_to_velocity_linear,velocity_to_track_speed,track_speed_to_pose_change
from cozmo.util import distance_mm, speed_mmps, degrees, Angle

import time
import math

tgtLocX = 200
tgtLocY = 200
tgtLocTheta = 0.0
tgtName = "Target"

# specifiying "Cube1", "Cube2" or "Cube3" as target selects it
# as the target to approach
if len(sys.argv) == 2:
   tgtLocX = 0
   tgtLocY = 0
   tgtName = sys.argv[1]

# 3 parameters specifies the target's (x, y, theta/pi) pose   
elif len(sys.argv) >= 4:
   tgtLocX = sys.argv[1]
   tgtLocY = sys.argv[2]
   tgtLocTheta = sys.argv[3]

# each cube is a dictionary indexed by ID and containing a pair [visible, relativePose] where
# relativePose is a Frame2D object that gives the cube's pose relative to the robot
cubes = {cozmo.objects.LightCube1Id: [False, None],
         cozmo.objects.LightCube2Id: [False, None],
         cozmo.objects.LightCube3Id: [False, None]}


currentPose=Frame2D()

targetPose=Frame2D.fromXYA(tgtLocX,tgtLocY,tgtLocTheta*math.pi)

interval = 0.1

# take sensor readings of the cubes
def updateCubes(robot, cubes):
   # read sensors
   robotPose = Frame2D.fromPose(robot.pose)
   for cubeID in cubes:
      cube = robot.world.get_light_cube(cubeID)

      if cube is not None and cube.is_visible:
         cubePose = Frame2D.fromPose(cube.pose)
         cubes[cube][1] = robotPose.inverse().mult(cubePose)
         cubes[cube][0] = True

# TODO - advanced - implement obstacle avoidance manoeuvre
def collision_avoidance(futurePose, cubes, curTrackSpeed):
    updatedTrackSpeed = curTrackSpeed
    for cube in cubes:
        # TODO have a sensible test for collision
         if futurePose == cubes[cube][1]:
            updatedTrackSpeed = [0,0]
    return updatedTrackSpeed
   
def test_search(robot: cozmo.robot.Robot):
   found_cube = False
   search_steps = 18  #Amount of search cycle repetitions
   #Start looking around
   #robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
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
    global currentPose
    global cubes
    robot.set_head_angle(Angle(degrees=0)).wait_for_completed
    time.sleep(0.5)
    while True:
        if tgtName == "Cube1":
           relativeTarget = cubes[cozmo.objects.LightCube1Id][1]
        elif tgtName == "Cube2":
           relativeTarget = cubes[cozmo.objects.LightCube2Id][1]
        elif tgtName == "Cube3":
           relativeTarget = cubes[cozmo.objects.LightCube3Id][1]
        else:
           relativeTarget = test_search(robot) # TODO determine current position of target relative to robot
        if relativeTarget is None:
           print(cubes) # TODO - advanced - more sophisticated exploration than spinning in place
        # print("relativeTarget"+str(relativeTarget))
        velocity = target_pose_to_velocity_linear(relativeTarget)
        # print("velocity"+str(velocity)) 
        trackSpeed = velocity_to_track_speed(velocity[0],velocity[1])
        # print("trackSpeedCommand"+str(trackSpeed))
        lspeed = robot.left_wheel_speed.speed_mmps
        rspeed = robot.right_wheel_speed.speed_mmps
        # print("trackSpeed"+str([lspeed, rspeed]))
        delta = track_speed_to_pose_change(lspeed, rspeed,interval)
        currentPose = Frame2D() # TODO update robot frame with delta
        nextDelta = track_speed_to_pose_change(trackSpeed[0], trackSpeed[1], interval)
        futurePose = Frame2D() # TODO update next frame with predicted delta
        print("currentPose"+str(currentPose))
        print("futurePose"+str(futurePose))
        # get a new cube measurement
        updateCubes(robot, cubes)
        trackSpeed = collision_avoidance(futurePose, cubes, trackSpeed)
        robot.drive_wheel_motors(l_wheel_speed=trackSpeed[0],r_wheel_speed=trackSpeed[1])
        time.sleep(interval)


cozmo.run_program(cozmo_drive_to_target)

