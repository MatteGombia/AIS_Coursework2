import cozmo
<<<<<<< HEAD
from cozmo.util import distance_mm, speed_mmps, degrees, Angle
import time
from frame2d import Frame2D 
import asyncio
import sys
import cozmo

def test_search(robot: cozmo.robot.Robot):
    found_cube = False
    search_steps = 80  #Amount of search cycle repetitions
    #Start looking around
    #robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
    robot.set_head_angle(Angle(degrees=0)).wait_for_completed
    time.sleep(0.1)
    try:
        for _ in range(search_steps):
            #Cube values
            #cubes = list(robot.world.visible_light_cubes.values())
            # if cubes:
            #     cube = cubes[0]
            #     print(f"Cube found: {cube}")
            #     #Go to object and wait
            #     robot.go_to_object(cube, distance_mm(50.0)).wait_for_completed()
            #     print("Arrived at the cube!")
            #     found_cube = True
            #     break




            cubeIDs = (cozmo.objects.LightCube1Id,cozmo.objects.LightCube2Id,cozmo.objects.LightCube3Id)
            for cubeID in cubeIDs: 
                cube = robot.world.get_light_cube(cubeID)
                if cube is not None and cube.is_visible:
                    cubePose2D = Frame2D.fromPose(cube.pose)
                    print("   2D frame: " + str(cubePose2D))
                    robot.go_to_object(cube, distance_mm(50.0)).wait_for_completed()
                    found_cube = True


            time.sleep(0.1)

            print("stay in the loop, state %d", found_cube)




            #If no cube found, move forward
            #robot.drive_straight(distance_mm(100), speed_mmps(50)).wait_for_completed()
            time.sleep(0.2)
            #Turn left 90 degrees to scan new area
            robot.turn_in_place(degrees(20)).wait_for_completed()
            time.sleep(0.2)
=======
from cozmo.util import distance_mm, speed_mmps, degrees

def test_search(robot: cozmo.robot.Robot):
    found_cube = False
    search_steps = 8 #Amount of search cycle repetitions
    robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
    try:
        for _ in range(search_steps):
            cubes = list(robot.world.visible_light_cubes.values())
            if cubes:
                cube = cubes[0]
                print("Cube found:", cube)
                robot.go_to_object(cube, distance_mm(50)).wait_for_completed()
                print("Arrived at the cube!")
                found_cube = True
                break
            else:
                robot.drive_straight(distance_mm(100), speed_mmps(50)).wait_for_completed()
                robot.turn_in_place(degrees(90)).wait_for_completed()
>>>>>>> 51f6e5eaa8ee002cd5f3b0f5dc1641aa4fbf89e3
        if not found_cube:
            print("Cube was not found.")
    except Exception as e:
        print("Error:", e)
    finally:
        #robot.stop_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
        pass
cozmo.run_program(test_search)
