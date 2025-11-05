import cozmo
from cozmo.util import distance_mm, speed_mmps, degrees
import time

def test_search(robot: cozmo.robot.Robot):
    found_cube = False
    search_steps = 8  #Amount of search cycle repetitions
    #Start looking around
    robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
    try:
        for _ in range(search_steps):
            #Cube values
            cubes = list(robot.world.visible_light_cubes.values())
            if cubes:
                cube = cubes[0]
                print(f"Cube found: {cube}")
                #Go to object and wait
                robot.go_to_object(cube, distance_mm(50.0)).wait_for_completed()
                print("Arrived at the cube!")
                found_cube = True
                break
            #If no cube found, move forward
            robot.drive_straight(distance_mm(100), speed_mmps(50)).wait_for_completed()
            time.sleep(0.5)
            #Turn left 90 degrees to scan new area
            robot.turn_in_place(degrees(90)).wait_for_completed()
            time.sleep(0.5)
        if not found_cube:
            print("Cube was not found.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        robot.stop_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
cozmo.run_program(test_search)
