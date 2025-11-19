import cozmo
from cozmo.util import distance_mm, speed_mmps, degrees

def cozmo_program(robot: cozmo.robot.Robot):
    while True:
        front_obstacles = robot.world.visible_objects
        obstacle_close = False
        
        for obj in front_obstacles:
            if obj.pose.position.x > 0 and obj.pose.position.x < 150:
                obstacle_close = True
                break
        
        if obstacle_close:
            robot.drive_wheels(0, 0)
            robot.turn_in_place(degrees(90)).wait_for_completed()
        else:
            robot.drive_wheels(50, 50)
        
        robot.sleep(0.1)

cozmo.run_program(cozmo_program)
