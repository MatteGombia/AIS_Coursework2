import cozmo
import math
import time
from statistics import mean

from cozmo.util import degrees, Angle
from cozmo.objects import CustomObject, CustomObjectMarkers, CustomObjectTypes

walls = []
marked_walls_seen = []
walls_angles = []


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
    
    print("Creating custom walls (synchronous method)...")
    
    for i in range(0, 8):
        wall = robot.world.define_custom_wall(types[i], markers[i], 200, 60, 50, 50, True)
        cozmo_walls.append(wall)
        print(f"  Defined wall {i+1}: {types[i]}")
    
    for i in range(8, 16):
        wall = robot.world.define_custom_wall(types[i], markers[i], 300, 60, 50, 50, True)
        cozmo_walls.append(wall)
        print(f"  Defined wall {i+1}: {types[i]}")
    
    print(f"Total walls defined: {len(cozmo_walls)}")
    return cozmo_walls


def add_wall(wall_x, wall_y):
    walls.append((wall_x, wall_y))


def handle_object_observed(evt, **kw):
    global walls, marked_walls_seen, walls_angles
    
    print(f"[OBJECT SEEN] Type: {type(evt.obj).__name__}", end="")
    if hasattr(evt.obj, 'object_type'):
        print(f" - {evt.obj.object_type}")
    else:
        print()
    
    if isinstance(evt.obj, CustomObject):
        if evt.obj not in marked_walls_seen:
            marked_walls_seen.append(evt.obj)
            print(f"*** WALL DETECTED at {evt.obj.pose.position} ***")
            print(f"    Type: {evt.obj.object_type}")
            print(f"    Full object: {evt.obj}")
            add_wall(evt.obj.pose.position.x, evt.obj.pose.position.y)
            walls_angles.append(evt.obj.pose.rotation.angle_z.radians + math.pi / 2)


def normalise_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def triangulate_from_two_markers(c1, c2, d1, d2):
    x1, y1 = c1
    x2, y2 = c2

    dx = x2 - x1
    dy = y2 - y1
    d = math.hypot(dx, dy)

    if d == 0:
        raise ValueError("Marker positions cannot be identical.")
    if d > d1 + d2:
        raise ValueError("No intersection: circles too far apart.")
    if d < abs(d1 - d2):
        raise ValueError("No intersection: one circle inside another.")

    a = (d1**2 - d2**2 + d**2) / (2 * d)
    h = math.sqrt(abs(d1**2 - a**2))

    x3 = x1 + a * dx / d
    y3 = y1 + a * dy / d

    rx1 = x3 + h * (dy / d)
    ry1 = y3 - h * (dx / d)

    rx2 = x3 - h * (dy / d)
    ry2 = y3 + h * (dx / d)

    return (rx1, ry1), (rx2, ry2)


def choose_correct_position(c1, c2, posA, posB, bearing1, bearing2):
    def bearing_error(robot_pos):
        xr, yr = robot_pos
        pred_b1 = math.atan2(c1[1] - yr, c1[0] - xr)
        pred_b2 = math.atan2(c2[1] - yr, c2[0] - xr)

        err1 = normalise_angle(pred_b1 - bearing1)
        err2 = normalise_angle(pred_b2 - bearing2)
        return abs(err1) + abs(err2)

    errA = bearing_error(posA)
    errB = bearing_error(posB)

    return posA if errA < errB else posB


def localise_robot(c1, c2, d1, d2, b1, b2):
    posA, posB = triangulate_from_two_markers(c1, c2, d1, d2)
    xr, yr = choose_correct_position(c1, c2, posA, posB, b1, b2)
    heading = (b1 + b2) / 2.0
    return xr, yr, heading


def get_marker_measurements(robot, marker_obj):
    dx = marker_obj.pose.position.x - robot.pose.position.x
    dy = marker_obj.pose.position.y - robot.pose.position.y

    dist = math.hypot(dx, dy)
    global_bearing = math.atan2(dy, dx)
    robot_heading = robot.pose.rotation.angle_z.radians

    rel_bearing = normalise_angle(global_bearing - robot_heading)
    return dist, rel_bearing


def average_measurements(robot, marker_obj, n=5):
    dists = []
    bears = []
    for _ in range(n):
        d, b = get_marker_measurements(robot, marker_obj)
        dists.append(d)
        bears.append(b)
        time.sleep(0.05)
    avg_d = mean(dists)
    avg_b = normalise_angle(mean(bears))
    return avg_d, avg_b


def scan_for_walls(robot: cozmo.robot.Robot, num_needed=2):
    global marked_walls_seen, walls, walls_angles
    
    marked_walls_seen.clear()
    walls.clear()
    walls_angles.clear()
    
    steps = 18
    step_angle = 20
    
    print("\n" + "="*70)
    print("SCANNING FOR WALLS")
    print("="*70)
    print(f"Looking for {num_needed} wall markers...")
    print("Performing 360° scan...\n")
    
    for step in range(steps):
        if len(marked_walls_seen) >= num_needed:
            print(f"\n✓ Found {len(marked_walls_seen)} walls - stopping scan early")
            break
        
        print(f"Step {step+1}/{steps} - Walls found: {len(marked_walls_seen)}")
        
        if step < steps - 1:
            robot.turn_in_place(degrees(step_angle)).wait_for_completed()
            time.sleep(0.3)
    
    print("\n" + "="*70)
    print(f"SCAN COMPLETE - Found {len(marked_walls_seen)} walls")
    print("="*70 + "\n")
    
    return len(marked_walls_seen) >= num_needed


def main(robot: cozmo.robot.Robot):
    print("\n" + "="*70)
    print("COZMO TRIANGULATION LOCALISATION")
    print("Using wall detection method from working exploration code")
    print("="*70 + "\n")
    
    robot.camera.image_stream_enabled = True
    print("✓ Camera enabled")
    
    create_cozmo_walls(robot)
    
    robot.add_event_handler(cozmo.objects.EvtObjectObserved, handle_object_observed)
    print("✓ Event handler registered")
    
    print("✓ Waiting for initialisation...")
    time.sleep(1)
    
    robot.set_head_angle(Angle(0)).wait_for_completed()
    print("✓ Head angle set")
    
    time.sleep(1)
    
    success = scan_for_walls(robot, num_needed=2)
    
    if not success or len(marked_walls_seen) < 2:
        print("\n" + "="*70)
        print("ERROR: Could not find 2 walls")
        print("="*70)
        print(f"Walls found: {len(marked_walls_seen)}")
        if len(marked_walls_seen) > 0:
            print("Detected walls:")
            for i, wall in enumerate(marked_walls_seen):
                print(f"  {i+1}. {wall.object_type} at {wall.pose.position}")
        return
    
    print("\n" + "="*70)
    print("PERFORMING TRIANGULATION")
    print("="*70)
    
    marker1 = marked_walls_seen[0]
    marker2 = marked_walls_seen[1]
    c1 = walls[0]
    c2 = walls[1]
    
    print(f"\nUsing walls:")
    print(f"  Wall 1: {marker1.object_type} at ({c1[0]:.1f}, {c1[1]:.1f})")
    print(f"  Wall 2: {marker2.object_type} at ({c2[0]:.1f}, {c2[1]:.1f})")
    
    print("\nMeasuring distances and bearings...")
    d1, b1 = average_measurements(robot, marker1, n=5)
    d2, b2 = average_measurements(robot, marker2, n=5)
    
    print(f"  Wall 1: dist={d1:.1f}mm, bearing={math.degrees(b1):.1f}°")
    print(f"  Wall 2: dist={d2:.1f}mm, bearing={math.degrees(b2):.1f}°")
    
    try:
        xr, yr, heading = localise_robot(c1, c2, d1, d2, b1, b2)
    except ValueError as e:
        print(f"\n❌ Triangulation error: {e}")
        return
    
    print("\n" + "="*70)
    print("TRIANGULATION RESULTS")
    print("="*70)
    print(f"Robot Position: ({xr:.1f}, {yr:.1f}) mm")
    print(f"Robot Heading: {heading:.3f} rad ({math.degrees(heading):.1f}°)")
    print("="*70)
    
    print("\n" + "="*70)
    print("COZMO INTERNAL POSE (comparison)")
    print("="*70)
    print(f"Internal Position: ({robot.pose.position.x:.1f}, {robot.pose.position.y:.1f}) mm")
    print(f"Internal Heading: {robot.pose.rotation.angle_z.radians:.3f} rad ({robot.pose.rotation.angle_z.degrees:.1f}°)")
    print("="*70 + "\n")


if __name__ == "__main__":
    cozmo.run_program(main)
