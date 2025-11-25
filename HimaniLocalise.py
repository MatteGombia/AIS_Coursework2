#!/usr/bin/env python3

import math
import asyncio
from statistics import mean

import cozmo
from cozmo.util import degrees
from cozmo.objects import LightCube1Id, LightCube2Id, LightCube3Id


# ---------------------------
# Triangulation maths (using two cubes as landmarks)
# ---------------------------

def triangulate_from_two_cubes(c1, c2, d1, d2):
    """
    c1, c2: (x, y) coordinates of cube1 and cube2 in the map frame
    d1, d2: distances from robot to cube1 and cube2 (in same units as map, e.g. mm)

    Returns: two possible robot positions (x, y)
    """
    x1, y1 = c1
    x2, y2 = c2

    # Distance between cubes
    dx = x2 - x1
    dy = y2 - y1
    d = math.hypot(dx, dy)

    # Validity checks
    if d == 0:
        raise ValueError("Cube positions cannot be identical.")
    if d > d1 + d2:
        raise ValueError("No intersection: circles too far apart.")
    if d < abs(d1 - d2):
        raise ValueError("No intersection: one circle inside another.")

    # Distance from cube1 to midpoint line between intersections
    a = (d1**2 - d2**2 + d**2) / (2 * d)
    h = math.sqrt(abs(d1**2 - a**2))

    # Midpoint between intersections
    x3 = x1 + a * dx / d
    y3 = y1 + a * dy / d

    # Intersection points (robot has 2 possible positions)
    rx1 = x3 + h * (dy / d)
    ry1 = y3 - h * (dx / d)

    rx2 = x3 - h * (dy / d)
    ry2 = y3 + h * (dx / d)

    return (rx1, ry1), (rx2, ry2)


def normalize_angle(angle):
    """Wrap angle into the range [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def choose_correct_position(c1, c2, posA, posB, bearing1, bearing2):
    """
    bearing1, bearing2: measured bearings from robot to cubes (in radians)
    posA, posB: the two possible robot positions from triangulation
    c1, c2: cube positions in the map
    """

    def bearing_error(robot_pos):
        xr, yr = robot_pos

        # expected bearing to cube1
        pred_b1 = math.atan2(c1[1] - yr, c1[0] - xr)
        # expected bearing to cube2
        pred_b2 = math.atan2(c2[1] - yr, c2[0] - xr)

        err1 = normalize_angle(pred_b1 - bearing1)
        err2 = normalize_angle(pred_b2 - bearing2)
        return abs(err1) + abs(err2)

    errA = bearing_error(posA)
    errB = bearing_error(posB)

    return posA if errA < errB else posB


def localize_robot(c1, c2, d1, d2, b1, b2):
    """
    Use triangulation + bearings to estimate robot pose (x, y, heading).

    c1, c2: cube positions in the map (x, y)
    d1, d2: distances to cubes
    b1, b2: bearings to cubes (relative to robot heading)
    """
    posA, posB = triangulate_from_two_cubes(c1, c2, d1, d2)
    xr, yr = choose_correct_position(c1, c2, posA, posB, b1, b2)
    heading = (b1 + b2) / 2.0  # simple heading estimate
    return xr, yr, heading


# ---------------------------
# Sensor helpers for cubes
# ---------------------------

def get_cube_measurements(robot, cube):
    """Return (distance, bearing) from robot to a cube."""
    dx = cube.pose.position.x - robot.pose.position.x
    dy = cube.pose.position.y - robot.pose.position.y

    dist = math.hypot(dx, dy)
    global_bearing = math.atan2(dy, dx)
    robot_heading = robot.pose.rotation.angle_z.radians

    rel_bearing = normalize_angle(global_bearing - robot_heading)
    return dist, rel_bearing


async def average_measurements(robot, cube, n=5):
    """Take n measurements and return averaged (distance, bearing)."""
    dists = []
    bears = []
    for _ in range(n):
        d, b = get_cube_measurements(robot, cube)
        dists.append(d)
        bears.append(b)
        await asyncio.sleep(0.05)
    avg_d = mean(dists)
    avg_b = normalize_angle(mean(bears))
    return avg_d, avg_b


# ---------------------------
# Finding any two visible cubes
# ---------------------------

def find_two_visible_cubes(robot):
    """
    Check all 3 light cubes and return the first two that are visible.
    Returns (cube1, cube2) or (None, None) if fewer than 2 visible.
    """
    visible = []

    for cube_id in (LightCube1Id, LightCube2Id, LightCube3Id):
        cube = robot.world.get_light_cube(cube_id)
        if cube is not None and cube.is_visible:
            visible.append(cube)
        if len(visible) == 2:
            break

    if len(visible) >= 2:
        return visible[0], visible[1]
    else:
        return None, None


async def search_for_two_cubes(robot, step_deg=20, max_steps=18):
    """
    Rotate in small steps until at least two cubes are visible,
    or we reach max_steps. Returns (cube1, cube2) or (None, None).
    """
    for _ in range(max_steps):
        c1, c2 = find_two_visible_cubes(robot)
        if c1 is not None and c2 is not None:
            return c1, c2

        await robot.turn_in_place(degrees(step_deg)).wait_for_completed()
        await asyncio.sleep(0.05)

    return None, None


# ---------------------------
# Main Cozmo program
# ---------------------------

async def run(robot: cozmo.robot.Robot):

    # Head roughly level (async action → must await)
    await robot.set_head_angle(degrees(0)).wait_for_completed()

    # Make sure camera is on
    robot.camera.image_stream_enabled = True

    print("Searching for any two visible cubes...")
    cube1, cube2 = await search_for_two_cubes(robot, step_deg=20, max_steps=18)
    if cube1 is None or cube2 is None:
        print("Could not find two visible cubes, stopping.")
        return

    print("Found two cubes:")
    print("  Cube 1 id:", cube1.object_id)
    print("  Cube 2 id:", cube2.object_id)

    # Take multiple readings and average them to reduce noise
    d1, b1 = await average_measurements(robot, cube1, n=5)
    print("Cube 1 distance, bearing (avg):", d1, b1)

    d2, b2 = await average_measurements(robot, cube2, n=5)
    print("Cube 2 distance, bearing (avg):", d2, b2)

    # Known cube positions in your map (YOU must define these correctly)
    # Example values in mm (change these to match your map layout!)
    c1 = (0.0, 0.0)       # cube 1 position in map frame
    c2 = (300.0, 300.0)     # cube 2 position in map frame

    try:
        xr, yr, heading = localize_robot(c1, c2, d1, d2, b1, b2)
    except ValueError as e:
        print("Triangulation error:", e)
        return

    print("Triangulated robot position:", xr, yr)
    print("Triangulated robot heading (rad):", heading)

    print("Cozmo internal pose:",
          robot.pose.position.x,
          robot.pose.position.y,
          robot.pose.rotation.angle_z.radians)


if __name__ == "__main__":
    cozmo.run_program(run)
