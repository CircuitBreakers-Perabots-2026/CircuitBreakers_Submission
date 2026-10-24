"""Section 1: autonomous obstacle-field navigation.

Strategy
--------
* Dead-reckon the robot pose (encoders for distance, IMU for heading).
* Build an occupancy grid from the front/left/right distance sensors.
* Plan with A* over the grid (unknown space is assumed free, so the plan
  is repeatedly corrected as new cubes are discovered).
* Follow the shortest, string-pulled path with a heading controller.
* Safety layer: back up when too close to something, detect being stuck,
  and spin-scan when no route is currently known.
* Finish when the floor camera sees RED (the transition zone) after the
  robot has left the red start zone.

Coordinates ("start frame"): origin = robot start position,
+x = initial heading (forward), +y = initial left. Angles in radians,
positive = counter-clockwise (same as turning.py).
"""

import heapq
import math

from floor_detection import read_floor
from turning import angle_difference


# ======================================================================
# CONFIGURATION -- adjust these to match your world file
# ======================================================================

# Arena rectangle in the start frame (metres). Section 1 is 2.44 x 1.22 m
# and the robot starts at the centre of a 0.25 x 0.25 m corner zone, so the
# arena extends 0.125 m behind / beside the robot.
# Default: the robot faces along the LONG side with the arena on its LEFT.
# If the arena is on its right, use ARENA_Y = (-1.095, 0.125) and make the
# GOAL y negative.
ARENA_X = (-0.125, 2.315)
ARENA_Y = (-0.125, 1.095)

# Approximate centre of the red transition zone (start frame). Used only
# to steer; the real stop condition is the camera seeing red.
GOAL = (2.19, 0.97)
GOAL_TOLERANCE = 0.12

# If the estimated goal is reached but no red floor has been seen (the
# goal estimate is a little off, or odometry drifted), the robot searches
# the area around GOAL instead of giving up.
SEARCH_RADIUS = 0.6          # how far around GOAL to look (m)
SEARCH_SPACING = 0.10        # spacing of the search points (m)
SEARCH_TOLERANCE = 0.08      # how close counts as "visited" (m)

# Robot geometry / sensors
WHEEL_RADIUS = 0.02          # same value as movement.py
ROBOT_RADIUS = 0.06          # half the robot's width, rounded up
SENSOR_MOUNT = 0.04          # distance from robot centre to each sensor

# Speeds (rad/s of the wheels; clamped to the motors' maximum)
CRUISE_SPEED = 1.4
TURN_SPEED = 0.8
BACKUP_SPEED = 0.5
WHEEL_ACCEL = 5.0            # rad/s^2 limit on command changes

# Safety thresholds (sensor readings, metres)
FRONT_STOP = 0.06
FRONT_SLOW = 0.20
SIDE_CAUTION = 0.05

MAX_TILT = math.radians(10)
MAX_RUN_TIME = 1800.0

# Optional manual override of "maze north" (start-frame radians, +ve = CCW).
# Leave as None: the robot finds the corridor walls beside the red cell and
# works out the maze direction itself. (The robot's start heading is NOT
# aligned with the arena in this world, so a start-frame constant is fragile.)
FINAL_HEADING = None

# Distance from the FAR edge of the red cell to the centre of the first maze
# cell (maze cells are 0.25 m, so half a cell). This is the one number to tune
# if the robot parks a little too far in / short of the first cell.
FIRST_CELL_OFFSET = 0.125

# Side beams closer than this count as "a corridor wall is there".
CORRIDOR_SIDE_MAX = 0.20
# Length of the straight creep used to measure heading error from the walls.
SLOPE_SPAN = 0.15

# Corridor-wall fitting at the red cell
CORRIDOR_WIDTH = 0.25        # gap between the two maze walls beside the red cell
CORRIDOR_TOLERANCE = 0.06
CORRIDOR_RANGE = 0.45        # only beams shorter than this are used
CORRIDOR_MIN_SCORE = 60      # minimum wall points for a trusted fit
SCAN_SPIN_SPEED = 0.6
HOUGH_BIN = 0.01
HOUGH_BINS = 100             # covers +-0.5 m either side of the robot

# Side length of the red cell (the maze cells are 0.25 m). Only used when
# a wall bounds the cell, so the robot can keep half a cell from that wall.
RED_CELL_SIZE = 0.25
CENTERING_SPEED = 0.6        # slow creep used while centring on the red cell
CENTERING_LIMIT = 0.5        # never creep further than this from the entry point

# Red-zone detection
MIN_TRAVEL_BEFORE_RED = 0.6  # ignore the red START zone
RED_CONFIRM_READS = 3

# ======================================================================
# Occupancy grid
# ======================================================================

RES = 0.025                  # grid resolution (m)
LEVEL_RADII = (ROBOT_RADIUS + 0.03, ROBOT_RADIUS, 0.0)  # safe -> tight
RADII_CELLS = tuple(int(math.ceil(r / RES - 1e-9)) for r in LEVEL_RADII)

L_HIT = 0.8
L_FREE = -0.15
L_OCC = 0.5
L_MIN, L_MAX = -2.0, 6.0

HIT_MAX_READING = 0.90       # readings above this mean "nothing seen"
MAP_RANGE = 0.60             # how far a beam is trusted to clear free space
CUBE_DEPTH_CELLS = 3         # cubes are >= 10 cm deep behind the hit face

EDGE_MARGIN = 0.06           # planner keeps this far from arena walls
I_MIN = int(math.ceil((ARENA_X[0] + EDGE_MARGIN) / RES))
I_MAX = int(math.floor((ARENA_X[1] - EDGE_MARGIN) / RES))
J_MIN = int(math.ceil((ARENA_Y[0] + EDGE_MARGIN) / RES))
J_MAX = int(math.floor((ARENA_Y[1] - EDGE_MARGIN) / RES))

BEAMS = (("front", 0.0), ("left", math.pi / 2), ("right", -math.pi / 2))


def cell_of(x, y):
    return (math.floor(x / RES), math.floor(y / RES))


def center_of(cell):
    return ((cell[0] + 0.5) * RES, (cell[1] + 0.5) * RES)


def disk_offsets(radius):
    return [
        (i, j)
        for i in range(-radius, radius + 1)
        for j in range(-radius, radius + 1)
        if i * i + j * j <= radius * radius
    ]


class OccupancyGrid:
    def __init__(self):
        self.logodds = {}
        self.occupied = set()
        self.counts = [dict() for _ in RADII_CELLS]
        self.offsets = [disk_offsets(r) for r in RADII_CELLS]
        self.version = 0

    def blocked(self, cell, level):
        return self.counts[level].get(cell, 0) > 0

    def _inflate(self, cell, sign):
        for counts, offsets in zip(self.counts, self.offsets):
            for di, dj in offsets:
                key = (cell[0] + di, cell[1] + dj)
                value = counts.get(key, 0) + sign
                if value <= 0:
                    counts.pop(key, None)
                else:
                    counts[key] = value

    def _apply(self, cell, delta):
        new = max(L_MIN, min(L_MAX, self.logodds.get(cell, 0.0) + delta))
        self.logodds[cell] = new
        was = cell in self.occupied
        now = new > L_OCC
        if now and not was:
            self.occupied.add(cell)
            self._inflate(cell, +1)
            self.version += 1
        elif was and not now:
            self.occupied.discard(cell)
            self._inflate(cell, -1)
            self.version += 1

    def force_occupied(self, x, y):
        cell = cell_of(x, y)
        need = L_OCC + 0.3 - self.logodds.get(cell, 0.0)
        if need > 0:
            self._apply(cell, need)

    def integrate(self, px, py, yaw, beam_angle, reading):
        """Add one distance-sensor beam to the map."""
        angle = yaw + beam_angle
        dx, dy = math.cos(angle), math.sin(angle)
        hit = reading < HIT_MAX_READING
        end = SENSOR_MOUNT + reading

        free_end = min(end - RES, SENSOR_MOUNT + MAP_RANGE)
        seen = set()
        d = 0.0
        while d <= free_end:
            cell = cell_of(px + dx * d, py + dy * d)
            if cell not in seen:
                seen.add(cell)
                self._apply(cell, L_FREE)
            d += RES * 0.5

        if hit:
            self._apply(cell_of(px + dx * end, py + dy * end), L_HIT)
            for k in range(1, CUBE_DEPTH_CELLS + 1):
                cell = cell_of(
                    px + dx * (end + k * RES), py + dy * (end + k * RES)
                )
                need = L_OCC + 0.1 - self.logodds.get(cell, 0.0)
                if need > 0:
                    self._apply(cell, need)


# ======================================================================
# Planning
# ======================================================================

NEIGHBOURS = (
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414),
)


def in_bounds(cell):
    return I_MIN <= cell[0] <= I_MAX and J_MIN <= cell[1] <= J_MAX


def line_free(grid, a, b, level):
    distance = math.hypot(b[0] - a[0], b[1] - a[1])
    steps = max(1, int(distance / (RES * 0.5)))
    for k in range(steps + 1):
        t = k / steps
        cell = cell_of(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
        if not in_bounds(cell) or grid.blocked(cell, level):
            return False
    return True


def plan(grid, start_xy, goal_xy, level, tolerance=GOAL_TOLERANCE):
    """A* from start to goal; returns a list of (x, y) or None."""
    start = cell_of(*start_xy)

    def walkable(cell):
        return in_bounds(cell) and not grid.blocked(cell, level)

    def heuristic(cell):
        x, y = center_of(cell)
        return math.hypot(x - goal_xy[0], y - goal_xy[1]) / RES

    heap = [(heuristic(start), 0.0, start)]
    best = {start: 0.0}
    parent = {start: None}
    closed = set()

    while heap:
        _, cost, cell = heapq.heappop(heap)
        if cell in closed:
            continue
        closed.add(cell)

        x, y = center_of(cell)
        if math.hypot(x - goal_xy[0], y - goal_xy[1]) <= tolerance:
            cells = []
            while cell is not None:
                cells.append(cell)
                cell = parent[cell]
            cells.reverse()
            points = [center_of(c) for c in cells]
            points[0] = tuple(start_xy)
            return points

        for di, dj, step in NEIGHBOURS:
            nxt = (cell[0] + di, cell[1] + dj)
            if nxt in closed or not walkable(nxt):
                continue
            if di and dj and not (
                walkable((cell[0] + di, cell[1]))
                and walkable((cell[0], cell[1] + dj))
            ):
                continue  # no corner cutting
            new_cost = cost + step
            if new_cost < best.get(nxt, float("inf")):
                best[nxt] = new_cost
                parent[nxt] = cell
                heapq.heappush(heap, (new_cost + heuristic(nxt), new_cost, nxt))

    return None


def smooth(points, grid, level, window=60):
    """Remove unnecessary waypoints (line-of-sight shortcuts)."""
    if len(points) <= 2:
        return points
    result = [points[0]]
    i = 0
    while i < len(points) - 1:
        j = min(len(points) - 1, i + window)
        while j > i + 1 and not line_free(grid, points[i], points[j], level):
            j -= 1
        result.append(points[j])
        i = j
    return result


def path_clear(grid, position, waypoints, level):
    last = position
    for waypoint in waypoints:
        if not line_free(grid, last, waypoint, level):
            return False
        last = waypoint
    return True


def find_route(grid, position, goal, tolerance, only_better_than=None):
    """Try the safest clearance first, then progressively tighter ones."""
    levels = range(3) if only_better_than is None else range(only_better_than)
    for level in levels:
        raw = plan(grid, position, goal, level, tolerance)
        if raw:
            return smooth(raw, grid, level), level
    return None, None


def build_search_points(grid, position):
    """Points around GOAL to visit, ordered nearest-neighbour first."""
    points = []
    steps = int(SEARCH_RADIUS / SEARCH_SPACING)
    for i in range(-steps, steps + 1):
        for j in range(-steps, steps + 1):
            px = GOAL[0] + i * SEARCH_SPACING
            py = GOAL[1] + j * SEARCH_SPACING
            if math.hypot(px - GOAL[0], py - GOAL[1]) > SEARCH_RADIUS:
                continue
            if not (ARENA_X[0] + 0.08 <= px <= ARENA_X[1] - 0.08
                    and ARENA_Y[0] + 0.08 <= py <= ARENA_Y[1] - 0.08):
                continue
            if grid.blocked(cell_of(px, py), 1):
                continue
            points.append((px, py))

    ordered = []
    here = position
    while points:
        nearest = min(points, key=lambda p: math.hypot(p[0] - here[0],
                                                       p[1] - here[1]))
        points.remove(nearest)
        ordered.append(nearest)
        here = nearest
    return ordered


# ======================================================================
# Corridor-wall fitting
# ======================================================================

def parallel_wall_score(points, phi):
    """Best score for two parallel walls ~CORRIDOR_WIDTH apart along phi.

    points are (x, y) hits relative to the robot centre. The walls must
    straddle the robot (it is standing between them).
    """
    sin_phi, cos_phi = math.sin(phi), math.cos(phi)
    half = HOUGH_BINS // 2
    hist = [0] * (HOUGH_BINS + 2)
    for px, py in points:
        index = int(math.floor((-px * sin_phi + py * cos_phi) / HOUGH_BIN)) + half
        if 0 <= index < HOUGH_BINS:
            hist[index + 1] += 1
    smooth = [hist[i] + hist[i + 1] + hist[i + 2] for i in range(HOUGH_BINS)]

    low = int(round((CORRIDOR_WIDTH - CORRIDOR_TOLERANCE) / HOUGH_BIN))
    high = int(round((CORRIDOR_WIDTH + CORRIDOR_TOLERANCE) / HOUGH_BIN))
    best = 0
    for i in range(HOUGH_BINS):
        first = smooth[i]
        if first < 15:
            continue
        if (i - half + 0.5) * HOUGH_BIN > 0.08:
            break
        for gap in range(low, high + 1):
            j = i + gap
            if j >= HOUGH_BINS:
                break
            second = smooth[j]
            if second < 15 or (j - half + 0.5) * HOUGH_BIN < -0.08:
                continue
            best = max(best, first + second)
    return best


def fit_corridor_axis(points):
    """Direction (radians, [0, pi)) of the corridor walls, or None."""
    if len(points) < 60:
        return None
    best_score, best_deg = 0, None
    for deg in range(180):
        score = parallel_wall_score(points, math.radians(deg))
        if score > best_score:
            best_score, best_deg = score, deg
    if best_deg is None or best_score < CORRIDOR_MIN_SCORE:
        return None
    fine = [
        (parallel_wall_score(points, math.radians(best_deg + k * 0.1)),
         best_deg + k * 0.1)
        for k in range(-30, 31)
    ]
    top = max(score for score, _ in fine)
    good = [deg for score, deg in fine if score >= 0.97 * top]
    return math.radians(sum(good) / len(good)) % math.pi


# ======================================================================
# Debug helper
# ======================================================================

def render_map(grid, pose):
    print("MAP (# obstacle, . free, space unknown, R robot, G goal):")
    robot_cell = cell_of(pose[0], pose[1])
    goal_cell = cell_of(*GOAL)
    for j in range(J_MAX, J_MIN - 1, -1):
        row = []
        for i in range(I_MIN, I_MAX + 1):
            cell = (i, j)
            if cell == robot_cell:
                row.append("R")
            elif cell == goal_cell:
                row.append("G")
            elif cell in grid.occupied:
                row.append("#")
            elif grid.logodds.get(cell, 0.0) < 0:
                row.append(".")
            else:
                row.append(" ")
        print("".join(row))


# ======================================================================
# Main controller
# ======================================================================

def get_device(robot, name):
    """robot.getDevice() that explains itself when a device is missing."""
    device = robot.getDevice(name)
    if device is None:
        available = sorted(
            robot.getDeviceByIndex(i).getName()
            for i in range(robot.getNumberOfDevices())
        )
        raise RuntimeError(
            f"Device '{name}' not found on this robot. "
            f"Devices available: {available}"
        )
    return device


def run_obstacle_field(robot, time_step=16):
    """Drive from the start zone to the red transition zone.

    Returns True when the red transition zone was reached.
    """
    dt = time_step / 1000.0

    left_motor = get_device(robot, "left_wheel_motor")
    right_motor = get_device(robot, "right_wheel_motor")
    for motor in (left_motor, right_motor):
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)

    max_speed = min(left_motor.getMaxVelocity(), right_motor.getMaxVelocity())
    if max_speed <= 0:
        max_speed = 10.0
    cruise = min(CRUISE_SPEED, max_speed)
    turn_speed = min(TURN_SPEED, max_speed)
    backup_speed = min(BACKUP_SPEED, max_speed)

    get_device(robot, "led_blue").set(0)
    get_device(robot, "led_green").set(0)

    sensors = {
        "front": get_device(robot, "front_distance"),
        "left": get_device(robot, "left_distance"),
        "right": get_device(robot, "right_distance"),
    }
    for sensor in sensors.values():
        sensor.enable(time_step)

    left_encoder = get_device(robot, "left_wheel_sensor")
    right_encoder = get_device(robot, "right_wheel_sensor")
    left_encoder.enable(time_step)
    right_encoder.enable(time_step)

    imu = get_device(robot, "imu")
    imu.enable(time_step)

    camera = get_device(robot, "floor_camera")
    camera.enable(time_step)

    grid = OccupancyGrid()

    x = y = 0.0
    yaw = 0.0
    yaw0 = None
    previous_left = previous_right = None
    travelled = 0.0

    state = "SETTLING"
    state_started = robot.getTime()
    start_time = state_started

    path = []
    path_index = 0
    path_level = 0
    last_plan_time = -99.0
    checked_version = -1
    force_replan = True
    rotating = False

    command_left = command_right = 0.0

    red_reads = 0
    target = GOAL
    target_tolerance = GOAL_TOLERANCE
    searching = False
    search_queue = []
    final_ok = 0
    reached = False
    plan_failures = 0
    backups = 0
    last_backup_time = -99.0
    backup_start_distance = 0.0
    scan_total = 0.0
    scan_previous = 0.0
    stuck_ref = (0.0, 0.0, 0.0, state_started)
    step_count = 0
    last_report = -1.0
    goal_distance = math.hypot(GOAL[0], GOAL[1])
    front = left_d = right_d = 1.0
    backup_origin = (0.0, 0.0)
    backup_started = 0.0

    def next_search_target():
        """Pick the next unvisited search point. False when none remain."""
        nonlocal target, target_tolerance, path, force_replan
        while search_queue:
            candidate = search_queue.pop(0)
            if math.hypot(candidate[0] - x, candidate[1] - y) > (
                SEARCH_TOLERANCE
            ):
                target = candidate
                target_tolerance = SEARCH_TOLERANCE
                path = []
                force_replan = True
                return True
        return False

    # ------------------------------------------------------------------
    # Blocking helpers used only for parking on the red cell.
    # ------------------------------------------------------------------
    def set_wheels(left, right):
        nonlocal command_left, command_right
        command_left = max(-max_speed, min(max_speed, left))
        command_right = max(-max_speed, min(max_speed, right))
        left_motor.setVelocity(command_left)
        right_motor.setVelocity(command_right)

    def sense_step():
        """Advance one step, update odometry, return the front reading."""
        nonlocal x, y, yaw, previous_left, previous_right, travelled
        if robot.step(time_step) == -1:
            raise RuntimeError("Simulation ended")
        _, _, yaw_abs_now = imu.getRollPitchYaw()
        yaw = angle_difference(yaw_abs_now, yaw0)
        left_now = left_encoder.getValue()
        right_now = right_encoder.getValue()
        moved = (
            (left_now - previous_left) + (right_now - previous_right)
        ) / 2.0 * WHEEL_RADIUS
        previous_left, previous_right = left_now, right_now
        x += moved * math.cos(yaw)
        y += moved * math.sin(yaw)
        travelled += abs(moved)
        return sensors["front"].getValue()

    def hold_still(seconds):
        set_wheels(0.0, 0.0)
        for _ in range(int(seconds / dt)):
            sense_step()

    def turn_to(heading, tolerance_deg=2.0):
        set_wheels(0.0, 0.0)
        started = robot.getTime()
        settled = 0
        while settled < 15:
            sense_step()
            if robot.getTime() - started > 45.0:
                raise RuntimeError("Timed out turning on the red cell")
            err = angle_difference(heading, yaw)
            if abs(err) <= math.radians(tolerance_deg):
                settled += 1
                set_wheels(0.0, 0.0)
            else:
                settled = 0
                t = max(-turn_speed, min(turn_speed, 1.2 * err))
                if abs(t) < 0.08:
                    t = math.copysign(0.08, err)
                set_wheels(-t, t)
        set_wheels(0.0, 0.0)

    def along(origin, heading):
        return (
            (x - origin[0]) * math.cos(heading)
            + (y - origin[1]) * math.sin(heading)
        )

    def go_to(origin, u, face, sign, target_s, front_limit=None):
        """Drive (forward or reverse) until the u-position equals target_s."""
        speed = min(CENTERING_SPEED, max_speed)
        settled = 0
        started = robot.getTime()
        while settled < 10:
            front = sense_step()
            if robot.getTime() - started > 90.0:
                raise RuntimeError("Timed out centring on the red cell")
            if front_limit is not None and sign > 0 and front < front_limit:
                break
            err = target_s - along(origin, u)
            if abs(err) < 0.004:
                settled += 1
                set_wheels(0.0, 0.0)
            else:
                settled = 0
                v = sign * max(-speed, min(speed, 4.0 * err))
                if abs(v) < 0.1:
                    v = math.copysign(0.1, v)
                fix = max(-0.1, min(0.1, 1.5 * angle_difference(face, yaw)))
                set_wheels(v - fix, v + fix)
        set_wheels(0.0, 0.0)

    def scan_corridor_axis():
        """Spin on the spot and fit the two maze walls beside the red cell.

        Returns the wall direction in [0, pi) (start frame) or None.
        """
        set_wheels(0.0, 0.0)
        spin = min(SCAN_SPIN_SPEED, max_speed)
        x0, y0 = x, y
        points = []
        swept = 0.0
        previous = yaw
        started = robot.getTime()
        count = 0
        # Front + left + right beams cover 360 degrees after ~200 degrees.
        while swept < math.pi + 0.35:
            sense_step()
            if robot.getTime() - started > 90.0:
                raise RuntimeError("Timed out scanning the red-cell walls")
            swept += abs(angle_difference(yaw, previous))
            previous = yaw
            set_wheels(-spin, spin)
            count += 1
            if count % 2:
                continue
            for name, beam in BEAMS:
                reading = sensors[name].getValue()
                if reading < CORRIDOR_RANGE:
                    reach = SENSOR_MOUNT + reading
                    angle = yaw + beam
                    points.append((
                        x - x0 + reach * math.cos(angle),
                        y - y0 + reach * math.sin(angle),
                    ))
        set_wheels(0.0, 0.0)
        return fit_corridor_axis(points)

    def choose_north(axis_angle, fallback):
        """Pick which way along the corridor leads into the maze.

        The robot travelled from the start towards the maze, so maze north is
        the corridor direction with positive progress from the start (0, 0).
        """
        progress = x * math.cos(axis_angle) + y * math.sin(axis_angle)
        if abs(progress) >= 0.15:
            heading = axis_angle if progress > 0 else axis_angle + math.pi
        else:
            heading = min(
                (axis_angle, axis_angle + math.pi),
                key=lambda c: abs(angle_difference(c, fallback)),
            )
        return angle_difference(heading, 0.0)

    def drive_along(origin, heading, limit, stop_test=None):
        """Creep forward holding `heading` until `limit` m, a wall ahead, or
        stop_test(travelled) returns True. Returns the distance travelled."""
        speed = min(CENTERING_SPEED, max_speed)
        started = robot.getTime()
        travelled = 0.0
        while True:
            front = sense_step()
            if robot.getTime() - started > 120.0:
                raise RuntimeError("Timed out creeping at the maze entrance")
            travelled = along(origin, heading)
            if travelled >= limit or front < FRONT_STOP + 0.02:
                break
            if stop_test is not None and stop_test(travelled):
                break
            fix = max(-0.1, min(0.1, 1.5 * angle_difference(heading, yaw)))
            set_wheels(speed - fix, speed + fix)
        set_wheels(0.0, 0.0)
        return travelled

    def sides_near():
        return (
            sensors["left"].getValue() < CORRIDOR_SIDE_MAX
            and sensors["right"].getValue() < CORRIDOR_SIDE_MAX
        )

    def choose_north(axis_angle, fallback):
        """Pick which way along the corridor leads into the maze.

        The robot travelled from the start towards the maze, so maze north is
        the corridor direction with positive progress from the start (0, 0).
        """
        progress = x * math.cos(axis_angle) + y * math.sin(axis_angle)
        if abs(progress) >= 0.15:
            heading = axis_angle if progress > 0 else axis_angle + math.pi
        else:
            heading = min(
                (axis_angle, axis_angle + math.pi),
                key=lambda c: abs(angle_difference(c, fallback)),
            )
        return angle_difference(heading, 0.0)

    def enter_corridor(north):
        """Creep north until both side beams see a wall. True when inside."""
        count = [0]

        def inside(_):
            count[0] = count[0] + 1 if sides_near() else 0
            return count[0] >= 5

        drive_along((x, y), north, 0.35, inside)
        return count[0] >= 5

    def measure_heading_error():
        """Creep SLOPE_SPAN m and compare it with the two side walls.

        With the heading rotated by theta from the corridor axis, the
        right-minus-left distance changes by 2*tan(theta) per metre driven.
        Returns (heading_used, theta) or None.
        """
        heading = yaw
        samples = []

        def record(travelled):
            if sides_near():
                samples.append((
                    travelled,
                    sensors["right"].getValue() - sensors["left"].getValue(),
                ))
            return False

        drive_along((x, y), heading, SLOPE_SPAN, record)
        if len(samples) < 40 or samples[-1][0] - samples[0][0] < 0.08:
            return None
        mean_s = sum(p[0] for p in samples) / len(samples)
        mean_d = sum(p[1] for p in samples) / len(samples)
        var = sum((p[0] - mean_s) ** 2 for p in samples)
        slope = sum((p[0] - mean_s) * (p[1] - mean_d) for p in samples) / var
        return heading, math.atan(slope / 2.0)

    def align_to_walls(north):
        """Refine maze north so the robot runs parallel to the corridor."""
        for _ in range(3):
            if not enter_corridor(north):
                print("WARNING: side walls not seen; keeping scanned heading.")
                return north
            result = measure_heading_error()
            if result is None:
                print("WARNING: wall-slope measurement failed.")
                return north
            heading, theta = result
            print(f"Wall slope: heading error {math.degrees(theta):+.1f} deg.")
            if abs(theta) > math.radians(25):
                print("WARNING: implausible heading error; ignoring it.")
                return north
            north = angle_difference(heading - theta, 0.0)
            turn_to(north, 0.5)
            if abs(theta) < math.radians(1.0):
                break
        return north

    def center_between_walls(north):
        """Slide sideways so the left and right wall distances are equal."""
        set_wheels(0.0, 0.0)
        left = right = 0.0
        for _ in range(20):
            sense_step()
            left += sensors["left"].getValue()
            right += sensors["right"].getValue()
        left /= 20.0
        right /= 20.0
        if left >= CORRIDOR_SIDE_MAX or right >= CORRIDOR_SIDE_MAX:
            print("WARNING: not between two walls; skipping side centring.")
            return
        shift = (right - left) / 2.0
        print(f"Side distances left={left:.3f} right={right:.3f}; "
              f"shifting {shift * 100:+.1f} cm to the right.")
        if abs(shift) < 0.004 or abs(shift) > 0.08:
            return
        sideways = angle_difference(
            north - math.pi / 2 if shift > 0 else north + math.pi / 2, 0.0
        )
        origin = (x, y)
        turn_to(sideways)
        go_to(origin, sideways, sideways, +1, abs(shift))
        turn_to(north, 0.5)

    def place_in_first_cell(north):
        """Use the red cell's far edge to park in the first maze cell.

        The camera may not look straight down under the robot, so the edge
        is crossed in both directions: facing north the camera leaves the red
        late by its offset, facing south it enters the red late by the same
        amount, and the average of the two is the true edge.
        """
        back = angle_difference(north + math.pi, 0.0)
        reference = (x, y)

        # 1. Get clear of the red cell, to its north.
        clear = [0]

        def beyond_red(_):
            clear[0] = clear[0] + 1 if read_floor(camera) != "red" else 0
            return clear[0] >= 5

        drive_along(reference, north, 0.6, beyond_red)
        if clear[0] < 5:
            print("WARNING: could not get clear of the red cell.")
            return

        # 2. Face south and find where the red starts.
        turn_to(back)
        hits = [0]

        def red_ahead(_):
            hits[0] = hits[0] + 1 if read_floor(camera) == "red" else 0
            return hits[0] >= 3

        drive_along((x, y), back, 0.8, red_ahead)
        if hits[0] < 3:
            print("WARNING: red cell not found again; staying put.")
            return
        entered = along(reference, north)
        # Carry on to the red cell's near edge, so that after turning round
        # the camera is certainly still over red.
        gone = [0]

        def left_south(_):
            gone[0] = gone[0] + 1 if read_floor(camera) != "red" else 0
            return gone[0] >= 3

        drive_along((x, y), back, 0.4, left_south)
        south_exit = along(reference, north)

        # 3. Face north and find where the red ends.
        turn_to(north)
        seen = [False]
        lost = [0]

        def left_red(_):
            if read_floor(camera) == "red":
                seen[0] = True
                lost[0] = 0
            else:
                lost[0] += 1
            return seen[0] and lost[0] >= 3

        drive_along((x, y), north, 0.6, left_red)
        if not (seen[0] and lost[0] >= 3):
            print("WARNING: red cell end not found; staying put.")
            return
        left = along(reference, north)

        far_edge = (entered + left) / 2.0
        target = far_edge + FIRST_CELL_OFFSET
        print(
            f"Red cell depth {entered - south_exit:.3f} m, camera offset "
            f"{(entered - left) / 2.0:+.3f} m; parking in the first cell "
            f"({FIRST_CELL_OFFSET:.3f} m past the edge)."
        )
        go_to(reference, north, north, +1, target)

    def center_on_red():
        """Park in the first maze cell, centred and facing maze north.

        Returns the maze-north heading (start frame, radians).
        """
        hold_still(0.3)

        if FINAL_HEADING is not None:
            north = angle_difference(FINAL_HEADING, 0.0)
        else:
            arrival_heading = yaw
            axis_angle = scan_corridor_axis()
            if axis_angle is not None:
                north = choose_north(axis_angle, arrival_heading)
                print(
                    "Corridor walls found; maze north = "
                    f"{math.degrees(north):.1f} deg (start frame)."
                )
            else:
                north = round(yaw / (math.pi / 2)) * (math.pi / 2)
                print("WARNING: corridor walls not found by the scan.")

        turn_to(north)
        north = align_to_walls(north)
        center_between_walls(north)
        place_in_first_cell(north)
        turn_to(north, 0.5)
        hold_still(0.5)
        print(f"Parked facing maze north ({math.degrees(north):.1f} deg).")
        return north

    print(
        f"Obstacle field: goal={GOAL}, arena x={ARENA_X}, y={ARENA_Y}, "
        f"inflation cells={RADII_CELLS}"
    )

    try:
        while robot.step(time_step) != -1:
            now = robot.getTime()
            step_count += 1

            readings = {n: s.getValue() for n, s in sensors.items()}
            roll, pitch, yaw_abs = imu.getRollPitchYaw()
            left_position = left_encoder.getValue()
            right_position = right_encoder.getValue()

            desired_left = 0.0
            desired_right = 0.0

            if state != "SETTLING":
                values = [roll, pitch, yaw_abs, left_position,
                          right_position, *readings.values()]
                if not all(math.isfinite(v) for v in values):
                    raise RuntimeError("Invalid sensor reading")
                if not all(0.0 <= v <= 1.0 for v in readings.values()):
                    raise RuntimeError("Invalid distance reading")
                if abs(pitch) > MAX_TILT or abs(roll) > MAX_TILT:
                    raise RuntimeError(
                        "Excessive tilt: "
                        f"pitch={math.degrees(pitch):.1f}, "
                        f"roll={math.degrees(roll):.1f}"
                    )
                if now - start_time > MAX_RUN_TIME:
                    raise RuntimeError("Run timed out")

            if state == "SETTLING":
                if now - state_started >= 1.0 and math.isfinite(yaw_abs):
                    yaw0 = yaw_abs
                    previous_left = left_position
                    previous_right = right_position
                    state = "DRIVE"
                    stuck_ref = (x, y, yaw, now)

            else:
                # ---- Odometry ------------------------------------------
                yaw = angle_difference(yaw_abs, yaw0)
                d_left = (left_position - previous_left) * WHEEL_RADIUS
                d_right = (right_position - previous_right) * WHEEL_RADIUS
                previous_left, previous_right = left_position, right_position
                step_distance = (d_left + d_right) / 2.0
                x += step_distance * math.cos(yaw)
                y += step_distance * math.sin(yaw)
                travelled += abs(step_distance)

                # ---- Mapping -------------------------------------------
                if step_count % 2 == 0:
                    for name, angle in BEAMS:
                        grid.integrate(x, y, yaw, angle, readings[name])

                front = readings["front"]
                left_d = readings["left"]
                right_d = readings["right"]
                goal_distance = math.hypot(target[0] - x, target[1] - y)

            # ---------------- State machine -----------------------------
            if state == "DRIVE":
                # Red transition zone? (ignore the red start zone)
                if True:
                    if (
                        math.hypot(x, y) > MIN_TRAVEL_BEFORE_RED
                        and read_floor(camera) == "red"
                    ):
                        red_reads += 1
                    else:
                        red_reads = 0
                    if red_reads >= RED_CONFIRM_READS:
                        reached = True
                        print("Red cell detected; centring on it.")
                        north = center_on_red()
                        state = "DONE"
                        print(
                            f"TRANSITION ZONE REACHED at ({x:.2f}, {y:.2f}) "
                            f"after {now - start_time:.1f} s, "
                            f"{travelled:.2f} m driven."
                        )

            if state == "DRIVE":
                at_last_waypoint = (
                    bool(path)
                    and path_index >= len(path) - 1
                    and math.hypot(path[-1][0] - x, path[-1][1] - y) < 0.05
                )
                if goal_distance <= target_tolerance or at_last_waypoint:
                    if not searching:
                        searching = True
                        search_queue = build_search_points(grid, (x, y))
                        print(
                            "Estimated goal reached but no red floor seen; "
                            f"searching {len(search_queue)} nearby points."
                        )
                    if not next_search_target():
                        raise RuntimeError(
                            "Red transition zone not found near the goal. "
                            "Check GOAL / ARENA constants."
                        )

            if state == "DRIVE":
                # ---- (Re)planning ---------------------------------------
                need_plan = force_replan or not path
                improve_only = False

                if not need_plan and grid.version != checked_version:
                    checked_version = grid.version
                    if not path_clear(
                        grid, (x, y), path[path_index:], path_level
                    ):
                        need_plan = True

                if (
                    not need_plan
                    and path_level > 0
                    and now - last_plan_time > 2.0
                ):
                    need_plan = True
                    improve_only = True

                if need_plan:
                    last_plan_time = now
                    force_replan = False
                    new_path, new_level = find_route(
                        grid, (x, y), target, target_tolerance,
                        only_better_than=path_level if improve_only else None,
                    )
                    if new_path:
                        path, path_level = new_path, new_level
                        path_index = 1 if len(path) > 1 else 0
                        checked_version = grid.version
                        plan_failures = 0
                    elif not improve_only and searching:
                        # Unreachable search point: try the next one.
                        path = []
                        if not next_search_target():
                            raise RuntimeError(
                                "Red transition zone not found near the goal"
                            )
                    elif not improve_only:
                        path = []
                        plan_failures += 1
                        if plan_failures > 3:
                            raise RuntimeError("No route to the goal found")
                        print("No route known; scanning surroundings.")
                        state = "SCAN"
                        scan_total = 0.0
                        scan_previous = yaw

            if state == "DRIVE" and path:
                # ---- Path following -------------------------------------
                while (
                    path_index < len(path) - 1
                    and math.hypot(path[path_index][0] - x,
                                   path[path_index][1] - y) < 0.08
                ):
                    path_index += 1

                tx, ty = path[path_index]
                bearing = math.atan2(ty - y, tx - x)
                error = angle_difference(bearing, yaw)

                # Gently steer away from very close side obstacles.
                if left_d < SIDE_CAUTION:
                    error -= 4.0 * (SIDE_CAUTION - left_d)
                if right_d < SIDE_CAUTION:
                    error += 4.0 * (SIDE_CAUTION - right_d)

                if rotating:
                    if abs(error) < 0.25:
                        rotating = False
                elif abs(error) > 0.7:
                    rotating = True

                if rotating:
                    turn = max(-turn_speed, min(turn_speed, 1.5 * error))
                    if abs(turn) < 0.15:
                        turn = math.copysign(0.15, error)
                    desired_left, desired_right = -turn, turn
                elif front < FRONT_STOP:
                    state = "BACKUP"
                    backup_origin = (x, y)
                    backup_started = now
                    print(f"Too close to an obstacle (front={front:.3f}). "
                          "Backing up.")
                else:
                    slow = max(
                        0.35,
                        min(1.0, (front - FRONT_STOP)
                            / (FRONT_SLOW - FRONT_STOP)),
                    )
                    align = max(0.35, 1.0 - abs(error) / 0.7)
                    speed = cruise * slow * align
                    steer = max(-0.5 * cruise, min(0.5 * cruise, 1.5 * error))
                    desired_left = speed - steer
                    desired_right = speed + steer

                # ---- Stuck detection ------------------------------------
                if state == "DRIVE" and now - stuck_ref[3] > 4.0:
                    moved = math.hypot(x - stuck_ref[0], y - stuck_ref[1])
                    turned = abs(angle_difference(yaw, stuck_ref[2]))
                    if (
                        moved < 0.02
                        and turned < 0.15
                        and abs(command_left) + abs(command_right) > 0.2
                    ):
                        print("Stuck detected; marking obstacle ahead.")
                        grid.force_occupied(
                            x + 0.12 * math.cos(yaw),
                            y + 0.12 * math.sin(yaw),
                        )
                        state = "BACKUP"
                        backup_origin = (x, y)
                        backup_started = now
                    stuck_ref = (x, y, yaw, now)

            elif state == "BACKUP":
                moved_back = math.hypot(x - backup_origin[0],
                                        y - backup_origin[1])
                if moved_back >= 0.08 or now - backup_started > 2.5:
                    backups = backups + 1 if now - last_backup_time < 10.0 else 1
                    last_backup_time = now
                    if backups > 8:
                        raise RuntimeError("Repeatedly blocked; giving up")
                    force_replan = True
                    rotating = False
                    stuck_ref = (x, y, yaw, now)
                    state = "DRIVE"
                else:
                    desired_left = desired_right = -backup_speed

            elif state == "SCAN":
                scan_total += abs(angle_difference(yaw, scan_previous))
                scan_previous = yaw
                if scan_total >= 2.0 * math.pi - 0.3:
                    force_replan = True
                    stuck_ref = (x, y, yaw, now)
                    state = "DRIVE"
                else:
                    desired_left, desired_right = turn_speed, -turn_speed

            # ---------------- Apply wheel commands ----------------------
            if state in ("SETTLING", "DONE"):
                command_left = command_right = 0.0
            else:
                max_change = WHEEL_ACCEL * dt
                command_left += max(-max_change, min(
                    max_change, desired_left - command_left))
                command_right += max(-max_change, min(
                    max_change, desired_right - command_right))
                command_left = max(-max_speed, min(max_speed, command_left))
                command_right = max(-max_speed, min(max_speed, command_right))

            left_motor.setVelocity(command_left)
            right_motor.setVelocity(command_right)

            if state == "DONE":
                break

            if now - last_report >= 1.0:
                print(
                    f"{state} | pos=({x:.2f}, {y:.2f}) | "
                    f"yaw={math.degrees(yaw):.0f} deg | "
                    f"goal_dist={goal_distance:.2f} | "
                    f"F/L/R={readings['front']:.2f}/"
                    f"{readings['left']:.2f}/{readings['right']:.2f} | "
                    f"wp={path_index}/{len(path)} lvl={path_level}"
                )
                last_report = now

    except RuntimeError as error:
        print(f"OBSTACLE NAVIGATION STOPPED: {error}")
        print(f"Last estimated position: ({x:.2f}, {y:.2f})")

    finally:
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        command_left = command_right = 0.0
        # A velocity command only takes effect on the next simulation step.
        # Without these steps the wheels keep their last speed after the
        # controller returns and the robot rolls on.
        for _ in range(int(0.5 / dt)):
            if robot.step(time_step) == -1:
                break
        render_map(grid, (x, y))

    return reached
