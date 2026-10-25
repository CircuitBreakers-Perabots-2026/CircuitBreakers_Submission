"""Section 1 mapping, A* exploration and verified maze-entry handoff.

Mapping and path-planning adapted from CircuitBreakers' Perabots-section01
repository, commit ffcfbcee353edae30a103f42b2539c28cbfaaea8. Navigation
searches sensed boundary openings rather than a fixed transition coordinate.
The start frame points forward along the long arena side, with the arena
on the robot's right, as in the supplied Arena.wbt starting pose.
"""

import heapq
import math
from dataclasses import dataclass

from floor_detection import read_floor
from movement import MoveForward
from maze_navigation import CardinalTurn
from turning import angle_difference

WHEEL_RADIUS = 0.02
CAMERA_FORWARD = 0.02
CELL_PITCH = 0.25
WALL_THICKNESS = 0.012
ARENA_X = (-0.125, 2.44 - 0.125)
ARENA_Y = (-(1.22 - 0.125), 0.125)
RES = 0.025
PADDING = 0.15
I_MIN = math.floor((ARENA_X[0] - PADDING) / RES)
I_MAX = math.ceil((ARENA_X[1] + PADDING) / RES)
J_MIN = math.floor((ARENA_Y[0] - PADDING) / RES)
J_MAX = math.ceil((ARENA_Y[1] + PADDING) / RES)
# Never fall back to a zero-width robot when a path is blocked.
LEVEL_RADII = (0.09, 0.06)
RADII_CELLS = tuple(math.ceil(r / RES) for r in LEVEL_RADII)
L_HIT, L_FREE, L_OCC = 0.8, -0.15, 0.5
L_MIN, L_MAX = -2.0, 6.0
HIT_MAX_READING, MAP_RANGE = 0.98, 0.75
CUBE_DEPTH_CELLS = 2
GOAL_TOLERANCE = 0.045
NEIGHBOURS = (
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
    (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
)
# Exact sensor origins in the permitted extensionSlot, in robot coordinates.
BEAMS = {
    "front": (0.045, 0.0, 0.0),
    "left": (0.003, 0.045, math.pi / 2),
    "right": (0.003, -0.045, -math.pi / 2),
    "rear": (-0.045, 0.0, math.pi),
    "front_left": (0.04, 0.025, math.pi / 4),
    "front_right": (0.04, -0.025, -math.pi / 4),
}
MAX_TILT = math.radians(10)
CRUISE_SPEED, TURN_SPEED, ACCELERATION = 1.8, 1.0, 1.0
FRONT_STOP = 0.055

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

    def integrate(self, px, py, yaw, beam_angle, reading, mount=(0.045, 0.0)):
        """Add one distance-sensor beam to the map."""
        angle = yaw + beam_angle
        dx, dy = math.cos(angle), math.sin(angle)
        px, py = (px + math.cos(yaw) * mount[0] - math.sin(yaw) * mount[1],
                  py + math.sin(yaw) * mount[0] + math.cos(yaw) * mount[1])
        hit = reading < HIT_MAX_READING
        end = reading

        free_end = min(end - RES, MAP_RANGE)
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


def in_bounds(cell):
    return I_MIN <= cell[0] <= I_MAX and J_MIN <= cell[1] <= J_MAX


def line_free(grid, a, b, level):
    distance = math.hypot(b[0] - a[0], b[1] - a[1])
    steps = max(1, int(distance / (RES * 0.5)))
    for k in range(steps + 1):
        t = k / steps
        cell = cell_of(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
        if not walkable(grid, cell, level):
            return False
    return True


def plan(grid, start_xy, goal_xy, level, tolerance=GOAL_TOLERANCE):
    """A* from start to goal; returns a list of (x, y) or None."""
    start = cell_of(*start_xy)

    def available(cell):
        return walkable(grid, cell, level)

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
            if nxt in closed or not available(nxt):
                continue
            if di and dj and not (
                available((cell[0] + di, cell[1]))
                and available((cell[0], cell[1] + dj))
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


def find_route(grid, position, goal, tolerance):
    """Try the safest clearance first, then progressively tighter ones."""
    for level in range(len(RADII_CELLS)):
        raw = plan(grid, position, goal, level, tolerance)
        if raw:
            return smooth(raw, grid, level), level
    return None, None


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


def inside_arena(x, y):
    return ARENA_X[0] <= x <= ARENA_X[1] and ARENA_Y[0] <= y <= ARENA_Y[1]


def walkable(grid, cell, level):
    if not in_bounds(cell) or grid.blocked(cell, level):
        return False
    # Beyond the field boundary, use only space actually seen by a beam.
    # This lets the planner use a discovered doorway without inventing one.
    return inside_arena(*center_of(cell)) or grid.logodds.get(cell, 0) < -0.1


def inspection_points():
    """Inspect the arena perimeter; no transition location is assumed."""
    inset = 0.13
    result = []
    for lo, hi, horizontal in ((ARENA_X[0], ARENA_X[1], True),
                               (ARENA_Y[0], ARENA_Y[1], False)):
        count = math.ceil((hi - lo - 2 * inset) / 0.14)
        for i in range(count + 1):
            v = lo + inset + i * (hi - lo - 2 * inset) / count
            if horizontal:
                result.extend(((v, ARENA_Y[0] + inset),
                               (v, ARENA_Y[1] - inset)))
            else:
                result.extend(((ARENA_X[0] + inset, v),
                               (ARENA_X[1] - inset, v)))
    return result


def survey_points():
    """Viewpoints across the field, plus close boundary inspections as fallback."""
    inset = 0.13
    axes = []
    for lo, hi in (ARENA_X, ARENA_Y):
        count = math.ceil((hi - lo - 2 * inset) / 0.25)
        axes.append([lo + inset + i * (hi - lo - 2 * inset) / count
                     for i in range(count + 1)])
    return list(dict.fromkeys([(x, y) for x in axes[0] for y in axes[1]]
                              + inspection_points()))


def boundary_samples():
    """Uniform samples of all four boundaries; no exit location is assumed."""
    points = []
    for lo, hi, horizontal in ((*ARENA_X, True), (*ARENA_Y, False)):
        count = math.ceil((hi - lo) / RES)
        for i in range(count + 1):
            along = lo + i * (hi - lo) / count
            if horizontal:
                points.extend(((along, ARENA_Y[0]), (along, ARENA_Y[1])))
            else:
                points.extend(((ARENA_X[0], along), (ARENA_X[1], along)))
    return list(dict.fromkeys(points))


def visible_boundary_gain(grid, position, unknown_boundary):
    """Estimate how much unobserved boundary a short scan could reveal."""
    gain = 0
    for point in unknown_boundary:
        distance = math.dist(position, point)
        if distance > MAP_RANGE - 0.1:
            continue
        steps = max(1, math.ceil(distance / RES))
        for i in range(1, steps + 1):
            fraction = i / steps
            cell = cell_of(position[0] + fraction * (point[0] - position[0]),
                           position[1] + fraction * (point[1] - position[1]))
            # A sensor ray is blocked by the obstacle, not its clearance halo.
            if cell in grid.occupied:
                break
        else:
            gain += 1
    return gain


def opening_route(grid, position, inspected):
    """Prioritise a measured opening over further exploration."""
    openings = [point for point, _ in boundary_openings(grid)
                if not any(math.dist(point, old) < 0.10 for old in inspected)]
    for point in sorted(openings, key=lambda p: math.dist(position, p)):
        route, level = find_route(grid, position, point, GOAL_TOLERANCE)
        if route:
            return point, route, level
    return None


def exploration_route(grid, position, candidates, inspected):
    """Choose boundary information per metre of planned travel, not edge order."""
    opening = opening_route(grid, position, inspected)
    if opening is not None:
        return opening
    remaining = [point for point in candidates
                 if not any(math.dist(point, old) < 0.10 for old in inspected)]
    unknown = [point for point in boundary_samples()
               if abs(grid.logodds.get(cell_of(*point), 0.0)) < 0.1]
    ranked = []
    for point in remaining:
        if grid.blocked(cell_of(*point), len(RADII_CELLS) - 1):
            continue
        gain = visible_boundary_gain(grid, point, unknown)
        if gain:
            # Include the cost of braking, turning and scanning rather than
            # preferring many tiny moves that reveal little new boundary.
            score = gain / (math.dist(position, point) + 0.35)
            ranked.append((score, gain, point))
    best_score, best = 0.0, None
    for upper_score, gain, point in sorted(ranked, reverse=True):
        if best is not None and upper_score < best_score:
            break
        route, level = find_route(grid, position, point, GOAL_TOLERANCE)
        if route:
            length = sum(math.dist(a, b) for a, b in zip(route, route[1:]))
            length += math.dist(route[-1], point)
            score = gain / (length + 0.35)
            if score > best_score:
                best_score, best = score, (point, route, level)
    if best is not None:
        return best
    # If a boundary opening was missed or occluded, inspect nearby positions.
    for point in sorted(remaining, key=lambda p: math.dist(position, p)):
        route, level = find_route(grid, position, point, GOAL_TOLERANCE)
        if route:
            return point, route, level
    raise RuntimeError("No reachable unexplored viewpoint or sensed opening")


def boundary_openings(grid):
    """Find runs of observed free cells crossing an outer boundary."""
    found = []
    for edge, normal, lo, hi, horizontal in (
        (ARENA_Y[0], -math.pi / 2, *ARENA_X, True),
        (ARENA_Y[1], math.pi / 2, *ARENA_X, True),
        (ARENA_X[0], math.pi, *ARENA_Y, False),
        (ARENA_X[1], 0.0, *ARENA_Y, False),
    ):
        run = []
        for i in range(math.ceil((hi - lo) / RES) + 1):
            along = lo + i * RES
            px, py = (along, edge) if horizontal else (edge, along)
            free = grid.logodds.get(cell_of(px, py), 0) < -0.1
            free = free and cell_of(px, py) not in grid.occupied
            if free:
                run.append((px, py))
            if (not free or along >= hi) and run:
                if len(run) >= 5:
                    mid = run[len(run) // 2]
                    goal = (mid[0] + 0.08 * math.cos(normal),
                            mid[1] + 0.08 * math.sin(normal))
                    found.append((goal, normal))
                run = []
    return found


@dataclass(frozen=True)
class MazeEntrance:
    north_yaw: float
    elapsed: float
    travelled: float


class NavigationIO:
    """Shared odometry, validation and acceleration-limited wheel control."""

    def __init__(self, robot, time_step):
        self.robot, self.time_step = robot, time_step
        self.dt = time_step / 1000.0
        self.motors = [get_device(robot, name + "_wheel_motor")
                       for name in ("left", "right")]
        self.encoders = [get_device(robot, name + "_wheel_sensor")
                         for name in ("left", "right")]
        self.sensors = {name: get_device(robot, name + "_distance") for name in BEAMS}
        self.imu = get_device(robot, "imu")
        self.camera = get_device(robot, "floor_camera")
        for device in [*self.encoders, *self.sensors.values(), self.imu, self.camera]:
            device.enable(time_step)
        for motor in self.motors:
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)
        for name in ("led_blue", "led_green"):
            get_device(robot, name).set(0)
        self.limit = min(10.0, *(motor.getMaxVelocity() for motor in self.motors))
        self.commands = [0.0, 0.0]
        self.positions = [0.0, 0.0]
        self.previous_positions = None
        self.x = self.y = self.yaw = 0.0
        self.yaw0 = None
        self.yaw_abs = 0.0
        self.travelled = 0.0
        self.peak_tilt = 0.0
        self.started = robot.getTime()
        self.now = self.started
        self.readings = {}
        self.grid = OccupancyGrid()
        self.frames = 0
        self.progress_mark = (0.0, 0.0, 0.0, self.started)

    def frame(self):
        if self.robot.step(self.time_step) == -1:
            raise RuntimeError("Simulation ended")
        self.now = self.robot.getTime()
        self.positions = [encoder.getValue() for encoder in self.encoders]
        roll, pitch, self.yaw_abs = self.imu.getRollPitchYaw()
        self.readings = {name: device.getValue() for name, device in self.sensors.items()}
        values = [roll, pitch, self.yaw_abs, *self.positions, *self.readings.values()]
        if not all(math.isfinite(v) for v in values):
            raise RuntimeError("Invalid Section 1 sensor reading")
        if not all(0 <= v <= 1 for v in self.readings.values()):
            raise RuntimeError("Invalid Section 1 distance reading")
        self.peak_tilt = max(self.peak_tilt, abs(roll), abs(pitch))
        if self.peak_tilt > MAX_TILT:
            raise RuntimeError("Excessive Section 1 body tilt")
        if self.now - self.started > 1800:
            raise RuntimeError("Section 1 exploration timed out")
        if self.yaw0 is None:
            self.yaw0 = self.yaw_abs
        old_yaw = self.yaw
        self.yaw = angle_difference(self.yaw_abs, self.yaw0)
        if self.previous_positions is not None:
            moved = sum(p - old for p, old in zip(
                self.positions, self.previous_positions)) * WHEEL_RADIUS / 2
            middle_yaw = old_yaw + angle_difference(self.yaw, old_yaw) / 2
            self.x += moved * math.cos(middle_yaw)
            self.y += moved * math.sin(middle_yaw)
            self.travelled += abs(moved)
        self.previous_positions = self.positions[:]
        px, py, heading, last_progress = self.progress_mark
        if (math.hypot(self.x - px, self.y - py) > 0.005
                or abs(angle_difference(self.yaw, heading)) > 0.1):
            self.progress_mark = (self.x, self.y, self.yaw, self.now)
        elif max(abs(v) for v in self.commands) > 0.2 and self.now - last_progress > 6:
            raise RuntimeError("No Section 1 position or heading progress for 6 seconds")
        self.frames += 1
        if self.frames % 2 == 0:
            for name, (mx, my, angle) in BEAMS.items():
                self.grid.integrate(self.x, self.y, self.yaw, angle,
                                    self.readings[name], (mx, my))

    def wheels(self, left, right):
        for i, desired in enumerate((left, right)):
            desired = max(-self.limit, min(self.limit, desired))
            change = ACCELERATION * self.dt
            self.commands[i] += max(-change, min(change, desired - self.commands[i]))
            self.motors[i].setVelocity(self.commands[i])

    def brake(self):
        while max(abs(v) for v in self.commands) > 0.001:
            self.wheels(0.0, 0.0)
            self.frame()
        for motor in self.motors:
            motor.setVelocity(0.0)
        self.commands = [0.0, 0.0]

    def hold(self, seconds=0.3):
        self.brake()
        end = self.now + seconds
        while self.now < end:
            self.frame()

    def turn_to(self, relative_heading):
        self.hold(0.1)
        absolute = angle_difference(self.yaw0 + relative_heading, 0)
        direction = "left" if angle_difference(absolute, self.yaw_abs) > 0 else "right"
        action = CardinalTurn(direction, self.now, absolute)
        while not action.complete:
            self.frame()
            left, right = action.update(self.now, self.yaw_abs)
            if action.fault:
                raise RuntimeError(f"Entrance turn: {action.fault}")
            self.wheels(left, right)
        self.hold(0.15)

    def move(self, distance, heading):
        if distance <= 0.001:
            return
        self.hold(0.1)
        action = MoveForward(distance, self.now, *self.positions, self.yaw_abs,
                             max_speed=1.2, acceleration=ACCELERATION)
        action.target_yaw = angle_difference(self.yaw0 + heading, 0)
        while not action.complete:
            self.frame()
            left, right = action.update(self.now, *self.positions,
                                        self.yaw_abs, self.readings["front"])
            if action.fault:
                raise RuntimeError(f"Entrance positioning: {action.fault}")
            self.wheels(left, right)
        self.hold(0.15)

    def scan(self):
        # Six beams include all four cardinal directions. A quarter-turn
        # fills their largest angular gap; a full spin adds no coverage.
        self.hold(0.1)
        swept, previous = 0.0, self.yaw
        while swept < math.pi / 2:
            self.wheels(-TURN_SPEED, TURN_SPEED)
            self.frame()
            swept += abs(angle_difference(self.yaw, previous))
            previous = self.yaw
        self.hold(0.1)


def entrance_handoff(io, north):
    """Verify corridor centring and the red-to-black edge before handoff."""
    io.hold()
    io.turn_to(north)
    start = io.travelled
    stable = 0
    nx, ny = math.cos(north), math.sin(north)
    boundary = max(x * nx + y * ny for x in ARENA_X for y in ARENA_Y)
    while True:
        io.frame()
        if io.travelled - start > 0.20 or io.readings["front"] < 0.05:
            raise RuntimeError("Could not verify the transition corridor")
        left, right = io.readings["left"], io.readings["right"]
        gap = left + right + 0.09
        good = left < 0.16 and right < 0.16 and abs(gap - 0.238) < 0.025
        # Get past the boundary wall's front face before measuring its sides.
        # Otherwise a beam can hit a corner and suggest a false lateral shift.
        good = good and io.x * nx + io.y * ny - boundary >= 0.02
        stable = stable + 1 if good else 0
        if stable >= 5:
            io.hold()
            left, right = io.readings["left"], io.readings["right"]
            gap = left + right + 0.09
            if left < 0.16 and right < 0.16 and abs(gap - 0.238) < 0.025:
                break
            # A beam can briefly clip a wall's front corner while braking.
            # Check again at rest before using it to measure lateral offset.
            stable = 0
        correction = max(-0.2, min(0.2, 4 * angle_difference(north, io.yaw)))
        io.wheels(0.6 - correction, 0.6 + correction)
    shift = (io.readings["right"] - io.readings["left"]) / 2
    if abs(shift) > 0.045:
        raise RuntimeError("Transition is too far off centre for a safe handoff")
    if abs(shift) > 0.002:
        sideways = north - math.pi / 2 if shift > 0 else north + math.pi / 2
        io.turn_to(sideways)
        io.move(abs(shift), sideways)
        io.turn_to(north)
    if abs(io.readings["left"] - io.readings["right"]) > 0.012:
        raise RuntimeError("Transition lateral centring failed")

    # The camera is 20mm in front of the robot origin. Capture the first
    # confirmed red-to-black edge, then compensate this offset and half
    # the wall thickness to reach the first cell's centre.
    start = io.travelled
    black = 0
    edge_distance = None
    while black < 5:
        io.frame()
        label = read_floor(io.camera)
        if label == "black":
            if black == 0:
                edge_distance = io.travelled
            black += 1
        else:
            black = 0
            edge_distance = None
        if io.travelled - start > 0.35 or io.readings["front"] < 0.05:
            raise RuntimeError("Maze entrance red-to-black edge was not found")
        correction = max(-0.2, min(0.2, 4 * angle_difference(north, io.yaw)))
        io.wheels(0.8 - correction, 0.8 + correction)
    io.hold()
    distance = CELL_PITCH / 2 - WALL_THICKNESS / 2 + CAMERA_FORWARD
    distance -= io.travelled - edge_distance
    if not 0.02 < distance < 0.18:
        raise RuntimeError("Invalid first-cell positioning distance")
    io.move(distance, north)
    if read_floor(io.camera) != "black":
        raise RuntimeError("First maze cell floor was not confirmed")
    if abs(angle_difference(north, io.yaw)) > math.radians(2.5):
        raise RuntimeError("Maze entrance heading is not aligned")
    print(f"MAZE ENTRANCE VERIFIED | heading={math.degrees(north):.2f} deg")
    return MazeEntrance(angle_difference(io.yaw0 + north, 0),
                        io.now - io.started, io.travelled)


def nearest_boundary_normal(x, y):
    return min((
        (abs(x - ARENA_X[0]), math.pi),
        (abs(x - ARENA_X[1]), 0.0),
        (abs(y - ARENA_Y[0]), -math.pi / 2),
        (abs(y - ARENA_Y[1]), math.pi / 2),
    ))[1]


def run_obstacle_field(robot, time_step=16):
    """Return a verified MazeEntrance; never report success after a fault."""
    io = NavigationIO(robot, time_step)
    inspections = survey_points()
    inspected = set()
    target = None
    path = []
    path_level = 0
    checked_version = -1
    last_plan = last_report = -1.0
    left_start = False
    red_reads = 0
    recoveries = 0
    rotating = False
    last_opening_check = -1.0

    def choose_target():
        for point in inspections:
            if math.dist((io.x, io.y), point) < 0.08:
                inspected.add(point)
        return exploration_route(io.grid, (io.x, io.y), inspections, inspected)

    try:
        io.hold(1.0)
        io.scan()
        print("SECTION 1: exploring sensed space for the transition opening.")
        while True:
            io.frame()
            label = read_floor(io.camera)
            if label == "white" or (label != "red" and io.travelled > 0.2):
                left_start = True
            near_boundary = min(abs(io.x - edge) for edge in ARENA_X) < 0.18
            near_boundary |= min(abs(io.y - edge) for edge in ARENA_Y) < 0.18
            red = (left_start and math.hypot(io.x, io.y) > 0.3
                   and near_boundary and label == "red")
            red_reads = red_reads + 1 if red else 0
            if red_reads >= 5:
                result = entrance_handoff(io, nearest_boundary_normal(io.x, io.y))
                print(
                    f"SECTION 1 COMPLETE | elapsed={result.elapsed:.3f} s | "
                    f"distance={result.travelled:.3f} m | "
                    f"peak_tilt={math.degrees(io.peak_tilt):.2f} deg"
                )
                return result

            if target is not None and (math.dist((io.x, io.y), target) < 0.06
                                      or (path and math.dist((io.x, io.y), path[-1]) < 0.035)):
                inspected.add(target)
                if inside_arena(*target):
                    io.scan()
                target, path = None, []

            if io.readings["front"] < FRONT_STOP:
                io.brake()
                if io.readings["rear"] < 0.06:
                    raise RuntimeError("Obstacle recovery has insufficient rear clearance")
                origin = io.travelled
                while io.travelled - origin < 0.04:
                    io.frame()
                    if io.readings["rear"] < 0.055:
                        break
                    io.wheels(-0.6, -0.6)
                io.hold()
                recoveries += 1
                if recoveries > 12:
                    raise RuntimeError("Repeated obstacle recovery without reaching the transition")
                io.scan()
                target, path = None, []

            if (target is not None and inside_arena(*target)
                    and io.now - last_opening_check >= 1.0):
                last_opening_check = io.now
                opening = opening_route(io.grid, (io.x, io.y), inspected)
                if opening is not None:
                    io.brake()
                    target, path, path_level = opening
                    if len(path) > 1:
                        path.pop(0)
                    checked_version, last_plan = io.grid.version, io.now

            if path and io.grid.version != checked_version and io.now - last_plan > 0.3:
                checked_version = io.grid.version
                if not path_clear(io.grid, (io.x, io.y), path, path_level):
                    io.brake()
                    path = []
            if not path:
                # Sensor updates can invalidate a path without invalidating
                # its destination. Keep that destination to avoid zig-zagging
                # between nearby frontiers on every map update.
                if target is not None:
                    path, path_level = find_route(
                        io.grid, (io.x, io.y), target, GOAL_TOLERANCE
                    )
                if not path:
                    try:
                        target, path, path_level = choose_target()
                    except RuntimeError:
                        # Scan only when new information is needed to plan.
                        io.scan()
                        target, path, path_level = choose_target()
                checked_version, last_plan = io.grid.version, io.now
                if len(path) > 1:
                    path.pop(0)

            while len(path) > 1 and math.dist((io.x, io.y), path[0]) < 0.05:
                path.pop(0)
            tx, ty = path[0]
            error = angle_difference(math.atan2(ty - io.y, tx - io.x), io.yaw)
            if rotating and abs(error) < 0.18:
                rotating = False
            elif not rotating and abs(error) > 0.65:
                rotating = True
            if rotating:
                spin = max(-TURN_SPEED, min(TURN_SPEED, 2.5 * error))
                io.wheels(-spin, spin)
            else:
                clear = max(0.2, min(1.0, (io.readings["front"] - FRONT_STOP) / 0.16))
                speed = CRUISE_SPEED * clear * max(0.4, 1 - abs(error))
                steer = max(-0.7, min(0.7, 2.5 * error))
                io.wheels(speed - steer, speed + steer)
            if io.now - last_report >= 2:
                print(f"SECTION1 | pose=({io.x:.3f},{io.y:.3f}) | "
                      f"target={tuple(round(v, 3) for v in target)} | "
                      f"inspected={len(inspected)} | front={io.readings['front']:.3f}")
                last_report = io.now
    except RuntimeError as error:
        print(f"SECTION 1 STOPPED: {error}")
        return None
    finally:
        for motor in io.motors:
            motor.setVelocity(0.0)

