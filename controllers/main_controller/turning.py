"""Heading-controlled 90-degree turns; all angles are radians."""

import math

def angle_difference(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))


class Turn90:
    """Call update once per simulation step and apply its wheel velocities."""

    def __init__(
        self, direction, start_time, *, max_speed=1.0,
        acceleration=1.0, settle_seconds=0.25,
    ):
        if direction not in ("left", "right"):
            raise ValueError("Turn direction must be left or right")
        if not all(math.isfinite(v) and v > 0 for v in (max_speed, acceleration)):
            raise ValueError(
                "Turn speed and acceleration must be positive and finite"
            )
        if not math.isfinite(settle_seconds) or settle_seconds < 0:
            raise ValueError("Turn settling time must be nonnegative and finite")
        self.max_speed = max_speed
        self.acceleration = acceleration
        self.settle_seconds = settle_seconds
        self.direction = 1 if direction == "left" else -1
        self.start_time = start_time
        self.previous_time = start_time
        self.previous_yaw = None
        self.start_yaw = None
        self.target_yaw = None
        self.turn_started = None
        self.hold_started = None
        self.speed = 0.0
        self.error = None
        self.change = 0.0
        self.status = "Settling"
        self.complete = False
        self.fault = None

    def update(self, now, yaw):
        dt = now - self.previous_time
        self.previous_time = now
        if self.complete or self.fault:
            return 0.0, 0.0
        if now - self.start_time < self.settle_seconds:
            self.previous_yaw = yaw if math.isfinite(yaw) else None
            return 0.0, 0.0
        if not math.isfinite(yaw):
            self.fault = "Invalid IMU heading"
            self.status = self.fault
            return 0.0, 0.0
        if dt <= 0.0:
            return -self.speed, self.speed

        yaw_rate = (
            angle_difference(yaw, self.previous_yaw) / dt
            if self.previous_yaw is not None else 0.0
        )
        self.previous_yaw = yaw

        if self.target_yaw is None:
            self.start_yaw = yaw
            self.target_yaw = angle_difference(
                yaw + self.direction * math.pi / 2, 0.0
            )
            self.turn_started = now

        self.error = angle_difference(self.target_yaw, yaw)
        self.change = angle_difference(yaw, self.start_yaw)
        if now - self.turn_started >= 12.0:
            self.fault = "Turn timed out"
            self.status = self.fault
            self.speed = 0.0
            return 0.0, 0.0

        within_tolerance = abs(self.error) <= math.radians(2)
        # Differential-drive yaw rate = 2 * wheel_radius / axle_length * speed.
        # Limit the turn speed by the remaining angle as well as the cruise cap.
        braking_speed = math.sqrt(
            self.acceleration * 0.064 * abs(self.error) / 0.02
        )
        desired_speed = (
            0.0 if within_tolerance
            else math.copysign(
                min(self.max_speed, braking_speed, 2.5 * abs(self.error)),
                self.error,
            )
        )
        max_change = self.acceleration * dt
        self.speed += max(
            -max_change, min(max_change, desired_speed - self.speed)
        )
        self.status = "Settling at target" if within_tolerance else "Turning"

        stopped = abs(self.speed) <= 0.02 and abs(yaw_rate) <= math.radians(3)
        if within_tolerance and stopped:
            if self.hold_started is None:
                self.hold_started = now
            elif now - self.hold_started >= 0.3:
                self.complete = True
                self.status = "TURN COMPLETE"
                self.speed = 0.0
        else:
            self.hold_started = None

        return -self.speed, self.speed
