"""Measured forward movement with heading correction."""

import math
from turning import angle_difference


class MoveForward:
    def __init__(
        self,
        target_distance,
        start_time,
        left_position,
        right_position,
        yaw,
    ):
        self.target_distance = target_distance
        self.start_time = start_time
        self.previous_time = start_time

        self.start_left = left_position
        self.start_right = right_position
        self.previous_left = left_position
        self.previous_right = right_position
        self.target_yaw = yaw

        self.left_command = 0.0
        self.right_command = 0.0

        self.distance = 0.0
        self.complete = False
        self.fault = None
        self.status = "Moving"

        self.hold_started = None
        self.progress_mark = 0.0
        self.last_progress_time = start_time

    def stop_with_fault(self, reason):
        self.fault = reason
        self.status = "FAULT: " + reason
        self.left_command = 0.0
        self.right_command = 0.0
        return 0.0, 0.0

    def update(
        self,
        now,
        left_position,
        right_position,
        yaw,
        front_distance,
    ):
        if self.complete or self.fault:
            return 0.0, 0.0

        readings = (
            left_position,
            right_position,
            yaw,
            front_distance,
        )

        if not all(math.isfinite(value) for value in readings):
            return self.stop_with_fault("Invalid sensor reading")

        if not 0.0 <= front_distance <= 1.0:
            return self.stop_with_fault("Invalid front distance")

        dt = now - self.previous_time

        if dt <= 0.0:
            return self.left_command, self.right_command

        self.previous_time = now

        left_rate = (
            left_position - self.previous_left
        ) / dt

        right_rate = (
            right_position - self.previous_right
        ) / dt

        self.previous_left = left_position
        self.previous_right = right_position

        wheel_radius = 0.02

        left_distance = (
            left_position - self.start_left
        ) * wheel_radius

        right_distance = (
            right_position - self.start_right
        ) * wheel_radius

        self.distance = (left_distance + right_distance) / 2
        remaining = self.target_distance - self.distance

        if now - self.start_time > 60.0:
            return self.stop_with_fault("Movement timed out")

        if remaining < -0.01:
            return self.stop_with_fault("Overshot target by over 1 cm")

        heading_error = angle_difference(self.target_yaw, yaw)

        if abs(heading_error) > math.radians(15):
            return self.stop_with_fault("Excessive heading deviation")

        at_target = remaining <= 0.002

        if not at_target:
            # An early stop is BLOCKED, not successful completion.
            if front_distance <= 0.05:
                return self.stop_with_fault("Blocked before target")

            if self.distance > self.progress_mark + 0.001:
                self.progress_mark = self.distance
                self.last_progress_time = now

            elif now - self.last_progress_time > 5.0:
                return self.stop_with_fault("No encoder progress")

            base_speed = min(
                0.5,
                max(0.1, 20.0 * remaining)
            )

            correction = max(
                -0.15,
                min(0.15, 2.0 * heading_error)
            )

            desired_left = max(
                0.0,
                min(0.5, base_speed - correction)
            )

            desired_right = max(
                0.0,
                min(0.5, base_speed + correction)
            )

            self.status = "Moving"

        else:
            desired_left = 0.0
            desired_right = 0.0
            self.status = "Settling at target"

        max_change = 0.25 * dt

        self.left_command += max(
            -max_change,
            min(
                max_change,
                desired_left - self.left_command
            )
        )

        self.right_command += max(
            -max_change,
            min(
                max_change,
                desired_right - self.right_command
            )
        )

        stopped = (
            abs(self.left_command) <= 0.02
            and abs(self.right_command) <= 0.02
            and abs(left_rate) <= 0.05
            and abs(right_rate) <= 0.05
        )

        if at_target and stopped:
            if self.hold_started is None:
                self.hold_started = now

            elif now - self.hold_started >= 0.3:
                self.complete = True
                self.status = "MOVE COMPLETE"
                self.left_command = 0.0
                self.right_command = 0.0

        else:
            self.hold_started = None

        return self.left_command, self.right_command