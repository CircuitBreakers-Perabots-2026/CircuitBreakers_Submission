"""Run movement actions one after another."""

from movement import MoveForward
from turning import Turn90


class ActionSequence:
    def __init__(self, steps, start_time):
        self.steps = list(steps)
        self.index = 0
        self.action = None
        self.wait_until = start_time

        self.complete = False
        self.fault = None
        self.status = "Starting sequence"

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

        if now < self.wait_until:
            self.status = "Pausing between actions"
            return 0.0, 0.0

        if self.index >= len(self.steps):
            self.complete = True
            self.status = "SEQUENCE COMPLETE"
            return 0.0, 0.0

        kind, value = self.steps[self.index]

        # Create each action once, with fresh sensor references.
        if self.action is None:
            if kind == "move":
                self.action = MoveForward(
                    target_distance=value,
                    start_time=now,
                    left_position=left_position,
                    right_position=right_position,
                    yaw=yaw,
                )

            elif kind == "turn":
                self.action = Turn90(value, now)

            else:
                self.fault = f"Unknown action: {kind}"
                self.status = self.fault
                return 0.0, 0.0

        if kind == "move":
            left_speed, right_speed = self.action.update(
                now=now,
                left_position=left_position,
                right_position=right_position,
                yaw=yaw,
                front_distance=front_distance,
            )

            self.status = (
                f"Step {self.index + 1}/{len(self.steps)}: "
                f"move {self.action.distance:.3f}/{value:.3f} m"
            )

        else:
            left_speed, right_speed = self.action.update(
                now, yaw
            )

            self.status = (
                f"Step {self.index + 1}/{len(self.steps)}: "
                f"turn {value}; {self.action.status}"
            )

        if self.action.fault:
            self.fault = (
                f"Step {self.index + 1}: {self.action.fault}"
            )
            self.status = self.fault
            return 0.0, 0.0

        if self.action.complete:
            self.index += 1
            self.action = None
            self.wait_until = now + 0.5

            if self.index >= len(self.steps):
                self.complete = True
                self.status = "SEQUENCE COMPLETE"
            else:
                self.status = "Pausing between actions"

            return 0.0, 0.0

        return left_speed, right_speed