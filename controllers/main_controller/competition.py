"""Automatic section selection and a verified continuous-run handoff."""

import argparse

from floor_detection import classify_floor_rgb, read_floor_rgb
from floor_map import consensus


def parse_mode(default, arguments):
    parser = argparse.ArgumentParser(description="PeraBots controller")
    parser.add_argument("--mode", default=default, choices=(
        "full", "auto", "section1", "maze", "navigate", "read_only",
        "approach", "turn_left", "turn_right", "move_cell", "sequence",
    ))
    return parser.parse_args(arguments).mode


def starting_section(robot, time_step):
    """Use the floor under the robot, so dedicated maze attempts also work."""
    camera = robot.getDevice("floor_camera")
    camera.enable(time_step)
    samples = []
    for _ in range(max(5, int(1.0 / (time_step / 1000)))):
        if robot.step(time_step) == -1:
            raise RuntimeError("Simulation ended during section selection")
        rgb = read_floor_rgb(camera)
        label = classify_floor_rgb(rgb)
        # White Section 1 floor can appear grey in the chassis shadow.
        if label == "unknown" and rgb is not None:
            if min(rgb) > 70 and max(rgb) - min(rgb) < 20:
                label = "white"
        samples.append(label)
    label = consensus(samples[-5:])
    if label in ("black", "blue", "green"):
        return "maze"
    if label in ("red", "white"):
        return "section1"
    raise RuntimeError("Cannot identify starting floor; check the camera or use --mode")


def run_course(robot, time_step=16, mode="full"):
    from obstacle_navigation import run_obstacle_field
    from maze_navigation import run_maze

    for name in ("left_wheel_motor", "right_wheel_motor"):
        motor = robot.getDevice(name)
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)
    for name in ("led_blue", "led_green"):
        robot.getDevice(name).set(0)
    try:
        section = starting_section(robot, time_step) if mode in ("full", "auto") else mode
        print(f"COMPETITION | mode={mode} | starting section={section}")
        if section == "maze":
            run_maze(robot, time_step)
            return
        entrance = run_obstacle_field(robot, time_step)
        if entrance is not None and mode != "section1":
            print("CONTINUOUS HANDOFF: verified first cell; starting Section 2.")
            run_maze(robot, time_step, entry_heading=entrance.north_yaw)
    except RuntimeError as error:
        print(f"COMPETITION STOPPED: {error}")
    finally:
        for name in ("left_wheel_motor", "right_wheel_motor"):
            robot.getDevice(name).setVelocity(0.0)
