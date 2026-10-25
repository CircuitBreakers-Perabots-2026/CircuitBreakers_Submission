"""Entry point for the PeraBots full course and dedicated section attempts."""

import sys

from competition import parse_mode, run_course

TIME_STEP = 16


def main():
    mode = parse_mode("full", sys.argv[1:])
    from controller import Robot

    run_course(Robot(), TIME_STEP, mode)


if __name__ == "__main__":
    main()
