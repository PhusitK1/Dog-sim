"""Shared helpers for the Rover X1 Simulator tests."""
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from rover_sim import World  # noqa: E402

FAST = 2000        # simulation speed factor used by the tests

# Programs must import and create the robot exactly like on the real robot.
# Tests that only exercise behavior get this header added by run().
HEADER = "from dobot_quad import *\nrobot = RobotClient()\n"


def cell(i):
    return i * 0.5 + 0.25


def box(i, j, h):
    return {"x": cell(i), "y": cell(j), "w": 0.5, "d": 0.5, "h": h}


def stairs_map():
    """Same layout as the 'Stairs' preset: up 0.10 -> 0.14 -> 0.24, then down again."""
    boxes = []
    for j in range(6, 10):
        for i, h in ((6, .10), (7, .10), (8, .14), (9, .14), (10, .24), (11, .24),
                     (12, .14), (13, .14), (14, .10), (15, .10)):
            boxes.append(box(i, j, h))
    for i in range(4, 18):
        boxes.append(box(i, 5, 0.6))
        boxes.append(box(i, 10, 0.6))
    return {"name": "Stairs", "size": [10, 8], "start": {"x": 1.25, "y": 4, "yaw": 0},
            "boxes": boxes, "goals": [{"x": cell(11), "y": 4}, {"x": 9.25, "y": 4}]}


def empty_map(start=(1.25, 1.25, 0), boxes=(), size=(10, 10)):
    return {"name": "test", "size": list(size),
            "start": {"x": start[0], "y": start[1], "yaw": start[2]},
            "boxes": list(boxes), "goals": []}


def make_world(m=None):
    w = World()
    if m is not None:
        w.set_map(m)
        w.reset_robot()
    w.speed_factor = FAST
    return w


def run(w, code, timeout=20.0, header=True):
    """Run a program to the end and return its log texts. Fails if it does not finish.
    header=True prepends the import + robot creation lines unless the code has its own."""
    if header and "dobot_quad" not in code:
        code = ("from dobot_quad import *\n" if "RobotClient(" in code else HEADER) + code
    assert w.run_code(code), "run_code refused to start"
    t = time.time()
    while w.running:
        if time.time() - t > timeout:
            w.stop()
            raise AssertionError(f"program did not finish within {timeout} s")
        time.sleep(0.005)
    return [entry[2] for entry in w.logs]


def printed(w):
    return [entry[2] for entry in w.logs if entry[1] == "print"]


def errors(w):
    return [entry[2] for entry in w.logs if entry[1] == "error"]
