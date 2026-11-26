"""Movement, collisions, stairs and obstacle avoidance."""
import threading
import time
import unittest

from helpers import box, empty_map, errors, make_world, printed, run, stairs_map


class StairsTest(unittest.TestCase):
    def test_two_walks_go_up_and_over_the_stairs(self):
        # the bug the user found: walk_forward(3) twice got stuck inside the stairs
        w = make_world(stairs_map())
        run(w, "robot.balance_stand()\nprint(robot.walk_forward(3).success)\nprint(robot.walk_forward(3).success)")
        self.assertEqual(printed(w), ["True", "True"])
        self.assertAlmostEqual(w.r["x"], 7.25, places=2)

    def test_round_trip_returns_to_start(self):
        w = make_world(stairs_map())
        run(w, "robot.balance_stand()\nfor d in (3, 3, 2):\n    robot.walk_forward(d)\n"
               "robot.rotate_left(180)\nfor d in (3, 3, 2):\n    print(robot.walk_forward(d).success)")
        self.assertEqual(printed(w), ["True"] * 3)
        self.assertAlmostEqual(w.r["x"], 1.25, places=1)
        self.assertEqual(w.r["z"], 0.0)

    def test_never_inside_a_step_while_walking(self):
        m = stairs_map()
        w = make_world(m)
        samples = []
        stop = threading.Event()

        def sample():
            while not stop.is_set():
                samples.append((w.r["x"], w.r["y"], w.r["z"]))
                time.sleep(0.0005)
        threading.Thread(target=sample, daemon=True).start()
        run(w, "robot.balance_stand()\nrobot.walk_forward(3)\nrobot.walk_forward(3)")
        stop.set()
        for x, y, z in samples:
            for b in m["boxes"]:
                inside = abs(x - b["x"]) < b["w"] / 2 and abs(y - b["y"]) < b["d"] / 2
                self.assertFalse(inside and b["h"] > z + 0.01, f"robot center inside a step at x={x:.2f} z={z}")

    def test_too_high_step_from_the_ground_is_blocked(self):
        w = make_world(empty_map(start=(1.25, 1.25, 0), boxes=[box(6, 2, 0.24)]))
        run(w, "robot.set_obstacle_avoidance(False)\nprint(robot.walk_forward(3).success)")
        self.assertEqual(printed(w), ["False"])
        self.assertEqual(w.r["z"], 0.0)


class CollisionTest(unittest.TestCase):
    def test_wall_stops_the_robot_without_overlap(self):
        w = make_world(empty_map(boxes=[box(8, 2, 0.6)]))
        run(w, "robot.set_obstacle_avoidance(False)\nprint(robot.walk_forward(3).success)")
        self.assertEqual(printed(w), ["False"])
        self.assertGreater(w.clearance(w.r["x"], w.r["y"], w.r["z"])[0], -0.02)

    def test_can_turn_and_leave_after_hitting_a_wall(self):
        w = make_world(empty_map(boxes=[box(8, 2, 0.6)]))
        run(w, "robot.set_obstacle_avoidance(False)\nrobot.walk_forward(3)\n"
               "robot.rotate_left(180)\nprint(robot.walk_forward(1).success)")
        self.assertEqual(printed(w), ["True"])

    def test_field_edge_stops_the_robot(self):
        w = make_world(empty_map(start=(8.5, 1.25, 0)))
        run(w, "robot.set_obstacle_avoidance(False)\nprint(robot.walk_forward(3).success)")
        self.assertEqual(printed(w), ["False"])
        self.assertLessEqual(w.r["x"], 10 - 0.32 + 0.01)

    def test_robot_inside_a_big_box_cannot_walk_through_it(self):
        # e.g. the start point was placed on a wall in the map editor
        big = {"x": 5.0, "y": 5.0, "w": 2.0, "d": 2.0, "h": 0.6}
        w = make_world(empty_map(start=(4.6, 5.0, 0), boxes=[big]))
        run(w, "robot.set_obstacle_avoidance(False)\nprint(robot.walk_forward(1).success)")
        self.assertEqual(printed(w), ["False"], "walking deeper into a wall must be blocked")
        self.assertLess(w.r["x"], 4.7)

    def test_robot_inside_a_box_can_get_out(self):
        big = {"x": 5.0, "y": 5.0, "w": 2.0, "d": 2.0, "h": 0.6}
        w = make_world(empty_map(start=(4.6, 5.0, 180), boxes=[big]))
        run(w, "robot.set_obstacle_avoidance(False)\nprint(robot.walk_forward(1).success)")
        self.assertEqual(printed(w), ["True"])


class LedgeTest(unittest.TestCase):
    def test_cannot_walk_off_a_high_ledge(self):
        # climb 0.10 -> 0.20 -> 0.30 (each step +0.10), then the far side drops 0.30 m to the floor
        w = make_world(empty_map(boxes=[box(4, 2, 0.10), box(5, 2, 0.20), box(6, 2, 0.30), box(7, 2, 0.30)]))
        run(w, "robot.set_obstacle_avoidance(False)\nprint(robot.walk_forward(3).success)\n"
               "print(robot.walk_forward(3).success)")
        self.assertEqual(printed(w), ["True", "False"])
        self.assertAlmostEqual(w.r["z"], 0.30, places=3)

    def test_ground_removed_under_the_robot_makes_it_fall(self):
        w = make_world(empty_map(start=(2.25, 1.25, 0), boxes=[box(4, 2, 0.10)]))
        self.assertAlmostEqual(w.r["z"], 0.10, places=3)
        w.set_map(empty_map(start=(2.25, 1.25, 0)))         # platform deleted in the editor, no reset
        w.r["z"] = 0.5                                      # robot was standing on a high platform
        run(w, "print(robot.walk_forward(1).success)")
        self.assertEqual(printed(w), ["True"])
        self.assertEqual(w.r["z"], 0.0)

    def test_reported_height_follows_the_posture(self):
        w = make_world(empty_map())
        run(w, "print(round(robot.get_state().robot_state.pos_body[2], 2))\nrobot.balance_stand()\n"
               "print(round(robot.get_state().robot_state.pos_body[2], 2))")
        lying, standing = (float(p) for p in printed(w))
        self.assertLess(lying, 0.2)
        self.assertAlmostEqual(standing, 0.28, places=2)


class AvoidanceTest(unittest.TestCase):
    def test_robotclient_stops_before_the_wall(self):
        w = make_world(empty_map(boxes=[box(8, 2, 0.6)]))
        run(w, "r = RobotClient()\nprint(r.walk_forward(3).success)")
        self.assertEqual(printed(w), ["False"])
        c = w.clearance(w.r["x"], w.r["y"], w.r["z"])[0]
        self.assertTrue(0 <= c <= 0.13, f"clearance {c:.3f}")

    def test_flags_are_collected(self):
        w = make_world(stairs_map())
        run(w, "robot.balance_stand()\nfor d in (3, 3, 2):\n    robot.walk_forward(d)")
        self.assertEqual(w.reached, {0, 1})
        self.assertEqual(errors(w), [])


if __name__ == "__main__":
    unittest.main()
