"""Simulated dobot_quad.RobotClient: names, value ranges and behavior of the real SDK."""
import re
import unittest

from helpers import empty_map, errors, make_world, printed, run


def run_ok(test, code, m=None):
    w = make_world(m or empty_map())
    run(w, code)
    test.assertEqual(errors(w), [], "\n".join(errors(w)))
    return w


class StateTest(unittest.TestCase):
    def test_starts_lying_down(self):
        w = run_ok(self, "print(robot.get_current_state_name())")
        self.assertEqual(printed(w), ["stand_down"])

    def test_set_target_state_is_case_insensitive(self):
        w = run_ok(self, "robot.set_target_state('Balance_Stand')\nprint(robot.get_current_state_name())")
        self.assertEqual(printed(w), ["balance_stand"])

    def test_wheel_only_state_raises_value_error(self):
        w = make_world(empty_map())
        run(w, "robot.set_target_state('drift')")
        self.assertTrue(any("ValueError" in e for e in errors(w)))

    def test_change_mode_toggles_walk_and_run(self):
        w = run_ok(self, "robot.walk()\nrobot.change_mode()\nprint(robot.get_current_state_name())\n"
                         "robot.change_mode()\nprint(robot.get_current_state_name())")
        self.assertEqual(printed(w), ["flying_trot", "walk"])

    def test_premade_robot_has_obstacle_avoidance_like_a_real_client(self):
        # RobotClient() turns avoidance on; the ready-made `robot` must behave the same
        w = run_ok(self, "print(robot.get_obstacle_avoidance())")
        self.assertEqual(printed(w), ["True"])


class MotionTest(unittest.TestCase):
    def test_distance_is_clamped_to_3_m(self):
        w = run_ok(self, "robot.walk_forward(5)", empty_map(start=(1.25, 5, 0)))
        self.assertAlmostEqual(w.r["x"], 4.25, places=2)

    def test_speed_ratio_override_is_temporary(self):
        w = run_ok(self, "robot.set_speed_ratio(30)\nrobot.walk_forward(1, speed_ratio=100)\nprint(robot.get_speed_ratio())")
        self.assertEqual(printed(w), ["30"])

    def test_rotate_accepts_0_and_1(self):
        w = run_ok(self, "robot.rotate(0, 90)\nrobot.rotate(1, 45)\n"
                         "import math\nprint(round(math.degrees(robot.get_state().robot_state.ori_body[2])))")
        self.assertEqual(printed(w), ["45"])

    def test_rotate_rejects_other_numbers(self):
        for bad in ("2", "-1"):
            w = make_world(empty_map())
            run(w, f"robot.rotate({bad}, 90)")
            errs = errors(w)
            self.assertTrue(errs and "ValueError" in errs[0], f"rotate({bad}, 90) -> {errs}")
            self.assertAlmostEqual(w.r["yaw"], 0.0, places=3)

    def test_thai_direction_is_fixed_in_the_code_by_autocorrect(self):
        # the real SDK raises ValueError for "ซ้าย", so Auto-correct rewrites the source
        from autocorrect import correct
        code, _, _ = correct("robot.rotate('ซ้าย', 90)\nimport math\n"
                             "print(round(math.degrees(robot.get_state().robot_state.ori_body[2])))")
        self.assertIn("robot.rotate('left', 90)", code)
        w = run_ok(self, code)
        self.assertEqual(printed(w), ["90"])

    def test_velocity_sequence_bad_step_is_a_clear_error(self):
        w = make_world(empty_map())
        run(w, "robot.velocity_sequence([(0.5, 0, 0)])")
        self.assertTrue(any("4 values" in e for e in errors(w)), errors(w))

    def test_velocity_sequence_stands_down_by_default(self):
        w = run_ok(self, "robot.velocity_sequence([(0.5, 0, 0, 1)])\nprint(robot.get_current_state_name())")
        self.assertEqual(printed(w), ["stand_down"])

    def test_execute_with_tuple_and_string(self):
        w = run_ok(self, "robot.execute(('line_walk', {'direction': 0, 'distance': 1}), 'balance_stand')\n"
                         "print(robot.get_current_state_name())", empty_map(start=(1.25, 5, 0)))
        self.assertEqual(printed(w), ["balance_stand"])
        self.assertAlmostEqual(w.r["x"], 2.25, places=2)

    def test_rotate_walk_positive_angle_turns_right(self):
        w = run_ok(self, "robot.rotate_walk(90, 1)", empty_map(start=(5, 5, 0)))
        self.assertAlmostEqual(w.r["y"], 4.0, places=2)


class BalanceAndLedTest(unittest.TestCase):
    def test_balance_values_are_clamped(self):
        w = run_ok(self, "robot.balance_stand()\nrobot.balance_pitch(20, 1.0)")
        self.assertAlmostEqual(w.r["bal"][1], 11.5, places=3)

    def test_balance_from_walk_switches_to_balance_stand(self):
        w = run_ok(self, "robot.walk()\nrobot.balance_roll(5, 1.0)\nprint(robot.get_current_state_name())")
        self.assertEqual(printed(w), ["balance_stand"])

    def test_led_colors_are_valid_hex(self):
        w = run_ok(self, "robot.set_leg_rgb(Leg.FL, 255, 0, 128)\nrobot.set_leg_brightness(Leg.RR, 999)")
        for c in w.r["leg_led"]:
            self.assertRegex(c, r"^#[0-9a-f]{6}$")
        self.assertEqual(w.r["leg_led"][0], "#ff0080")

    def test_led_color_and_reset(self):
        w = run_ok(self, "robot.set_all_legs_color(Color.GREEN)\nprint(robot.reset_legs())")
        self.assertIsNone(w.r["leg_led"])

    def test_dance_duration_must_be_in_range(self):
        w = make_world(empty_map())
        run(w, "robot.dance(duration=1)")
        self.assertTrue(errors(w) and "ValueError" in errors(w)[0], errors(w))


class TelemetryTest(unittest.TestCase):
    def test_get_state_fields(self):
        w = run_ok(self, "s = robot.get_state()\nprint(s.success, s.current_state, len(s.robot_state.pos_body),"
                         " len(s.robot_state.ori_body), len(s.robot_state.jpos_leg))")
        self.assertEqual(printed(w), ["True stand_down 3 3 12"])

    def test_get_motions_lists_line_walk(self):
        w = run_ok(self, "print('line_walk' in [m.motion_id for m in robot.get_motions().motions])")
        self.assertEqual(printed(w), ["True"])

    def test_misspelled_method_fails_at_run_time_with_a_hint(self):
        # the real RobotClient raises AttributeError; Auto-correct fixes it in the source instead
        w = make_world(empty_map())
        run(w, "robot.walkForward(1)")
        self.assertTrue(errors(w) and re.search(r"AttributeError.*walk_forward", errors(w)[0]), errors(w))


if __name__ == "__main__":
    unittest.main()
