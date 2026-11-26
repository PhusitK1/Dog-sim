"""Poka-yoke: the simulator must be at least as strict as the REAL dobot_quad SDK.

Anything the real SDK rejects must also fail here (with a helpful hint), otherwise a
program passes in the simulator and crashes on the robot. Rules taken from
Dobot-Team/dobot_quad_sdk high_level/python/dobot_quad/robot_client.py.
"""
import unittest

from helpers import empty_map, errors, make_world, printed, run


class StrictValuesTest(unittest.TestCase):
    def expect_error(self, code, exc="ValueError", contains=None):
        w = make_world(empty_map())
        run(w, code)
        errs = errors(w)
        self.assertTrue(errs and exc in errs[0], f"{code!r} -> {errs}")
        if contains:
            self.assertIn(contains, errs[0])
        return w

    def run_ok(self, code):
        w = make_world(empty_map())
        run(w, code)
        self.assertEqual(errors(w), [], f"{code!r} -> {errors(w)}")
        return w

    def test_misspelled_direction_fails_like_real(self):
        w = self.expect_error("robot.rotate('lefft', 90)", contains="left")
        self.assertAlmostEqual(w.r["yaw"], 0.0, places=3)

    def test_thai_direction_fails_at_run_time(self):
        self.expect_error("robot.rotate('ซ้าย', 90)", contains="left")

    def test_direction_case_and_spaces_are_ok_like_real(self):
        w = self.run_ok("robot.rotate(' Left ', 90)")
        self.assertAlmostEqual(w.r["yaw"], 1.5708, places=3)

    def test_line_walk_direction_must_be_exact(self):
        self.expect_error("robot.line_walk('fwd', 1)")
        self.expect_error("robot.line_walk('2', 1)")

    def test_rotate_float_direction_is_truncated_like_real(self):
        w = self.run_ok("robot.rotate(1.7, 90)")
        self.assertAlmostEqual(w.r["yaw"], -1.5708, places=3)

    def test_gait_state_and_action_typos_fail(self):
        self.expect_error("robot.velocity_sequence([(0.3, 0, 0, 1)], gait='flying_trott')")
        self.expect_error("robot.set_target_state('balance_stnd')")
        self.expect_error("robot.atomic_action('grove')")

    def test_balance_mode_typo_fails(self):
        self.expect_error("robot.balance_pitch(5, 1.0, 'dinamic')")

    def test_obstacle_avoidance_accepts_only_bool_or_on_off(self):
        self.expect_error("robot.set_obstacle_avoidance('yes')")
        w = self.run_ok("robot.set_obstacle_avoidance('OFF')\nprint(robot.get_obstacle_avoidance())")
        self.assertEqual(printed(w), ["False"])

    def test_method_typo_fails_with_a_hint(self):
        self.expect_error("robot.walkForward(1)", exc="AttributeError", contains="walk_forward")

    def test_dance_duration_out_of_range_fails(self):
        self.expect_error("robot.dance(duration=1)")
        self.expect_error("robot.dance(duration=200)")

    def test_negative_wave_duration_fails(self):
        self.expect_error("robot.wave_hand(-1)")

    def test_balance_sequence_needs_exact_ids_and_4_values(self):
        self.expect_error("robot.balance_sequence([('balance_pich', 5, 1.0, 'dynamic')])")
        self.expect_error("robot.balance_sequence([('balance_pitch', 5)])")


class LedLikeRealTest(unittest.TestCase):
    def leds(self, code):
        w = make_world(empty_map())
        run(w, code)
        return w

    def test_rgb_out_of_range_fails(self):
        w = self.leds("robot.set_leg_rgb(Leg.FL, 300, 0, 0)")
        self.assertTrue(errors(w) and "ValueError" in errors(w)[0], errors(w))

    def test_brightness_is_clamped_like_real(self):
        w = self.leds("robot.set_leg_brightness(Leg.RR, 999)")
        self.assertEqual(errors(w), [])
        self.assertEqual(w.r["leg_led"][3], "#ffffff")

    def test_docs_style_set_legs_rgb_fails_like_real(self):
        # robot.set_legs_rgb([Leg.FL, Leg.FR], Color.BLUE) is shown in the docs but the
        # real code reads cfg.leg on every item -> AttributeError on the robot
        w = self.leds("robot.set_legs_rgb([Leg.FL, Leg.FR], Color.BLUE)")
        self.assertTrue(errors(w), "must fail like on the real robot")
        self.assertIn("LegLedConfig", errors(w)[0])

    def test_set_legs_rgb_with_configs(self):
        w = self.leds("robot.set_legs_rgb([LegLedConfig(Leg.FL, 0, 0, 255), LegLedConfig(leg=Leg.RR, r=255)])")
        self.assertEqual(errors(w), [])
        self.assertEqual(w.r["leg_led"], ["#0000ff", "#000000", "#000000", "#ff0000"])

    def test_text_color_and_leg_are_rejected(self):
        # real SDK: a text color silently turns the light OFF, a text leg raises ValueError
        for code in ("robot.set_all_legs_color('green')", "robot.set_leg_color('fl', Color.RED)"):
            w = self.leds(code)
            self.assertTrue(errors(w) and "ValueError" in errors(w)[0], f"{code}: {errors(w)}")

    def test_fill_lights_exist_but_cannot_be_set(self):
        w = self.leds("robot.set_leg_color(Leg.FILL_FRONT, Color.RED)")
        self.assertTrue(errors(w) and "ValueError" in errors(w)[0], errors(w))
        w = self.leds("print(robot.reset_legs([Leg.FILL_FRONT, Leg.FL]))")
        self.assertEqual(errors(w), [])

    def test_preset_colors_match_the_real_sdk(self):
        w = self.leds("robot.set_leg_color(Leg.FL, Color.ORANGE)\nrobot.set_leg_color(Leg.FR, Color.YELLOW)\n"
                      "robot.set_leg_color(Leg.RL, Color.PURPLE)\nrobot.set_leg_color(Leg.RR, Color.CYAN)")
        self.assertEqual(w.r["leg_led"], ["#ffa500", "#ffff00", "#800080", "#00ffff"])


class ExportsTest(unittest.TestCase):
    def test_everything_the_real_package_exports_can_be_imported(self):
        w = make_world(empty_map())
        run(w, "from dobot_quad import (RobotClient, Leg, Color, LegLedConfig, VALID_STATES,\n"
               "    VALID_BALANCE_MOTIONS, VALID_GAITS, VALID_WHEEL_STATES, VALID_WHEEL_GAITS,\n"
               "    VALID_ATOMIC_ACTIONS, CameraId, CameraStreamInfo, StreamEvent, StreamState,\n"
               "    VideoConnectionError, VideoError, VideoTimeoutError)\n"
               "print(sorted(VALID_GAITS), 'balance_stand' in VALID_STATES)")
        self.assertEqual(errors(w), [])
        self.assertEqual(printed(w), ["['flying_trot', 'walk'] True"])

    def test_no_simulator_only_attributes(self):
        w = make_world(empty_map())
        run(w, "print(hasattr(robot, 'addr'))")
        self.assertEqual(printed(w), ["False"])


class NothingPreloadedTest(unittest.TestCase):
    """On the real robot nothing exists until you import / create it."""

    def expect_name_error(self, code, hint):
        w = make_world(empty_map())
        run(w, code, header=False)
        errs = errors(w)
        self.assertTrue(errs and "NameError" in errs[0], f"{code!r} -> {errs}")
        self.assertIn(hint, errs[0])

    def test_robot_must_be_created(self):
        self.expect_name_error("robot.walk_forward(1)", "RobotClient(")

    def test_sdk_names_must_be_imported(self):
        self.expect_name_error("print(Color.RED)", "from dobot_quad import")

    def test_math_must_be_imported(self):
        self.expect_name_error("print(math.pi)", "import math")


if __name__ == "__main__":
    unittest.main()
