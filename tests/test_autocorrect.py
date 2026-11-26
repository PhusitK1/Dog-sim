"""Auto-correct: fixes what it should, never breaks valid code, never crashes."""
import os
import random
import re
import unittest

from helpers import ROOT

from autocorrect import correct

# what a complete real-robot program starts with
H = 'from dobot_quad import RobotClient\nrobot = RobotClient("192.168.5.2:50051")\n'
HC = 'from dobot_quad import RobotClient, Color\nrobot = RobotClient("192.168.5.2:50051")\n'


def fixes(changes):
    return [c for c in changes if not c.get("warning")]


def warnings(changes):
    return [c for c in changes if c.get("warning")]


def load_examples():
    src = open(os.path.join(ROOT, "static", "examples.js"), encoding="utf-8").read()
    return dict(re.findall(r'"(\d+\. [^"]+)": `(.*?)`,', src, re.S))


class ExamplesTest(unittest.TestCase):
    def test_examples_are_clean_except_the_typo_demo(self):
        ex = load_examples()
        self.assertGreaterEqual(len(ex), 9)
        for name, code in ex.items():
            new, ch, err = correct(code)
            self.assertIsNone(err, name)
            self.assertEqual(warnings(ch), [], name)
            if "Typos" in name:
                self.assertGreaterEqual(len(fixes(ch)), 8)
            else:
                self.assertEqual(ch, [], f"{name}: {ch}")
                self.assertEqual(new, code)


class FixesTest(unittest.TestCase):
    def check(self, src, expected, header=H):
        new, ch, err = correct(header + src)
        self.assertEqual(new.strip(), (header + expected).strip(), ch)
        self.assertIsNone(err)

    def test_sdk_typos(self):
        self.check("robot.walk_forwrd(2, sped_ratio=80)", "robot.walk_forward(2, speed_ratio=80)")
        self.check("robot.balanceStand()", "robot.balance_stand()")
        self.check("robot.walk_left(1)", "robot.move_left(1)")
        self.check("Robot.jump()", "robot.jump()")

    def test_imports_and_constants(self):
        self.check("from dobotquad import RobotClinet, Colour\nr = RobotClinet()\nr.set_all_legs_color(Colour.GREN)",
                   "from dobot_quad import RobotClient, Color\nr = RobotClient()\nr.set_all_legs_color(Color.GREEN)",
                   header="")

    def test_missing_colon_and_paren(self):
        self.check("for i in range(2)\n    robot.jump()", "for i in range(2):\n    robot.jump()")
        self.check("robot.walk_forward(1", "robot.walk_forward(1)")

    def test_missing_robot_prefix(self):
        self.check("walk_forward(1)", "robot.walk_forward(1)")

    def test_thai_keyboard_digits_and_notes(self):
        self.check("พนินะใไฟสา๘ดนพไฟพก๖-๗", "robot.walk_forward(3)")
        self.check("robot.walk_forward(๓)", "robot.walk_forward(3)")
        self.check("robot.stand_down() หมอบ", "robot.stand_down()  # หมอบ")


class StringValuesTest(unittest.TestCase):
    """The real SDK raises ValueError for these, so they are fixed in the source code."""

    def check(self, src, expected, header=H, expected_header=None):
        new, ch, err = correct(header + src)
        self.assertEqual(new.strip(), ((expected_header or header) + expected).strip(), ch)

    def test_direction_gait_state_action_mode_switch(self):
        self.check("robot.rotate('lefft', 90)", "robot.rotate('left', 90)")
        self.check('robot.rotate("ซ้าย", 90)', 'robot.rotate("left", 90)')
        self.check("robot.line_walk('foward', 1)", "robot.line_walk('forward', 1)")
        self.check("robot.velocity_sequence([(0.3, 0, 0, 1)], gait='flying_trott')",
                   "robot.velocity_sequence([(0.3, 0, 0, 1)], gait='flying_trot')")
        self.check("robot.set_target_state('balance_stnd')", "robot.set_target_state('balance_stand')")
        self.check("robot.atomic_action('grove')", "robot.atomic_action('groove')")
        self.check("robot.balance_pitch(5, 1.0, 'dinamic')", "robot.balance_pitch(5, 1.0, 'dynamic')")
        self.check("robot.balance_pitch(5, mode='Static')", "robot.balance_pitch(5, mode='static')")
        self.check("robot.set_obstacle_avoidance('of')", "robot.set_obstacle_avoidance('off')")

    def test_text_color_becomes_a_color_constant(self):
        self.check("robot.set_all_legs_color('green')", "robot.set_all_legs_color(Color.GREEN)", expected_header=HC)
        self.check("robot.set_all_legs_color('แดง')", "robot.set_all_legs_color(Color.RED)", expected_header=HC)

    def test_valid_values_are_left_alone(self):
        for src in ("robot.rotate('Left', 90)", "robot.line_walk('front', 1)", "print('lefft')",
                    "robot.set_target_state('emergency')"):
            new, ch, err = correct(H + src)
            self.assertEqual(new, H + src, ch)

    def test_unknown_value_is_reported(self):
        new, ch, err = correct(H + "robot.rotate('up', 90)")
        self.assertEqual(len(warnings(ch)), 1)
        self.assertIn("left", warnings(ch)[0]["reason"])


class MissingHeaderTest(unittest.TestCase):
    """On the real robot nothing exists until it is imported / created."""

    def test_robot_creation_and_import_are_added(self):
        new, ch, err = correct("robot.walk_forward(1)\n")
        self.assertEqual(new, H + "robot.walk_forward(1)\n")

    def test_added_after_leading_comments(self):
        new, ch, err = correct("# my program\n\nrobot.jump()\n")
        self.assertEqual(new, "# my program\n\n" + H + "robot.jump()\n")

    def test_missing_names_join_the_existing_import(self):
        src = 'from dobot_quad import RobotClient\nrobot = RobotClient("1.2.3.4:50051")\nrobot.set_all_legs_color(Color.RED)\n'
        new, ch, err = correct(src)
        self.assertEqual(new, src.replace("import RobotClient\n", "import RobotClient, Color\n"))

    def test_math_and_time_imports(self):
        self.assertEqual(correct("print(math.pi)\n")[0], "import math\nprint(math.pi)\n")
        self.assertEqual(correct("time.sleep(1)\n")[0], "import time\ntime.sleep(1)\n")

    def test_nothing_added_when_not_needed(self):
        for src in ("x = 1\nprint(x)\n", "import math\nprint(math.pi)\n",
                    "from dobot_quad import *\nrobot = RobotClient()\nrobot.set_all_legs_color(Color.RED)\n"):
            new, ch, err = correct(src)
            self.assertEqual(new, src, ch)

    def test_other_client_name_only_needs_the_import(self):
        new, ch, err = correct("r = RobotClient()\nr.jump()\n")
        self.assertEqual(new, "from dobot_quad import RobotClient\nr = RobotClient()\nr.jump()\n")


class LeaveAloneTest(unittest.TestCase):
    """Valid code must come back unchanged."""

    def same(self, src):
        new, ch, err = correct(src)
        self.assertEqual(new, src, ch)
        self.assertEqual(ch, [])

    def test_thai_variables_strings_and_comments(self):
        self.same(H + "ระยะ = 2\nrobot.walk_forward(ระยะ)  # เดินหน้า ๒ เมตร\nprint(\"ระยะ\", ระยะ)\n")

    def test_strings_and_fstrings(self):
        self.same(H + 'print("robot.walk_forwrd(3)")\nprint(f"{robot.get_speed_ratio()} %")\n')

    def test_curly_quotes_inside_strings_and_comments(self):
        self.same(H + 'print("He said “hi” and ‘bye’")\n# a “quoted” word\n')

    def test_thai_in_a_docstring(self):
        self.same('"""\nโปรแกรมเดินหน้า ๓ เมตร\nพนินะ\n"""\n' + H + "robot.walk_forward(3)\n")

    def test_import_with_a_comment_and_multiline_import(self):
        self.same('from dobot_quad import RobotClient  # the client\nrobot = RobotClient("192.168.5.2:50051")\n')
        self.same('from dobot_quad import (\n    RobotClient,\n    Color,\n)\nrobot = RobotClient()\n'
                  'robot.set_all_legs_color(Color.RED)\n')

    def test_real_sdk_keyword_arguments(self):
        self.same(HC + "robot.walk_forward(1, show_progress=False)\nrobot.set_all_legs_color(Color.RED, verbose=True)\n")

    def test_robot_reassigned_to_something_else(self):
        self.same('robot = "Rover"\nprint(robot.upper())\n')

    def test_except_variable(self):
        self.same(H + "try:\n    robot.jump()\nexcept Exception as err:\n    print(err.args)\n")

    def test_video_attribute(self):
        self.same(H + "print(robot.video)\n")

    def test_user_function_with_an_sdk_name(self):
        self.same("def jump():\n    print('mine')\njump()\n")

    def test_multiline_call(self):
        self.same(H + "robot.walk_forward(\n    1.0,\n    speed_ratio=50,\n)\n")

    def test_unknown_names_are_not_turned_into_robot_commands(self):
        # `run` / `stop` are only aliases - guessing robot.flying_trot() / robot.ready()
        # would silently change what the program does
        for src in ("run()\n", "stop()\n", "turn(90)\n"):
            new, ch, err = correct(src)
            self.assertEqual(new, src, f"{src.strip()} was changed to {new.strip()}")

    def test_idempotent(self):
        for code in load_examples().values():
            once, _, _ = correct(code)
            twice, ch, _ = correct(once)
            self.assertEqual(twice, once)
            self.assertEqual(fixes(ch), [])


class StillFixedTest(unittest.TestCase):
    def test_curly_quotes_used_as_string_delimiters(self):
        new, ch, err = correct(H + "print(“hi”)\nprint(“it's”)\n")
        self.assertEqual(new, H + "print(\"hi\")\nprint(\"it's\")\n")
        self.assertIsNone(err)

    def test_attribute_that_the_real_client_does_not_have(self):
        new, ch, err = correct(H + "print(robot.addr)\n")
        self.assertEqual(len(warnings(ch)), 1)


class RemovedHelpersTest(unittest.TestCase):
    def test_rover_and_dog_are_reported(self):
        new, ch, err = correct("rover.walk(2)\ndog.turn_left(90)\n")
        w = warnings(ch)
        self.assertEqual(len(w), 2)
        self.assertIn("walk_forward", w[0]["reason"])
        self.assertIn("rotate_left", w[1]["reason"])

    def test_own_variable_named_rover_is_fine(self):
        new, ch, err = correct("from dobot_quad import RobotClient\nrover = RobotClient()\nrover.walk_forward(1)\n")
        self.assertEqual(ch, [])


class FuzzTest(unittest.TestCase):
    def test_never_crashes_on_random_input(self):
        rnd = random.Random(1234)
        alphabet = ("abcdefghijklmnopqrstuvwxyz_ ().:,'\"#=+-*/\n\t0123456789[]{}"
                    "พนินะใไฟสาดกเ้่๑๒๓“”（）robot.walk_forward")
        snippets = ["robot.", "rover.", "for i in range(3)", "    ", "if ", "else", "def f(", "Color.",
                    "from dobot_quad import ", "print(", "\"", "'''", "\"\"\"", "f\"{", "\\", "(", ")",
                    "robot.rotate('", "robot.set_all_legs_color(", "math.", "time."]
        for n in range(600):
            parts = []
            for _ in range(rnd.randint(1, 12)):
                if rnd.random() < 0.4:
                    parts.append(rnd.choice(snippets))
                else:
                    parts.append("".join(rnd.choice(alphabet) for _ in range(rnd.randint(1, 15))))
            src = "".join(parts)
            try:
                new, ch, err = correct(src)
            except Exception as e:  # noqa: BLE001
                self.fail(f"correct() crashed on {src!r}: {type(e).__name__}: {e}")
            self.assertIsInstance(new, str)


if __name__ == "__main__":
    unittest.main()
