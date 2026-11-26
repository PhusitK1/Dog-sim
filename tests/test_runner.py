"""The program runner: stopping, errors, built-ins that could hang the simulator."""
import time
import unittest

from helpers import HEADER, empty_map, errors, make_world, printed, run


class RunnerTest(unittest.TestCase):
    def test_stop_ends_an_infinite_loop(self):
        w = make_world(empty_map())
        self.assertTrue(w.run_code("while True:\n    x = 1"))
        time.sleep(0.2)
        w.stop()
        t = time.time()
        while w.running and time.time() - t < 2:
            time.sleep(0.01)
        self.assertFalse(w.running)
        self.assertIn("■ Program stopped", [e[2] for e in w.logs])

    def test_time_sleep_can_be_stopped(self):
        # real-robot programs use time.sleep(); Stop must still work
        w = make_world(empty_map())
        w.speed_factor = 1
        self.assertTrue(w.run_code("import time\ntime.sleep(30)\nprint('after')"))
        time.sleep(0.3)
        w.stop()
        t = time.time()
        while w.running and time.time() - t < 3:
            time.sleep(0.01)
        self.assertFalse(w.running, "time.sleep(30) ignored the Stop button")
        self.assertEqual(printed(w), [])

    def test_time_sleep_follows_sim_speed(self):
        w = make_world(empty_map())
        t = time.time()
        run(w, "import time\ntime.sleep(5)\nprint('done')", timeout=8)
        self.assertLess(time.time() - t, 3.0)
        self.assertEqual(printed(w), ["done"])

    def test_input_does_not_hang_the_simulator(self):
        w = make_world(empty_map())
        run(w, "n = input('how many? ')", timeout=3)
        self.assertTrue(errors(w) and "input" in errors(w)[0], errors(w))

    def test_exit_is_reported(self):
        w = make_world(empty_map())
        run(w, "print('a')\nexit()\nprint('b')", timeout=3)
        self.assertEqual(printed(w), ["a"])
        self.assertTrue(any("exit" in e[2].lower() for e in w.logs if e[1] == "sys"),
                        [e[2] for e in w.logs])

    def test_syntax_error_is_reported_with_line(self):
        w = make_world(empty_map())
        run(w, "robot.walk_forward(1\n", header=False)
        self.assertTrue(errors(w) and "line 1" in errors(w)[0], errors(w))

    def test_removed_rover_helpers_give_a_hint(self):
        w = make_world(empty_map())
        run(w, "rover.walk(2)")
        self.assertTrue(errors(w) and "rover.* commands were removed" in errors(w)[0])

    def test_dobot_quad_import_forms(self):
        w = make_world(empty_map())
        run(w, "import dobot_quad as dq\nfrom dobot_quad import *\n"
               "r = dq.RobotClient('1.2.3.4:50051')\nprint(r.is_quad(), Color.RED.name, Leg.FL.value)")
        self.assertEqual(errors(w), [])
        self.assertEqual(printed(w), ["True RED 1"])

    def test_with_statement(self):
        w = make_world(empty_map())
        run(w, "from dobot_quad import RobotClient\nwith RobotClient() as r:\n    print(r.get_robot_type())")
        self.assertEqual(printed(w), ["miniQuad"])

    def test_print_sep_and_end(self):
        w = make_world(empty_map())
        run(w, "print(1, 2, sep='-', end='!')")
        self.assertEqual(printed(w), ["1-2!"])

    def test_execute_loop_can_be_stopped(self):
        for code in ("robot.execute('get_state', loop=True)", "robot.execute(loop=True)"):
            w = make_world(empty_map())
            self.assertTrue(w.run_code(HEADER + code))
            time.sleep(0.3)
            w.stop()
            t = time.time()
            while w.running and time.time() - t < 2:
                time.sleep(0.01)
            self.assertFalse(w.running, f"{code} could not be stopped")

    def test_second_run_is_refused_while_running(self):
        w = make_world(empty_map())
        self.assertTrue(w.run_code("while True:\n    pass"))
        self.assertFalse(w.run_code("print(1)"))
        w.stop()


if __name__ == "__main__":
    unittest.main()
