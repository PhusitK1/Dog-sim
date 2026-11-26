"""Run every example program in the simulator, exactly like pressing Run in the app."""
import html
import os
import re
import unittest

from helpers import ROOT, errors, make_world, run, stairs_map
from test_autocorrect import load_examples

from autocorrect import correct
from rover_sim import default_map


def map_for(name):
    # the app loads these maps automatically when the example is chosen
    if "Stairs" in name:
        return stairs_map()
    return default_map()


class ExamplesRunTest(unittest.TestCase):
    def run_example(self, name):
        code, _, err = correct(load_examples()[name])
        self.assertIsNone(err, name)
        w = make_world(map_for(name))
        run(w, code, timeout=60)
        self.assertEqual(errors(w), [], f"{name}: {errors(w)}")
        return w

    def test_every_example_runs_without_errors(self):
        for name in load_examples():
            with self.subTest(example=name):
                self.run_example(name)

    def test_only_example_5_runs_into_obstacles(self):
        # an example that is stopped by obstacle avoidance draws the wrong shape
        for name in load_examples():
            with self.subTest(example=name):
                w = self.run_example(name)
                stops = [e[2] for e in w.logs if "obstacle avoidance: stopped" in e[2] or "Hit the" in e[2]]
                if name.startswith("5."):
                    self.assertTrue(stops, "example 5 is about obstacle avoidance")
                else:
                    self.assertEqual(stops, [], name)

    def test_manual_code_samples_run(self):
        manual = open(os.path.join(ROOT, "static", "manual.html"), encoding="utf-8").read()
        samples = [html.unescape(s) for s in re.findall(r"<pre><code>(.*?)</code>", manual, re.S)]
        self.assertGreaterEqual(len(samples), 7)
        for i, sample in enumerate(samples):
            with self.subTest(sample=i, first_line=sample.splitlines()[0]):
                code, _, err = correct(sample)
                self.assertIsNone(err)
                w = make_world(default_map())
                run(w, code, timeout=60)
                self.assertEqual(errors(w), [])
                stops = [e[2] for e in w.logs if "obstacle avoidance: stopped" in e[2] or "Hit the" in e[2]]
                self.assertEqual(stops, [], "manual sample runs into an obstacle")

    def test_example_4_collects_all_three_flags(self):
        name = next(n for n in load_examples() if n.startswith("4."))
        w = self.run_example(name)
        self.assertEqual(w.reached, {0, 1, 2})

    def test_example_8_climbs_the_stairs_and_collects_both_flags(self):
        name = next(n for n in load_examples() if n.startswith("8."))
        w = self.run_example(name)
        self.assertEqual(w.reached, {0, 1}, "both flags on the Stairs map")
        prints = [e[2] for e in w.logs if e[1] == "print"]
        self.assertEqual(prints[:3], ["True", "True", "True"], "all three walks reach their distance")


if __name__ == "__main__":
    unittest.main()
