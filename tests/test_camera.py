"""Simulated robot cameras (robot.video) and the runner features camera programs rely on:
__file__ / sys.argv, grpc, threads started by the program, Stop that lets cleanup run."""
import importlib.util
import sys
import threading
import time
import unittest
import urllib.error
import urllib.request

from helpers import empty_map, errors, make_world, printed, run

import sdk_compat
import server
from rover_sim import WORLD

HAS_CV2 = importlib.util.find_spec("cv2") is not None
TINY_JPEG = b"\xff\xd8\xff\xd9"      # start + end of image: enough for the server and the SDK


class Feeder(threading.Thread):
    """Plays the part of the simulator page: uploads a frame for every camera that is wanted."""

    def __init__(self, world, jpeg=TINY_JPEG):
        super().__init__(daemon=True)
        self.world, self.jpeg = world, jpeg
        self.done = threading.Event()
        self.seen = set()

    def run(self):
        while not self.done.is_set():
            for cam in self.world.cams.wanted():
                self.seen.add(cam)
                self.world.cams.put(cam, self.jpeg)
            time.sleep(0.03)

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *_):
        self.done.set()
        self.join(2)


class CameraApiTest(unittest.TestCase):
    def setUp(self):
        self.w = make_world(empty_map())

    def test_types_match_the_real_sdk(self):
        self.assertEqual((sdk_compat.CameraId.FRONT_RGB, sdk_compat.CameraId.REAR_RGB), (0, 1))
        self.assertEqual(sdk_compat.CameraId.FRONT_RGB.rtsp_name, "camera1")
        self.assertEqual(sdk_compat.CameraId.REAR_RGB.rtsp_name, "camera2")
        self.assertEqual({s.value for s in sdk_compat.StreamState},
                         {"stopped", "preparing", "ready", "recovering", "stopping", "error"})
        e = sdk_compat.VideoTimeoutError("x", reason=sdk_compat.LossReason.NO_DATA,
                                         camera=sdk_compat.CameraId.REAR_RGB)
        self.assertIsInstance(e, RuntimeError)
        self.assertIsInstance(e, sdk_compat.VideoError)
        self.assertEqual((e.reason.value, e.camera), ("no_data", sdk_compat.CameraId.REAR_RGB))

    def test_open_waits_for_frames_and_returns_the_stream_address(self):
        code = ("from dobot_quad import RobotClient, CameraId\n"
                "robot = RobotClient()\n"
                "uri = robot.video.open(CameraId.FRONT_RGB)\n"
                "print(uri)\n"
                "info = robot.video.get_stream_info(CameraId.FRONT_RGB)\n"
                "print(robot.video.is_streaming(CameraId.FRONT_RGB), info.state, info.available)\n"
                "robot.video.close(CameraId.FRONT_RGB)\n"
                "print(robot.video.get_stream_info(CameraId.FRONT_RGB).state)\n")
        with Feeder(self.w) as f:
            run(self.w, code)
        out = printed(self.w)
        self.assertEqual(errors(self.w), [])
        self.assertTrue(out[0].endswith("/video/camera1"), out)
        self.assertEqual(out[1:], ["True ready True", "stopped"])
        self.assertEqual(f.seen, {0})
        self.assertEqual(self.w.cams.wanted(), [])

    def test_open_without_the_page_times_out_like_the_robot(self):
        old = sdk_compat._SimVideo.READY_TIMEOUT
        sdk_compat._SimVideo.READY_TIMEOUT = 0.4
        try:
            run(self.w, "from dobot_quad import RobotClient, CameraId, VideoError, VideoTimeoutError\n"
                        "robot = RobotClient()\n"
                        "try:\n"
                        "    robot.video.open(CameraId.REAR_RGB)\n"
                        "except VideoTimeoutError as e:\n"
                        "    print(isinstance(e, VideoError), e.reason, e.camera.name)\n")
        finally:
            sdk_compat._SimVideo.READY_TIMEOUT = old
        self.assertEqual(printed(self.w), ["True no_data REAR_RGB"])
        self.assertTrue(any("simulator page" in t for t in run(self.w, "pass") if "💡" in t))
        self.assertEqual(self.w.cams.wanted(), [])

    def test_open_twice_needs_close_twice(self):
        code = ("from dobot_quad import RobotClient, CameraId\n"
                "robot = RobotClient()\n"
                "a = robot.video.open(CameraId.FRONT_RGB)\n"
                "b = robot.video.open(CameraId.FRONT_RGB)\n"
                "robot.video.close(CameraId.FRONT_RGB)\n"
                "print(a == b, robot.video.is_streaming(CameraId.FRONT_RGB))\n"
                "robot.video.close(CameraId.FRONT_RGB)\n"
                "robot.video.close(CameraId.FRONT_RGB)\n"
                "print(robot.video.is_streaming(CameraId.FRONT_RGB))\n")
        with Feeder(self.w):
            run(self.w, code)
        self.assertEqual(printed(self.w), ["True True", "False"])

    def test_bad_camera_fails_like_the_real_enum(self):
        run(self.w, "from dobot_quad import RobotClient\nrobot = RobotClient()\nrobot.video.open(5)\n")
        self.assertTrue(any("ValueError" in e for e in errors(self.w)), errors(self.w))

    def test_cameras_are_released_when_the_program_ends(self):
        code = ("from dobot_quad import RobotClient, CameraId\n"
                "robot = RobotClient()\n"
                "robot.video.open(CameraId.FRONT_RGB)\n"
                "robot.video.open(CameraId.REAR_RGB)\n"
                "print(len(robot.video.list_cameras()))\n")
        with Feeder(self.w) as f:
            run(self.w, code)
        self.assertEqual(f.seen, {0, 1})
        self.assertEqual(self.w.cams.wanted(), [])

    def test_robot_close_releases_the_cameras(self):
        code = ("from dobot_quad import RobotClient, CameraId\n"
                "robot = RobotClient()\n"
                "robot.video.open(CameraId.FRONT_RGB)\n"
                "robot.close()\n"
                "print(robot.video.is_streaming(CameraId.FRONT_RGB))\n")
        with Feeder(self.w):
            run(self.w, code)
        self.assertEqual(printed(self.w), ["False"])


class RunnerLikeAScriptTest(unittest.TestCase):
    def setUp(self):
        self.w = make_world(empty_map())

    def test_file_and_argv_belong_to_the_program(self):
        argv = list(sys.argv)
        run(self.w, "import os, sys, argparse\n"
                    "print(os.path.basename(__file__), sys.argv == [__file__])\n"
                    "p = argparse.ArgumentParser()\n"
                    "p.add_argument('address', nargs='?', default='192.168.5.2:50051')\n"
                    "p.add_argument('--no-depth', action='store_true')\n"
                    "a = p.parse_args()\n"
                    "print(a.address, a.no_depth)\n", header=False)
        self.assertEqual(printed(self.w), ["program.py True", "192.168.5.2:50051 False"])
        self.assertEqual(sys.argv, argv)

    def test_grpc_channel_reaches_the_simulated_robot(self):
        run(self.w, "import grpc\n"
                    "ch = grpc.insecure_channel('192.168.5.2:50051')\n"
                    "grpc.channel_ready_future(ch).result(timeout=3.0)\n"
                    "ch.close()\n"
                    "print('ready')\n"
                    "grpc.secure_channel('x', None)\n", header=False)
        self.assertEqual(printed(self.w), ["ready"])
        self.assertTrue(any("secure_channel" in e and "simulator" in e for e in errors(self.w)), errors(self.w))

    def test_daemon_threads_end_with_the_program(self):
        run(self.w, "import threading\n"
                    "def spin():\n"
                    "    n = 0\n"
                    "    while True:\n"
                    "        n += 1\n"
                    "threading.Thread(target=spin, name='spinner', daemon=True).start()\n", header=False)
        time.sleep(0.2)
        self.assertNotIn("spinner", [t.name for t in threading.enumerate() if t.is_alive()])

    def test_non_daemon_threads_keep_the_program_running(self):
        logs = run(self.w, "import threading\n"
                           "def late():\n"
                           "    threading.Event().wait(0.3)\n"
                           "    print('late')\n"
                           "threading.Thread(target=late).start()\n"
                           "print('main done')\n", header=False)
        self.assertEqual(printed(self.w), ["main done", "late"])
        self.assertLess(logs.index("late"), next(i for i, t in enumerate(logs) if "Program finished" in t))

    def test_stop_lets_cleanup_code_run(self):
        self.w.run_code("try:\n"
                        "    while True:\n"
                        "        pass\n"
                        "finally:\n"
                        "    print('cleanup')\n")
        time.sleep(0.3)
        self.w.stop()
        t = time.time()
        while self.w.running and time.time() - t < 5:
            time.sleep(0.01)
        self.assertFalse(self.w.running)
        self.assertIn("cleanup", printed(self.w))
        self.assertTrue(any("Program stopped" in e[2] for e in self.w.logs))

    def test_a_program_that_ignores_stop_is_still_stopped(self):
        self.w.run_code("while True:\n"
                        "    try:\n"
                        "        while True:\n"
                        "            pass\n"
                        "    except Exception:\n"
                        "        pass\n")
        time.sleep(0.2)
        self.w.stop()
        t = time.time()
        while self.w.running and time.time() - t < 8:
            time.sleep(0.01)
        self.assertFalse(self.w.running)

    def test_errors_in_threads_reach_the_console(self):
        run(self.w, "import threading\n"
                    "def bad():\n"
                    "    1 / 0\n"
                    "t = threading.Thread(target=bad, name='worker')\n"
                    "t.start()\n"
                    "t.join()\n", header=False)
        self.assertTrue(any("worker" in e and "ZeroDivisionError" in e for e in errors(self.w)), errors(self.w))


class CameraStreamServerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.srv = server.Server(("127.0.0.1", 0), server.Handler)
        cls.base = f"http://127.0.0.1:{cls.srv.server_address[1]}"
        threading.Thread(target=cls.srv.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()

    def post_frame(self, cam, body, ctype="image/jpeg"):
        req = urllib.request.Request(f"{self.base}/api/frame?cam={cam}", data=body, method="POST",
                                     headers={"Content-Type": ctype})
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status
        except urllib.error.HTTPError as e:
            return e.code

    def test_frames_are_checked(self):
        self.assertEqual(self.post_frame(0, TINY_JPEG), 200)
        self.assertEqual(self.post_frame(0, b"not a jpeg"), 400)
        self.assertEqual(self.post_frame(7, TINY_JPEG), 400)
        self.assertEqual(self.post_frame(0, TINY_JPEG, "text/plain"), 403)      # forms of other sites
        self.assertEqual(self.post_frame(0, TINY_JPEG, "application/json"), 403)

    def test_stream_is_mjpeg_with_the_uploaded_frame(self):
        frame = b"\xff\xd8" + b"rover" * 50 + b"\xff\xd9"
        self.assertEqual(self.post_frame(1, frame), 200)
        with urllib.request.urlopen(self.base + "/video/camera2", timeout=5) as r:
            self.assertIn("multipart/x-mixed-replace", r.headers["Content-Type"])
            head = r.read(len(frame) + 100)
        self.assertIn(b"--frame", head)
        self.assertIn(frame, head)

    def test_unknown_camera_is_404(self):
        with self.assertRaises(urllib.error.HTTPError) as cm:
            urllib.request.urlopen(self.base + "/video/camera9", timeout=5)
        self.assertEqual(cm.exception.code, 404)

    def test_stream_address_points_to_this_server(self):
        self.assertTrue(WORLD.cams.uri(0).startswith(self.base.replace("127.0.0.1", "127.0.0.1")))

    @unittest.skipUnless(HAS_CV2, "opencv-python is not installed")
    def test_opencv_plays_the_camera_like_on_the_robot(self):
        import cv2
        import numpy as np
        img = np.zeros((240, 320, 3), np.uint8)
        img[:, :, 2] = 200
        jpeg = cv2.imencode(".jpg", img)[1].tobytes()
        WORLD.stop()
        t = time.time()
        while WORLD.running and time.time() - t < 3:
            time.sleep(0.01)
        WORLD.cams.port = self.srv.server_address[1]
        code = ("import cv2\n"
                "from dobot_quad import RobotClient, CameraId\n"
                "robot = RobotClient()\n"
                "uri = robot.video.open(CameraId.FRONT_RGB)\n"
                "cap = cv2.VideoCapture(uri, cv2.CAP_FFMPEG)\n"
                "shapes = []\n"
                "for _ in range(3):\n"
                "    ok, frame = cap.read()\n"
                "    shapes.append(frame.shape if ok else None)\n"
                "cap.release()\n"
                "robot.video.close(CameraId.FRONT_RGB)\n"
                "print(shapes)\n")
        with Feeder(WORLD, jpeg):
            run(WORLD, code, timeout=30)
        self.assertEqual(errors(WORLD), [])
        self.assertEqual(printed(WORLD)[-1], "[(240, 320, 3), (240, 320, 3), (240, 320, 3)]")


if __name__ == "__main__":
    unittest.main()
