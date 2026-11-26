"""HTTP server: API responses, bad input, file serving, several browser windows."""
import json
import os
import shutil
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request

from helpers import FAST

import server
from rover_sim import WORLD, default_map


class ServerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.srv = server.Server(("127.0.0.1", 0), server.Handler)
        cls.base = f"http://127.0.0.1:{cls.srv.server_address[1]}"
        threading.Thread(target=cls.srv.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()

    def setUp(self):
        WORLD.stop()
        t = time.time()
        while WORLD.running and time.time() - t < 3:
            time.sleep(0.01)
        WORLD.set_map(default_map())       # every test starts from the default map
        WORLD.reset_robot()
        WORLD.speed_factor = FAST

    # ---------- helpers ----------
    def get(self, path):
        with urllib.request.urlopen(self.base + path, timeout=5) as r:
            return r.status, r.read()

    def post(self, path, body):
        data = body if isinstance(body, bytes) else json.dumps(body).encode()
        req = urllib.request.Request(self.base + path, data=data, method="POST",
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status, json.loads(r.read() or b"null")
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read() or b"null")

    def wait_idle(self):
        t = time.time()
        while time.time() - t < 10:
            if not json.loads(self.get("/api/state")[1])["running"]:
                return
            time.sleep(0.02)
        self.fail("program did not finish")

    # ---------- tests ----------
    def test_page_and_docs(self):
        status, body = self.get("/")
        self.assertEqual(status, 200)
        self.assertIn(b"Rover X1", body)
        docs = json.loads(self.get("/api/docs")[1])
        self.assertIn("walk_forward", [d["name"] for d in docs["sdk"]])
        self.assertNotIn("rover", docs)

    def test_check_endpoint(self):
        status, r = self.post("/api/check", {"code": "robot.walk_forwrd(1)"})
        self.assertEqual(status, 200)
        self.assertEqual(r["code"], 'from dobot_quad import RobotClient\nrobot = RobotClient("192.168.5.2:50051")\n'
                                    "robot.walk_forward(1)")

    def test_run_and_state(self):
        status, r = self.post("/api/run", {"code": "print('hi')", "autocorrect": True, "reset": True})
        self.assertTrue(r["ok"])
        self.wait_idle()
        logs = [entry[2] for entry in json.loads(self.get("/api/state")[1])["logs"]]
        self.assertIn("hi", logs)

    def test_run_id_tells_windows_apart(self):
        # two browser windows share one robot: each must know whether the running
        # program is its own (otherwise it highlights the wrong line)
        status, r = self.post("/api/run", {"code": "import time\ntime.sleep(1)"})
        self.assertIn("run_id", r)
        state = json.loads(self.get("/api/state")[1])
        self.assertEqual(state.get("run_id"), r["run_id"])
        WORLD.stop()

    def test_bad_json_returns_400(self):
        status, r = self.post("/api/run", b"{not json")
        self.assertEqual(status, 400)

    def test_missing_fields_return_400_not_a_crash(self):
        for path, body in (("/api/map", {}), ("/api/speed", {"speed": "fast"}), ("/api/map", {"map": "x"})):
            status, r = self.post(path, body)
            self.assertEqual(status, 400, f"{path} {body} -> {status} {r}")
            self.assertFalse(r["ok"])

    def test_incomplete_map_is_repaired(self):
        bad = {"size": ["6", 6], "start": {"x": 1, "y": 1},
               "boxes": [{"x": 3, "y": 1}, {"x": "a"}, "junk"], "goals": [{"x": 5}]}
        status, r = self.post("/api/map", {"map": bad})
        self.assertEqual(status, 200)
        self.post("/api/run", {"code": "robot.walk_forward(1)"})
        self.wait_idle()
        state = json.loads(self.get("/api/state")[1])
        errs = [e[2] for e in state["logs"] if e[1] == "error"]
        self.assertEqual(errs, [])
        m = json.loads(self.get("/api/map")[1])["map"]
        self.assertEqual(m["size"], [6.0, 6.0])
        self.assertTrue(all({"x", "y", "w", "d", "h"} <= set(b) for b in m["boxes"]))

    def raw_post(self, path, body, headers):
        req = urllib.request.Request(self.base + path, data=json.dumps(body).encode(), method="POST")
        for k, v in headers.items():
            req.add_header(k, v)
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status
        except urllib.error.HTTPError as e:
            return e.code

    def test_other_websites_cannot_run_code(self):
        # CSRF: a page on another site posting to localhost must not start a program
        before = WORLD.run_id
        body = {"code": "print('pwned')"}
        for headers in ({"Content-Type": "text/plain"},
                        {"Content-Type": "application/json", "Origin": "http://evil.example"}):
            self.assertEqual(self.raw_post("/api/run", body, headers), 403, headers)
        self.assertEqual(WORLD.run_id, before)

    def test_dns_rebinding_host_is_rejected(self):
        port = self.srv.server_address[1]
        req = urllib.request.Request(self.base + "/api/state")
        req.add_header("Host", f"evil.example:{port}")
        with self.assertRaises(urllib.error.HTTPError) as ctx:
            urllib.request.urlopen(req, timeout=5)
        self.assertEqual(ctx.exception.code, 403)

    def test_negative_content_length_does_not_hang(self):
        import socket
        port = self.srv.server_address[1]
        with socket.create_connection(("127.0.0.1", port), timeout=3) as s:
            s.sendall(f"POST /api/check HTTP/1.1\r\nHost: 127.0.0.1:{port}\r\n"
                      "Content-Type: application/json\r\nContent-Length: -1\r\n\r\n".encode())
            try:
                reply = s.recv(200)
            except socket.timeout:
                self.fail("server hangs on Content-Length: -1")
        self.assertIn(b"400", reply.split(b"\r\n")[0])

    def test_map_post_returns_the_repaired_map(self):
        status, r = self.post("/api/map", {"map": {"size": [6, 6], "boxes": [{"x": 3, "y": 1}], "goals": []}})
        self.assertEqual(status, 200)
        self.assertEqual(r["map"]["boxes"][0]["h"], 0.6)

    def test_state_has_a_server_boot_id(self):
        # an open page must notice when the server was restarted (logs start again from 1)
        a = json.loads(self.get("/api/state")[1])
        b = json.loads(self.get("/api/state")[1])
        self.assertIn("boot", a)
        self.assertEqual(a["boot"], b["boot"])

    def test_no_files_outside_static(self):
        for path in ("/../server.py", "/..%2fserver.py", "/%2e%2e/server.py", "/static/../../server.py"):
            try:
                status, body = self.get(path)
            except urllib.error.HTTPError as e:
                status, body = e.code, b""
            self.assertEqual(status, 404, path)
            self.assertNotIn(b"import", body)

    def test_no_files_from_a_sibling_folder_with_a_similar_name(self):
        tmp = tempfile.mkdtemp()
        try:
            os.makedirs(os.path.join(tmp, "static"))
            os.makedirs(os.path.join(tmp, "static_secret"))
            with open(os.path.join(tmp, "static_secret", "key.txt"), "w") as f:
                f.write("SECRET")
            old = server.STATIC
            server.STATIC = os.path.join(tmp, "static")
            try:
                try:
                    status, body = self.get("/../static_secret/key.txt")
                except urllib.error.HTTPError as e:
                    status, body = e.code, b""
            finally:
                server.STATIC = old
            self.assertNotIn(b"SECRET", body)
            self.assertEqual(status, 404)
        finally:
            shutil.rmtree(tmp)


if __name__ == "__main__":
    unittest.main()
