"""
server.py - starts the Dobot Rover X1 Simulator.

Usage:   python server.py      (or double-click RoverX1Sim.exe / run.bat)
The browser opens http://localhost:8765 automatically.
Works offline: three.js and CodeMirror are bundled in static/vendor/.
"""
import json
import mimetypes
import os
import sys
import threading
import urllib.request
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

# When packaged as an .exe (PyInstaller) all files are unpacked to sys._MEIPASS
ROOT = getattr(sys, "_MEIPASS", os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
for _s in (sys.stdout, sys.stderr):   # allow non-ASCII output in the Windows console
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass

from autocorrect import correct  # noqa: E402
from rover_sim import WORLD, api_docs  # noqa: E402

STATIC = os.path.join(ROOT, "static")
PORT = int(os.environ.get("ROVER_PORT", "8765"))


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):  # keep the console quiet
        pass

    def _json(self, obj, code=200):
        data = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _body(self):
        n = int(self.headers.get("Content-Length", 0) or 0)
        if n < 0 or n > 5_000_000:
            raise ValueError(f"bad Content-Length {n}")
        if not n:
            return {}
        return json.loads(self.rfile.read(n).decode("utf-8"))

    def _allowed(self, post):
        """Only this computer's own page may use the API.

        The server runs Python code, so a web page on another site must not be able to
        send it programs (CSRF), and a hostile DNS name pointing to 127.0.0.1 must not
        reach it either (DNS rebinding)."""
        port = self.server.server_address[1]
        hosts = {f"localhost:{port}", f"127.0.0.1:{port}", f"[::1]:{port}"}
        if (self.headers.get("Host") or "").strip().lower() not in hosts:
            return False
        origin = self.headers.get("Origin")
        if origin is not None and origin.strip().rstrip("/").lower() not in {f"http://{h}" for h in hosts}:
            return False
        content_type = (self.headers.get("Content-Type") or "").lower()
        return not post or content_type.startswith("application/json")

    def do_GET(self):
        if not self._allowed(post=False):
            self._json({"ok": False, "error": "forbidden"}, 403)
            return
        self._safe(self._get)

    def do_POST(self):
        if not self._allowed(post=True):
            self._drain()      # read the body first, or the client may get a reset instead of 403
            self._json({"ok": False, "error": "forbidden"}, 403)
            return
        self._safe(self._post)

    def _drain(self):
        try:
            n = int(self.headers.get("Content-Length", 0) or 0)
        except ValueError:
            return
        if 0 < n <= 5_000_000:
            self.rfile.read(n)

    def _safe(self, fn):
        """Bad input must give a JSON error, never a dropped connection."""
        try:
            fn()
        except (KeyError, ValueError, TypeError, AttributeError) as e:
            self._json({"ok": False, "error": f"bad request: {type(e).__name__}: {e}"}, 400)
        except Exception as e:  # noqa: BLE001
            self._json({"ok": False, "error": f"server error: {type(e).__name__}: {e}"}, 500)

    def _get(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)
        if u.path == "/api/state":
            self._json(WORLD.snapshot(int(q.get("since", [0])[0]),
                                      int(q.get("trail", [0])[0]),
                                      int(q.get("tid", [-1])[0])))
            return
        if u.path == "/api/map":
            with WORLD.lock:
                self._json({"map": WORLD.map, "version": WORLD.map_version})
            return
        if u.path == "/api/docs":
            self._json(api_docs())
            return
        self._static(u.path)

    def _post(self):
        u = urlparse(self.path)
        try:
            body = self._body()
        except (ValueError, UnicodeDecodeError):
            self._json({"ok": False, "error": "bad json"}, 400)
            return
        if not isinstance(body, dict):
            raise ValueError("the request body must be a JSON object")
        if u.path == "/api/check":
            code, changes, err = correct(body.get("code", ""))
            self._json({"code": code, "changes": changes, "error": err})
            return
        if u.path == "/api/run":
            code = body.get("code", "")
            changes, err = [], None
            if body.get("autocorrect", True):
                code, changes, err = correct(code)
                for c in changes:
                    if c.get("warning"):
                        WORLD.log(f"⚠ line {c['line']}: {c['before']} - {c['reason']}", "warn")
                    else:
                        WORLD.log(f"🔧 line {c['line']}: {c['reason']}", "fix")
            if WORLD.running:
                self._json({"ok": False, "error": "A program is already running - press Stop first"})
                return
            if body.get("reset", True):
                WORLD.reset_robot()
            run_id = WORLD.run_code(code)
            self._json({"ok": bool(run_id), "run_id": run_id or None, "code": code,
                        "changes": changes, "error": err})
            return
        if u.path == "/api/stop":
            WORLD.stop()
            self._json({"ok": True})
            return
        if u.path == "/api/reset":
            WORLD.stop()
            if WORLD.thread:
                WORLD.thread.join(timeout=2)
            WORLD.reset_robot()
            WORLD.log("↺ Robot reset to the start position", "sys")
            self._json({"ok": True})
            return
        if u.path == "/api/map":
            WORLD.set_map(body["map"])
            if not WORLD.running and body.get("reset", True):
                WORLD.reset_robot()
            with WORLD.lock:   # send back the repaired map so the page draws exactly what is simulated
                self._json({"ok": True, "version": WORLD.map_version, "map": WORLD.map})
            return
        if u.path == "/api/speed":
            WORLD.speed_factor = max(0.1, min(10.0, float(body.get("speed", 1))))
            self._json({"ok": True})
            return
        if u.path == "/api/clearlog":
            with WORLD.lock:
                WORLD.logs.clear()
            self._json({"ok": True})
            return
        self._json({"ok": False, "error": "not found"}, 404)

    def _static(self, path):
        if path in ("/", ""):
            path = "/index.html"
        root = os.path.normpath(STATIC)
        full = os.path.normpath(os.path.join(root, path.lstrip("/")))
        # commonpath (not startswith): "static_secret" must not count as inside "static"
        inside = os.path.normcase(os.path.commonpath([root, full])) == os.path.normcase(root)
        if not inside or not os.path.isfile(full):
            self.send_error(404)
            return
        ctype = mimetypes.guess_type(full)[0] or "application/octet-stream"
        if ctype.startswith("text/") or ctype == "application/javascript":
            ctype += "; charset=utf-8"
        with open(full, "rb") as f:
            data = f.read()
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


class Server(ThreadingHTTPServer):
    # On Windows, SO_REUSEADDR would let a second copy bind the same port - keep it off
    allow_reuse_address = os.name != "nt"
    daemon_threads = True


def already_running(url):
    """Is the simulator already open (e.g. the .exe was double-clicked twice)?"""
    try:
        with urllib.request.urlopen(url + "/api/map", timeout=1.5) as r:
            return r.status == 200
    except Exception:  # noqa: BLE001
        return False


def pause_if_exe():
    if getattr(sys, "frozen", False):
        try:
            input("\nPress Enter to close this window...")
        except EOFError:
            pass


def main():
    mimetypes.add_type("application/javascript", ".js")
    url = f"http://localhost:{PORT}"
    if already_running(url):
        print("The simulator is already running → opening it in the browser")
        webbrowser.open(url)
        return
    try:
        srv = Server(("127.0.0.1", PORT), Handler)
    except OSError as e:
        print(f"✖ Cannot open port {PORT} ({e}) - another program may be using it.")
        print("  Close that program, or set the ROVER_PORT environment variable to another number.")
        pause_if_exe()
        return
    print("=" * 56)
    print("  Dobot Rover X1 Simulator")
    print(f"  Open in your browser:  {url}")
    print("  Close this window (or press Ctrl+C) to quit")
    print("=" * 56)
    if "--no-browser" not in sys.argv:
        threading.Timer(0.8, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nSimulator closed")


if __name__ == "__main__":
    main()
