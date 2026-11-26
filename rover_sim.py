"""
rover_sim.py - Simulation core for the Dobot Rover X1 quadruped.

Responsibilities:
  * keep the map (obstacle boxes / walkable steps / goal flags)
  * simulate robot motion (speed, turning, collisions, climbing low steps)
  * run the user's program in a separate thread that can be stopped at any time

User programs only get the real SDK API (`robot` = dobot_quad.RobotClient, see sdk_compat.py).
There are deliberately no simulator-only commands, so every program that runs here
uses commands that also exist on the real robot (poka-yoke).
Numbers (speed, body size) are approximations - tune them in CONFIG below.
"""
import difflib
import math
import sys
import threading
import time
import traceback
import types

# ----------------------------------------------------------------------------
# Robot constants (approx. body 0.76 x 0.37 m, ~15 kg)
# ----------------------------------------------------------------------------
CONFIG = {
    "gaits": {
        #          max speed m/s        max turn deg/s     step frequency Hz
        "walk": {"max_speed": 0.5, "max_turn": 60, "freq": 1.6},
        "trot": {"max_speed": 1.0, "max_turn": 90, "freq": 2.4},
        "run":  {"max_speed": 1.8, "max_turn": 120, "freq": 3.2},
    },
    "radius": 0.32,          # collision circle radius of the body (m)
    "foot_radius": 0.26,     # area covered by the feet - decides which surface supports the robot (m)
    "step_limit": 0.15,      # boxes at most this much higher than the robot can be stepped onto (m)
    "max_step_down": 0.25,   # higher drops are a ledge the robot will not walk off (m)
    "body_height": 0.28,     # standing body height (m)
    "dt": 0.02,              # simulation time step (s)
}

# Thai words users often type inside strings -> English values understood by the SDK
THAI_WORDS = {
    "ซ้าย": "left", "ขวา": "right", "หน้า": "forward", "ข้างหน้า": "forward", "เดินหน้า": "forward",
    "หลัง": "backward", "ข้างหลัง": "backward", "ถอยหลัง": "backward",
    "แดง": "red", "เขียว": "green", "น้ำเงิน": "blue", "ฟ้า": "cyan", "เหลือง": "yellow",
    "ส้ม": "orange", "ม่วง": "purple", "ขาว": "white", "ปิด": "off", "เปิด": "on",
    "เดิน": "walk", "ยืน": "balance_stand", "หมอบ": "stand_down",
}

PROGRAM_FILENAME = "<rover_program>"


def default_map():
    boxes = []
    # wall in the middle + a block + two low steps
    for i in range(4):
        boxes.append({"x": 5.25, "y": 2.25 + i * 0.5, "w": 0.5, "d": 0.5, "h": 0.6, "color": "#8d99ae"})
    boxes.append({"x": 2.75, "y": 6.75, "w": 0.5, "d": 0.5, "h": 0.3, "color": "#e07a5f"})
    boxes.append({"x": 7.25, "y": 7.25, "w": 0.5, "d": 0.5, "h": 0.1, "color": "#f2cc8f"})
    boxes.append({"x": 7.75, "y": 7.25, "w": 0.5, "d": 0.5, "h": 0.1, "color": "#f2cc8f"})
    return {
        "name": "Starter field",
        "size": [10, 10],
        "start": {"x": 1.25, "y": 1.25, "yaw": 0},
        "boxes": boxes,
        "goals": [{"x": 8.25, "y": 1.75}, {"x": 8.25, "y": 8.25}, {"x": 1.75, "y": 8.25}],
    }


class StopSim(Exception):
    """Raised when the user presses Stop."""


def _num(v, default=None):
    try:
        f = float(v)
        return f if math.isfinite(f) else default
    except (TypeError, ValueError):
        return default


def sanitize_map(m):
    """Return a clean copy of a map with all required fields and numeric values."""
    if not isinstance(m, dict):
        raise ValueError("map must be an object with size / start / boxes / goals")
    size = m.get("size") if isinstance(m.get("size"), (list, tuple)) and len(m.get("size")) == 2 else [10, 10]
    W = min(60.0, max(2.0, _num(size[0], 10.0)))
    D = min(60.0, max(2.0, _num(size[1], 10.0)))
    st = m.get("start") if isinstance(m.get("start"), dict) else {}
    start = {"x": min(W, max(0.0, _num(st.get("x"), 1.25))),
             "y": min(D, max(0.0, _num(st.get("y"), 1.25))),
             "yaw": _num(st.get("yaw"), 0.0)}
    boxes = []
    for b in m.get("boxes") or []:
        if not isinstance(b, dict) or _num(b.get("x")) is None or _num(b.get("y")) is None:
            continue
        boxes.append({"x": _num(b["x"]), "y": _num(b["y"]),
                      "w": max(0.05, _num(b.get("w"), 0.5)), "d": max(0.05, _num(b.get("d"), 0.5)),
                      "h": max(0.0, _num(b.get("h"), 0.6)),
                      "color": b.get("color") if isinstance(b.get("color"), str) else "#8d99ae"})
    goals = [{"x": _num(g["x"]), "y": _num(g["y"])} for g in (m.get("goals") or [])
             if isinstance(g, dict) and _num(g.get("x")) is not None and _num(g.get("y")) is not None]
    name = m.get("name") if isinstance(m.get("name"), str) else "Map"
    return {"name": name, "size": [W, D], "start": start, "boxes": boxes, "goals": goals}


def thai_to_english(value):
    """Translate a Thai word typed in a string (e.g. "ซ้าย") to the English value."""
    v = str(value).strip()
    return THAI_WORDS.get(v, v)


# ----------------------------------------------------------------------------
# World: all simulation state
# ----------------------------------------------------------------------------
class World:
    def __init__(self):
        self.lock = threading.RLock()
        self.map = default_map()
        self.map_version = 1
        self.logs = []          # [(seq, kind, text)]
        self.log_seq = 0
        self.speed_factor = 1.0
        self.stop_flag = threading.Event()
        self.running = False
        self.thread = None
        self.cur_line = None
        self.run_id = 0         # increases with every Run, so each browser window knows its own program
        self.boot = f"{time.time():.6f}"   # changes when the server restarts (open pages resync)
        self.trail_id = 0
        self.reset_robot()

    # ---------- logging ----------
    def log(self, text, kind="info"):
        with self.lock:
            self.log_seq += 1
            self.logs.append((self.log_seq, kind, str(text)))
            if len(self.logs) > 600:
                self.logs = self.logs[-400:]

    # ---------- map ----------
    def set_map(self, m):
        """Replace the map. Missing or invalid fields (e.g. from a hand-edited .json) are
        repaired; raises ValueError if `m` is not a map at all."""
        m = sanitize_map(m)
        with self.lock:
            self.map = m
            self.map_version += 1
            self.reached = set()

    # ---------- robot ----------
    def reset_robot(self):
        with self.lock:
            s = self.map["start"]
            self.r = {
                "x": float(s["x"]), "y": float(s["y"]), "z": 0.0,
                "yaw": math.radians(s.get("yaw", 0)),
                "roll": 0.0, "pitch": 0.0,
                "vx": 0.0, "vy": 0.0, "wz": 0.0,
                "gait": "trot", "posture": "lie",     # starts lying down, like the real robot at power-on
                "body_height": CONFIG["body_height"],
                "phase": 0.0, "led": "#00d0ff",
                "bumped": False, "action": None, "action_id": 0,
                "odometer": 0.0, "battery": 100.0,
                # ---- state of the real SDK (dobot_quad.RobotClient) ----
                "fsm": "stand_down", "speed_ratio": 50, "avoid": False,
                "bal": [0.0, 0.0, 0.0, 0.0],      # roll deg, pitch deg, yaw deg, height m
                "leg_led": None,                  # ['#rrggbb' x4] or None = default lights
            }
            self.trail = [(round(self.r["x"], 3), round(self.r["y"], 3))]
            self.trail_id += 1
            self.reached = set()
            self.sim_time = 0.0
            self.r["z"] = self.support_height(self.r["x"], self.r["y"], 0.0)

    # ---------- geometry helpers ----------
    def support_height(self, x, y, z_now):
        """Height of the surface the robot stands on at (x, y).

        Uses the whole area under the feet (not only the body center), so the robot
        stays on a step until its feet have left it. Only boxes reachable from z_now
        (at most step_limit higher) count; stepping down is always allowed."""
        foot = CONFIG["foot_radius"]
        z = 0.0
        for b in self.map["boxes"]:
            if b["h"] - z_now > CONFIG["step_limit"] + 1e-9:
                continue
            dx = max(abs(x - b["x"]) - b["w"] / 2, 0.0)
            dy = max(abs(y - b["y"]) - b["d"] / 2, 0.0)
            if dx * dx + dy * dy < foot * foot:
                z = max(z, b["h"])
        return z

    def clearance(self, x, y, z_ref):
        """Distance from the body edge to the nearest obstacle (negative = overlapping) and its kind.
        Boxes more than step_limit above z_ref are obstacles."""
        rad = CONFIG["radius"]
        W, D = self.map["size"]
        best, kind = min(x, y, W - x, D - y) - rad, "field edge"
        for b in self.map["boxes"]:
            if b["h"] - z_ref <= CONFIG["step_limit"] + 1e-9:
                continue
            ix = b["w"] / 2 - abs(x - b["x"])     # > 0 when the body center is inside the box
            iy = b["d"] / 2 - abs(y - b["y"])
            if ix > 0 and iy > 0:
                # center inside: the deeper it is, the more negative (so walking deeper is blocked,
                # walking out through the nearest side is allowed)
                d = -min(ix, iy) - rad
            else:
                cx = min(max(x, b["x"] - b["w"] / 2), b["x"] + b["w"] / 2)
                cy = min(max(y, b["y"] - b["d"] / 2), b["y"] + b["d"] / 2)
                d = math.hypot(x - cx, y - cy) - rad
            if d < best:
                best, kind = d, "obstacle"
        return best, kind

    # ---------- state for the browser ----------
    def snapshot(self, since=0, trail_from=0, trail_id=-1):
        with self.lock:
            r = dict(self.r)
            r["yaw_deg"] = round(math.degrees(r["yaw"]), 1) % 360
            if trail_id != self.trail_id:
                trail_from = 0
            return {
                "robot": r,
                "running": self.running,
                "sim_time": round(self.sim_time, 2),
                "map_version": self.map_version,
                "logs": [entry for entry in self.logs if entry[0] > since],
                "trail_id": self.trail_id,
                "trail_from": trail_from,
                "trail": self.trail[trail_from:],
                "reached": sorted(self.reached),
                "speed_factor": self.speed_factor,
                "line": self.cur_line,
                "run_id": self.run_id,
                "boot": self.boot,
            }

    # ---------- run user code ----------
    def run_code(self, code):
        with self.lock:
            if self.running:
                return False
            self.running = True
            self.stop_flag.clear()
            self.run_id += 1
            self.cur_line = None
        self.thread = threading.Thread(target=self._exec, args=(code,), daemon=True)
        self.thread.start()
        return self.run_id

    def stop(self):
        self.stop_flag.set()

    def _exec(self, code):
        import builtins
        import sdk_compat

        motion = Motion(self)
        sdk_mod = sdk_compat.make_module(self)
        real_import = builtins.__import__

        # time.sleep() in user code follows the sim speed and stops with the Stop button
        sim_time_mod = types.ModuleType("time")
        sim_time_mod.__dict__.update({k: v for k, v in vars(time).items() if not k.startswith("__")})
        sim_time_mod.sleep = motion.wait

        def sim_import(name, globals=None, locals=None, fromlist=(), level=0):
            if name == "dobot_quad" or name.startswith("dobot_quad."):
                return sdk_mod                 # use the simulated SDK instead of the real one
            if name == "time":
                return sim_time_mod
            return real_import(name, globals, locals, fromlist, level)

        def no_input(*args, **kwargs):
            raise RuntimeError("input() is not available in the simulator (there is no keyboard "
                               "console) - put the value in the code instead, e.g. n = 3")

        bi = dict(vars(builtins))
        bi["__import__"] = sim_import
        bi["print"] = motion.print
        bi["input"] = no_input
        # Nothing is pre-loaded: like on the real robot the program must import dobot_quad
        # and create robot = RobotClient(...) itself (Auto-correct adds missing lines).
        g = {"__name__": "__main__", "__builtins__": bi}

        stop_flag = self.stop_flag
        world = self

        def local_trace(frame, event, arg):
            if stop_flag.is_set():
                raise StopSim()
            if event == "line":
                world.cur_line = frame.f_lineno
            return local_trace

        def global_trace(frame, event, arg):
            if frame.f_code.co_filename == PROGRAM_FILENAME:
                return local_trace
            return None

        self.log("▶ Program started", "sys")
        try:
            compiled = compile(code, PROGRAM_FILENAME, "exec")
            sys.settrace(global_trace)
            exec(compiled, g)
            self.log(f"✔ Program finished (simulated time {self.sim_time:.1f} s)", "sys")
        except StopSim:
            self.log("■ Program stopped", "sys")
        except SystemExit as e:
            code_txt = "" if e.code in (None, 0) else f" (code {e.code})"
            self.log(f"■ Program ended by exit(){code_txt} (simulated time {self.sim_time:.1f} s)", "sys")
        except SyntaxError as e:
            self.log(f"✖ SyntaxError on line {e.lineno}: {e.msg}", "error")
            if e.text:
                self.log("   " + e.text.rstrip(), "error")
        except Exception as e:  # noqa: BLE001
            self.log(friendly_error(e, g), "error")
        finally:
            sys.settrace(None)
            with self.lock:
                self.r["vx"] = self.r["vy"] = self.r["wz"] = 0.0
                self.running = False


def friendly_error(e, g):
    tb = traceback.extract_tb(e.__traceback__)
    line = None
    for fr in tb:
        if fr.filename == PROGRAM_FILENAME:
            line = fr.lineno
    where = f"line {line}: " if line else ""
    msg = f"✖ {where}{type(e).__name__}: {e}"
    if isinstance(e, NameError):
        import sdk_compat
        name = getattr(e, "name", None)
        if name in ("rover", "dog"):
            msg += ("\n   💡 The simulator-only rover.* commands were removed. "
                    "Use the real SDK: robot.walk_forward(), robot.rotate_left(), ...")
        elif name == "robot":
            msg += ("\n   💡 Create the robot first, like on the real robot:\n"
                    "      from dobot_quad import RobotClient\n"
                    "      robot = RobotClient(\"192.168.5.2:50051\")")
        elif name in sdk_compat.SDK_EXPORTS:
            msg += f"\n   💡 Add at the top: from dobot_quad import {name}"
        elif name in ("math", "time", "random"):
            msg += f"\n   💡 Add at the top: import {name}"
        elif name:
            cand = difflib.get_close_matches(name, list(g.keys()) + dir(__builtins__), 3, 0.6)
            if cand:
                msg += f"\n   💡 Did you mean {', '.join(cand)}?"
    return msg


# ----------------------------------------------------------------------------
# Motion: internal movement engine used by the simulated RobotClient.
# Not available to user programs.
# ----------------------------------------------------------------------------
class Motion:
    def __init__(self, world):
        self._w = world

    def print(self, *args, sep=" ", end="", **_):
        self._w.log(sep.join(str(a) for a in args) + (end if end != "\n" else ""), "print")

    def check_stop(self):
        if self._w.stop_flag.is_set():
            raise StopSim()

    def sleep(self, dt):
        """Sleep for `dt` simulated seconds (scaled by the sim speed)."""
        self.check_stop()
        time.sleep(dt / max(self._w.speed_factor, 0.05))

    def step(self, vx, vy, wz, dt):
        """Advance one time step. Returns False if the move was blocked."""
        w = self._w
        with w.lock:
            r = w.r
            yaw = r["yaw"]
            c, s = math.cos(yaw), math.sin(yaw)
            nx = r["x"] + (vx * c - vy * s) * dt
            ny = r["y"] + (vx * s + vy * c) * dt
            z_here = w.support_height(r["x"], r["y"], r["z"])
            if z_here < r["z"] - 1e-9:
                r["z"] = z_here        # the floor under the robot was removed in the editor: it falls
            z_new = w.support_height(nx, ny, r["z"])
            c_new, kind = w.clearance(nx, ny, z_new)
            c_old, _ = w.clearance(r["x"], r["y"], z_new)
            hit = None
            # Only moves that go *deeper* into an obstacle are blocked, so a robot that
            # overlaps something (e.g. right after stepping down) can always move away.
            if r["z"] - z_new > CONFIG["max_step_down"] + 1e-9:
                hit, kind = "ledge", f"{r['z'] - z_new:.2f} m drop"
            elif c_new < 0 and c_new < c_old - 1e-9:
                hit = kind
            elif r["avoid"] and (vx or vy) and c_new < 0.12 and c_new < c_old - 1e-9:
                hit = "avoid"
            moved = hit is None
            if hit:
                if not r["bumped"]:
                    if hit == "ledge":
                        w.log(f"🛑 Ledge too high to step down ({kind}) at ({r['x']:.2f}, {r['y']:.2f}) - "
                              "stopped", "warn")
                    elif hit == "avoid":
                        w.log(f"🛡 obstacle avoidance: stopped before the {kind} at "
                              f"({r['x']:.2f}, {r['y']:.2f})", "warn")
                    else:
                        w.log(f"💥 Hit the {hit} at ({r['x']:.2f}, {r['y']:.2f}) - stopped", "warn")
                r["bumped"] = True
                r["vx"] = r["vy"] = 0.0
            else:
                r["bumped"] = False
                r["odometer"] += math.hypot(nx - r["x"], ny - r["y"])
                r["x"], r["y"], r["z"] = nx, ny, z_new
                r["vx"], r["vy"] = vx, vy
                last = w.trail[-1]
                if math.hypot(nx - last[0], ny - last[1]) > 0.08:
                    w.trail.append((round(nx, 3), round(ny, 3)))
                    if len(w.trail) > 3000:
                        w.trail = w.trail[-2000:]
                        w.trail_id += 1
            r["yaw"] = (yaw + wz * dt + math.pi) % (2 * math.pi) - math.pi
            r["wz"] = wz
            speed = math.hypot(vx, vy) + abs(wz) * 0.25
            if speed > 0.01:
                r["phase"] = (r["phase"] + CONFIG["gaits"][r["gait"]]["freq"] * dt) % 1.0
            # a little IMU motion for realism
            r["roll"] = -wz * math.hypot(vx, vy) * 0.08
            r["pitch"] = -0.02 * vx
            r["battery"] = max(0.0, r["battery"] - (0.002 + 0.01 * speed) * dt)
            w.sim_time += dt
            # goal flags on the map
            for i, gl in enumerate(w.map["goals"]):
                if i not in w.reached and math.hypot(gl["x"] - r["x"], gl["y"] - r["y"]) < 0.45:
                    w.reached.add(i)
                    w.log(f"🚩 Reached flag {i + 1}! ({len(w.reached)}/{len(w.map['goals'])})", "goal")
                    if len(w.reached) == len(w.map["goals"]):
                        w.log("🏆 All flags collected - well done!", "goal")
        self.sleep(dt)
        return moved

    def idle(self):
        with self._w.lock:
            self._w.r["vx"] = self._w.r["vy"] = self._w.r["wz"] = 0.0

    def travel(self, dist, vx, vy):
        """Move `dist` meters with body velocity (vx, vy). Returns False if blocked."""
        dt = CONFIG["dt"]
        done = 0.0
        v = math.hypot(vx, vy)
        ok = True
        while done < dist - 1e-6:
            step = min(dt, (dist - done) / v)
            ramp = min(1.0, 0.35 + done / 0.25, 0.35 + (dist - done) / 0.25)   # ease in / out
            if not self.step(vx * ramp, vy * ramp, 0.0, step / ramp if ramp < 1 else step):
                ok = False
                break
            done += v * step
        self.idle()
        return ok

    def turn(self, angle, speed):
        """Turn in place `angle` degrees (positive = left) at `speed` deg/s."""
        g = CONFIG["gaits"][self._w.r["gait"]]
        speed = min(abs(float(speed)), g["max_turn"])
        total = math.radians(abs(float(angle)))
        wz = math.radians(speed) * (1 if angle >= 0 else -1)
        dt = CONFIG["dt"]
        done = 0.0
        while done < total - 1e-6:
            step = min(dt, (total - done) / abs(wz))
            self.step(0.0, 0.0, wz, step)
            done += abs(wz) * step
        self.idle()

    def move(self, vx, vy, wz_deg, duration):
        """Hold body velocity vx, vy (m/s) and turn rate wz (deg/s) for `duration` s.
        Returns False if blocked."""
        g = CONFIG["gaits"][self._w.r["gait"]]
        vx = max(-g["max_speed"], min(g["max_speed"], float(vx)))
        vy = max(-g["max_speed"] * 0.5, min(g["max_speed"] * 0.5, float(vy)))
        wz = math.radians(max(-g["max_turn"], min(g["max_turn"], float(wz_deg))))
        dt = CONFIG["dt"]
        t = 0.0
        ok = True
        while t < float(duration) - 1e-6:
            step = min(dt, float(duration) - t)
            if not self.step(vx, vy, wz, step):
                ok = False
                if abs(wz) < 1e-6:      # still allowed to keep turning when blocked
                    break
            t += step
        self.idle()
        return ok

    def wait(self, seconds):
        """Wait `seconds` (simulated time). Stays stoppable; sleeps in chunks of >= 5 ms real time."""
        seconds = float(seconds)
        t = 0.0
        while t < seconds - 1e-9:
            chunk = min(seconds - t, max(CONFIG["dt"], 0.005 * max(self._w.speed_factor, 0.05)))
            with self._w.lock:
                self._w.sim_time += chunk
            self.sleep(chunk)
            t += chunk


def api_docs():
    import sdk_compat
    return {"sdk": sdk_compat.sdk_docs(), "consts": sdk_compat.SDK_CONSTS}


WORLD = World()
