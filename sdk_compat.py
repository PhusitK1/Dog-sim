"""
sdk_compat.py - Simulated version of the real Dobot Rover X1 high-level SDK.
Reference: https://github.com/Dobot-Team/dobot_quad_sdk  (high_level/python/dobot_quad)

Code written like this in the simulator:

    from dobot_quad import RobotClient
    robot = RobotClient("192.168.5.2:50051")
    robot.balance_stand()
    robot.walk_forward(2.0)

can be run on the real robot (after installing the real SDK and connecting to the robot).

Poka-yoke rule: this simulator is AT LEAST AS STRICT as the real SDK. Everything the real
robot_client.py rejects (ValueError / AttributeError) also fails here - with a 💡 hint -
so a program never passes here and then crashes on the robot. Typos are fixed in the
source code by autocorrect.py, never silently at run time.

Function names / parameters / value ranges follow the real SDK; the *behavior* (real
speeds, dances, balancing) is an approximation for learning.
"""
import difflib
import inspect
import math
import threading
import time
import types
from dataclasses import dataclass, field
from enum import Enum, IntEnum
from typing import Optional

from rover_sim import CONFIG, Motion, StopSim, thai_to_english

# ----------------------------------------------------------------------------
# Constants (same values as the real SDK)
# ----------------------------------------------------------------------------
LEGGED_STATES = ["passive", "ready", "stand_down", "balance_stand", "walk", "flying_trot",
                 "rl", "gongxi", "dance0", "wave", "jump", "backflip", "recovery"]
WHEEL_ONLY_STATES = ["wheel_loco", "drift", "climb", "handstand"]
STATE_ALIASES = {"emergency": "passive", "choreo": "gongxi"}          # exactly the real aliases
BALANCE_MOTIONS = ["balance_pitch", "balance_yaw", "balance_roll", "balance_height", "balance_neutral"]
ATOMIC_ACTIONS = {"twirl_jump": 2.0, "diag_step": 7.0, "hop_step": 4.0, "groove": 9.0,
                  "bounce": 7.0, "body_wave": 9.0, "hip_circle": 9.0, "head_circle": 9.0}
GAITS = ["walk", "flying_trot"]
DIRECTION_MAP = {"forward": 0, "front": 0, "backward": 1, "back": 1, "left": 2, "right": 3}
POSE_MODES = ["dynamic", "static"]
STATE_POST_SLEEP = 2.0
DANCE_DEFAULT, DANCE_MIN, DANCE_MAX = 126.5, 4.0, 128.0
WAVE_DEFAULT, WAVE_TAIL = 5.0, 2.0
LIMITS = {"roll": 17.0, "pitch": 11.5, "yaw": 11.5}
HEIGHT_MIN, HEIGHT_MAX = -0.08, 0.0

# exported like the real package
VALID_STATES = frozenset(LEGGED_STATES)
VALID_BALANCE_MOTIONS = frozenset(BALANCE_MOTIONS)
VALID_ATOMIC_ACTIONS = frozenset(ATOMIC_ACTIONS)
VALID_GAITS = frozenset(GAITS)
VALID_WHEEL_STATES = frozenset(["passive", "ready", "stand_down"] + WHEEL_ONLY_STATES)
VALID_WHEEL_GAITS = frozenset({"wheel_loco"})

# Approximate speeds at speed_ratio = 100 (NOT in the SDK docs - assumed values)
MODE_SPEED = {
    "walk":        {"gait": "trot", "vmax": 1.0, "wmax": 90},
    "rl":          {"gait": "trot", "vmax": 1.2, "wmax": 100},
    "flying_trot": {"gait": "run",  "vmax": 1.8, "wmax": 120},
}
LOCO_STATES = set(MODE_SPEED)
LYING_STATES = {"passive", "ready", "stand_down"}


class Leg(IntEnum):
    FL = 1           # front left leg light
    FR = 2           # front right leg light
    RL = 3           # rear left leg light
    RR = 4           # rear right leg light
    FILL_FRONT = 5   # front fill light (cannot be set with the leg LED functions)
    FILL_BACK = 6    # rear fill light


MAIN_LEGS = (Leg.FL, Leg.FR, Leg.RL, Leg.RR)


class Color(IntEnum):
    OFF = 0
    RED = 1
    ORANGE = 2
    YELLOW = 3
    GREEN = 4
    CYAN = 5
    BLUE = 6
    PURPLE = 7
    WHITE = 8


COLOR_RGB = {Color.OFF: (0, 0, 0), Color.RED: (255, 0, 0), Color.ORANGE: (255, 165, 0),
             Color.YELLOW: (255, 255, 0), Color.GREEN: (0, 255, 0), Color.CYAN: (0, 255, 255),
             Color.BLUE: (0, 0, 255), Color.PURPLE: (128, 0, 128), Color.WHITE: (255, 255, 255)}


# ---- camera streaming types: same names and values as dobot_quad/video/types.py ----
class CameraId(IntEnum):
    """Camera identifier."""
    FRONT_RGB = 0  # Front RGB camera
    REAR_RGB = 1   # Rear RGB camera

    @property
    def spec(self):
        return CAMERA_SPECS[self]

    @property
    def rtsp_name(self):
        """Stream name used inside the stream address."""
        return CAMERA_SPECS[self].rtsp_name

    @property
    def dds_index(self):
        return CAMERA_SPECS[self].dds_index

    @property
    def description(self):
        return CAMERA_SPECS[self].description

    def __str__(self):
        return self.name


@dataclass(frozen=True)
class CameraSpec:
    rtsp_name: str
    dds_index: int
    description: str
    codec: str = "H.264"
    resolution: str = "1280x720"


CAMERA_SPECS = {
    CameraId.FRONT_RGB: CameraSpec("camera1", 0, "Front RGB"),
    CameraId.REAR_RGB: CameraSpec("camera2", 1, "Rear RGB"),
}
STREAM_CODEC = "H.264"


class StreamState(Enum):
    STOPPED = "stopped"        # Never opened, or closed again
    PREPARING = "preparing"    # open() is waiting for the camera to really stream
    READY = "ready"            # open() returned an address you can play
    RECOVERING = "recovering"  # The stream dropped or stalled
    STOPPING = "stopping"      # Last user released it
    ERROR = "error"            # Recovery gave up; call open() to try again

    def __str__(self):
        return self.value


class StreamOwner(Enum):
    UNKNOWN = "unknown"
    SDK = "sdk"
    EXTERNAL = "external"

    def __str__(self):
        return self.value


class LossReason(Enum):
    NONE = "none"
    EXTERNAL = "external"
    STALL = "stall"
    PORT_UNREACHABLE = "port_unreachable"
    NO_DATA = "no_data"
    CLOSED = "closed"

    def __str__(self):
        return self.value


class RecoveredBy(Enum):
    NONE = "none"
    L1_RESUME = "L1_resume"
    L2_RESTART = "L2_restart"

    def __str__(self):
        return self.value


class StreamEventKind(Enum):
    READY = "StreamReady"
    LOST = "StreamLost"
    RECONNECTED = "StreamReconnected"
    CLOSED = "StreamClosed"
    ERROR = "StreamError"


class LegLedConfig:
    """Per-leg RGB and brightness entry (same fields as the real dataclass)."""

    def __init__(self, leg, r=0, g=0, b=0, brightness=-1):
        self.leg, self.r, self.g, self.b, self.brightness = leg, r, g, b, brightness

    def __repr__(self):
        return f"LegLedConfig(leg={self.leg!r}, r={self.r}, g={self.g}, b={self.b}, brightness={self.brightness})"


@dataclass(frozen=True)
class CameraStreamInfo:
    """Snapshot returned by robot.video.get_stream_info()."""
    camera: CameraId
    state: StreamState = StreamState.STOPPED
    owner: StreamOwner = StreamOwner.UNKNOWN
    uri: str = ""
    codec: str = STREAM_CODEC
    bitrate_kbps: Optional[float] = None
    reason: LossReason = LossReason.NONE
    message: str = ""

    @property
    def available(self):
        return self.state == StreamState.READY and bool(self.uri)

    @property
    def is_streaming(self):
        return self.state == StreamState.READY

    def to_dict(self):
        return {"camera": self.camera.name, "state": self.state.value, "owner": self.owner.value,
                "uri": self.uri, "codec": self.codec, "bitrate_kbps": self.bitrate_kbps,
                "reason": self.reason.value, "message": self.message}


@dataclass(frozen=True)
class StreamEvent:
    """Event passed to robot.video.on_state_change() callbacks."""
    kind: StreamEventKind
    camera: CameraId
    old_state: StreamState
    new_state: StreamState
    reason: LossReason = LossReason.NONE
    recovered_by: RecoveredBy = RecoveredBy.NONE
    uri: str = ""
    message: str = ""
    timestamp: float = field(default_factory=time.monotonic)

    @property
    def name(self):
        return self.kind.value

    def to_dict(self):
        return {"event": self.name, "camera": self.camera.name, "old_state": self.old_state.value,
                "new_state": self.new_state.value, "reason": self.reason.value,
                "recovered_by": self.recovered_by.value, "uri": self.uri, "message": self.message,
                "timestamp": self.timestamp}


class VideoError(RuntimeError):
    """Base class of every exception raised by robot.video."""

    def __init__(self, message, *, reason=LossReason.NONE, camera=None):
        super().__init__(message)
        self.reason = reason
        self.camera = camera


class VideoConnectionError(VideoError):
    """The robot or its streaming service could not be reached."""


class VideoTimeoutError(VideoError):
    """The operation did not finish in time."""


def _result(success=True, **kw):
    return types.SimpleNamespace(success=success, **kw)


def _hint(value, choices):
    """' 💡 ...' suggestion for a wrong value (Thai word or typo)."""
    en = thai_to_english(value)
    if en in choices:
        return f" 💡 use '{en}' (Thai words do not work on the real robot)"
    m = difflib.get_close_matches(str(value).strip().lower(), list(choices), 1, 0.6)
    return f" 💡 did you mean '{m[0]}'?" if m else ""


# ----------------------------------------------------------------------------
# RobotClient (simulated)
# ----------------------------------------------------------------------------
class SimRobotClient:
    """Mimics dobot_quad.RobotClient but drives the simulated robot."""
    _world = None          # set by make_module() for each user program

    def __init__(self, addr="192.168.5.2:50051", _quiet=False):
        object.__setattr__(self, "_rv", Motion(self._world))
        object.__setattr__(self, "_addr", addr)
        object.__setattr__(self, "_led_cache", {})
        object.__setattr__(self, "video", _SimVideo(self))
        # The SDK docs say the constructor enables obstacle avoidance (the current SDK code only
        # assumes it is on). The simulator turns it on so both agree.
        self.set_obstacle_avoidance(True, _quiet=True)
        if not _quiet:
            self._log(f"🔌 [sim] Connected RobotClient('{addr}')", "sys")
            self._log(f"   speed_ratio={self._r['speed_ratio']}, obstacle_avoidance=True", "info")

    # ---------- helpers ----------
    @property
    def _r(self):
        return self._world.r

    def _log(self, text, kind="info"):
        self._world.log(text, kind)

    def _fix(self, text):
        self._log("🔧 " + text, "fix")

    def _progress(self, text):
        """Show the command being executed (like show_progress in the real SDK)."""
        self._log("▶ " + text, "sdk")

    def _done(self, ok):
        if not ok:
            self._log("   ✗ stopped before reaching the target (blocked / obstacle avoidance) → success=False", "warn")
        return _result(ok)

    def _wait(self, sec):
        self._rv.wait(sec)

    @staticmethod
    def _choose(what, value, choices, lower=True):
        """Value must match exactly (case-insensitive when the real SDK lowercases it)."""
        if not isinstance(value, str):
            raise ValueError(f"{what} must be text, got {value!r}")
        v = value.strip().lower() if lower else value
        if v in choices:
            return v
        raise ValueError(f"invalid {what} {value!r} - valid: {sorted(set(choices))}.{_hint(value, choices)}")

    def _clamp(self, name, v, lo, hi):
        v = float(v)
        c = max(lo, min(hi, v))
        if c != v:
            self._fix(f"{name}={v:g} is outside [{lo:g}, {hi:g}] → using {c:g} (the real SDK clamps it too)")
        return c

    def _set_state(self, name, sleep=STATE_POST_SLEEP, quiet=False):
        w = self._world
        with w.lock:
            prev = self._r["fsm"]
            self._r["fsm"] = name
            if name in LYING_STATES:
                self._r["posture"] = "lie"
            elif name == "balance_stand" or name in LOCO_STATES:
                self._r["posture"] = "stand"
            if name in LOCO_STATES:
                self._r["gait"] = MODE_SPEED[name]["gait"]
            if name != "balance_stand":
                self._r["bal"] = [0.0, 0.0, 0.0, 0.0]
            self._r["action"] = None
        if not quiet:
            self._log(f"⚙ state: {prev} → {name}", "info")
        self._wait(sleep)
        return _result(True, state=name)

    def _ensure_loco(self):
        st = self._r["fsm"]
        if st in LYING_STATES:
            self._fix(f"robot is in '{st}' (lying down) → standing up first. "
                      "On the real robot call robot.balance_stand() yourself before walking")
            self.balance_stand()
        if self._r["fsm"] not in LOCO_STATES:
            self._set_state("walk", sleep=0.3, quiet=True)

    def _mode(self):
        return MODE_SPEED.get(self._r["fsm"], MODE_SPEED["walk"])

    def _with_ratio(self, speed_ratio):
        if speed_ratio is None:
            return self._r["speed_ratio"]
        return int(self._clamp("speed_ratio", int(speed_ratio), 10, 100))

    def _play(self, action, seconds, end_state=None):
        w = self._world
        with w.lock:
            self._r["action"] = action
            self._r["action_id"] += 1
            self._r["posture"] = "stand"
        self._wait(seconds)
        with w.lock:
            self._r["action"] = None
        if end_state:
            self._set_state(end_state, sleep=0.3, quiet=True)

    # ---------- context manager ----------
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    def close(self):
        """Close the connection (also releases every camera stream, like the real SDK)."""
        self.video.close_all()
        self._log("🔌 [sim] Connection closed", "sys")

    def __repr__(self):
        return f"<RobotClient(sim) addr='{self._addr}' state='{self._r['fsm']}'>"

    # ======================================================================
    # 2.2 Query
    # ======================================================================
    def get_state(self):
        """Full telemetry: current_state, current_speed_ratio, robot_state.pos_body, ..."""
        r = self._r
        c, s = math.cos(r["yaw"]), math.sin(r["yaw"])
        if r["posture"] == "lie":
            body = 0.11
        else:
            body = r["body_height"] + r["bal"][3]
        rs = types.SimpleNamespace(
            pos_body=[round(r["x"], 4), round(r["y"], 4), round(r["z"] + body, 4)],
            vel_body=[round(r["vx"] * c - r["vy"] * s, 4), round(r["vx"] * s + r["vy"] * c, 4), 0.0],
            acc_body=[0.0, 0.0, 9.81],
            omega_body=[0.0, 0.0, round(r["wz"], 4)],
            ori_body=[round(r["roll"], 4), round(r["pitch"], 4), round(r["yaw"], 4)],
            jpos_leg=[0.0] * 12, jpos_leg_des=[0.0] * 12, jvel_leg=[0.0] * 12, jvel_leg_des=[0.0] * 12,
            jtau_leg=[0.0] * 12, jtau_leg_des=[0.0] * 12,
            grf_left=[0.0, 0.0, 75.0], grf_right=[0.0, 0.0, 75.0], grf_vertical_filtered=[75.0, 75.0],
        )
        return _result(True, current_state=r["fsm"], current_speed_ratio=r["speed_ratio"],
                       obstacle_avoidance_enabled=r["avoid"], robot_state=rs)

    def get_motions(self):
        """List of motions supported by the robot."""
        return _result(True, motions=[types.SimpleNamespace(motion_id=m, parameters=[]) for m in sorted(MOTION_IDS)])

    def get_current_state_name(self):
        """Current state name, e.g. 'balance_stand', 'walk'."""
        return self._r["fsm"]

    def get_speed_ratio(self):
        """Current speed_ratio (10-100)."""
        return self._r["speed_ratio"]

    def get_obstacle_avoidance(self):
        """True if obstacle avoidance is enabled."""
        return self._r["avoid"]

    def get_robot_type(self):
        """Robot type: 'miniQuad' (legged) or 'miniQuadW' (wheel-legged)."""
        return "miniQuad"

    def is_quad(self):
        """True = legged robot."""
        return True

    def is_quad_wheel(self):
        """True = wheel-legged robot."""
        return False

    # ======================================================================
    # 2.3 Config
    # ======================================================================
    def set_speed_ratio(self, ratio):
        """Set the base speed 10-100 (%)."""
        v = int(self._clamp("speed_ratio", int(ratio), 10, 100))
        with self._world.lock:
            self._r["speed_ratio"] = v
        return _result(True, current_speed_ratio=v)

    def set_obstacle_avoidance(self, enable, _quiet=False):
        """Enable/disable obstacle avoidance: True/False or 'on'/'off'."""
        if isinstance(enable, str):
            enable = self._choose("obstacle avoidance setting", enable, ["on", "off"]) == "on"
        with self._world.lock:
            self._r["avoid"] = bool(enable)
        if not _quiet:
            self._log(f"🛡 obstacle avoidance = {'ON' if enable else 'OFF'}", "info")
        return _result(True, current_enabled=bool(enable))

    # ======================================================================
    # 2.4 execute
    # ======================================================================
    def execute(self, *motions, loop=False, show_progress=True):
        """Run motions in order: 'jump' or ('line_walk', {'direction': 0, 'distance': 1})."""
        res = None
        while True:
            for m in motions:
                if isinstance(m, str):
                    mid, params = m, {}
                else:
                    mid, params = m[0], dict(m[1])
                if mid not in MOTION_IDS:
                    raise ValueError(f"unknown motion '{mid}'.{_hint(mid, MOTION_IDS)} "
                                     "List them with robot.get_motions()")
                if mid == "line_walk":
                    res = self.line_walk(params.get("direction", 0), params.get("distance", 0))
                elif mid == "rotation":
                    res = self.rotate(params.get("direction", "left"), params.get("angle", 90))
                elif mid.endswith("_velocity_seq"):
                    res = self.velocity_sequence(params.get("velocity_sequence", ""),
                                                 gait=mid.replace("_velocity_seq", ""), stand_down_after=False)
                elif mid == "dance0":
                    res = self.dance(params.get("duration", DANCE_DEFAULT))
                elif mid == "wave":
                    res = self.wave_hand(params.get("duration", WAVE_DEFAULT))
                elif mid in BALANCE_MOTIONS:
                    res = self._balance(mid.replace("balance_", ""), params.get("value", 0.0),
                                        params.get("duration", 2.0), params.get("mode", "dynamic"))
                elif mid in ATOMIC_ACTIONS:
                    res = self.atomic_action(mid)
                else:
                    res = getattr(self, mid)(**params)
            if not loop:
                return res
            self._rv.sleep(CONFIG["dt"])        # a loop must stay stoppable even with no motions

    # ======================================================================
    # 2.5 State switching
    # ======================================================================
    def set_target_state(self, state_name, show_progress=True):
        """Switch state by name, e.g. 'balance_stand', 'walk', 'flying_trot'."""
        if not isinstance(state_name, str):
            raise ValueError(f"state name must be text, got {state_name!r}")
        norm = state_name.strip().lower()
        norm = STATE_ALIASES.get(norm, norm)
        if norm in WHEEL_ONLY_STATES:
            raise ValueError(f"'{norm}' only exists on the wheel-legged robot (miniQuadW); "
                             "the Rover X1 here is legged")
        norm = self._choose("state", norm, LEGGED_STATES)
        fn = {"dance0": self.dance, "wave": self.wave_hand}.get(norm) or getattr(self, norm)
        return fn()

    def passive(self, show_progress=True):
        """Motor power off - the robot collapses (emergency use)."""
        return self._set_state("passive")

    def emergency(self, show_progress=True):
        """Emergency stop (= passive)."""
        return self.passive()

    def ready(self, show_progress=True):
        """Lie down slowly and safely."""
        return self._set_state("ready")

    def stand_down(self, show_progress=True):
        """Lie down."""
        return self._set_state("stand_down")

    def balance_stand(self, show_progress=True):
        """Balanced stand (required before balance_* / dynamic_pose)."""
        return self._set_state("balance_stand")

    def walk(self, show_progress=True):
        """Switch to walk mode (does not move yet - use walk_forward / velocity_sequence)."""
        return self._set_state("walk")

    def rl(self, show_progress=True):
        """Switch to RL (reinforcement-learning) locomotion mode."""
        return self._set_state("rl")

    def flying_trot(self, show_progress=True):
        """Switch to running mode (faster than walk)."""
        return self._set_state("flying_trot")

    def change_mode(self, show_progress=True):
        """Toggle walk ⇄ run (flying_trot)."""
        new = "walk" if self._r["fsm"] == "flying_trot" else "flying_trot"
        self._log(f"⚙ change_mode → {new}", "info")
        return self._set_state(new, sleep=1.0, quiet=True)

    def gongxi(self, show_progress=True):
        """Congratulations bow gesture."""
        self._set_state("gongxi", sleep=0, quiet=True)
        self._log("🎉 gongxi", "info")
        self._play("gongxi", 4.0)
        return _result(True)

    def choreo(self, show_progress=True):
        """(deprecated) use gongxi()."""
        self._log("⚠ DeprecationWarning: choreo() was renamed to gongxi()", "warn")
        return self.gongxi()

    def dance(self, duration=DANCE_DEFAULT, show_progress=True):
        """Dance for `duration` seconds (4-128, default 126.5 like the real robot)."""
        d = float(duration)
        if not DANCE_MIN <= d <= DANCE_MAX:
            raise ValueError(f"dance duration must be in [{DANCE_MIN:g}, {DANCE_MAX:g}] seconds, got {duration}")
        self._set_state("dance0", sleep=0, quiet=True)
        self._log(f"💃 dance {d:g}s" + ("  (💡 use dance(duration=10) for a shorter dance)" if d > 60 else ""), "info")
        self._play("dance", d)
        return _result(True)

    def dance0(self, show_progress=True, duration=DANCE_DEFAULT):
        """Same as dance()."""
        return self.dance(duration)

    def wave_hand(self, duration=WAVE_DEFAULT, show_progress=True):
        """Wave a paw for `duration` seconds."""
        d = float(duration)
        if d < 0:
            raise ValueError(f"wave duration must be >= 0, got {duration}")
        self._set_state("wave", sleep=0, quiet=True)
        self._log("👋 wave", "info")
        self._play("wave", d + WAVE_TAIL)
        return _result(True)

    def wave(self, duration=WAVE_DEFAULT, show_progress=True):
        """Same as wave_hand()."""
        return self.wave_hand(duration)

    def jump(self, show_progress=True):
        """Jump."""
        self._set_state("jump", sleep=0, quiet=True)
        self._play("jump", 3.0)
        return _result(True)

    def backflip(self, show_progress=True):
        """Backflip."""
        self._set_state("backflip", sleep=0, quiet=True)
        self._play("backflip", STATE_POST_SLEEP + 0.5)
        return _result(True)

    def recovery(self, show_progress=True):
        """Get back up after a fall."""
        self._set_state("recovery", sleep=0, quiet=True)
        self._play("recovery", 4.0)
        return _result(True)

    # ---------- atomic choreography ----------
    def atomic_action(self, action_name, show_progress=True):
        """Short dance move: twirl_jump, diag_step, hop_step, groove, bounce, body_wave, hip_circle, head_circle."""
        name = self._choose("atomic action", action_name, list(ATOMIC_ACTIONS))
        self._log(f"🕺 {name}", "info")
        self._play(name, ATOMIC_ACTIONS[name], end_state="walk")
        return _result(True)

    def twirl_jump(self, show_progress=True):
        """Jump with a twirl."""
        return self.atomic_action("twirl_jump")

    def diag_step(self, show_progress=True):
        """Diagonal steps."""
        return self.atomic_action("diag_step")

    def hop_step(self, show_progress=True):
        """Front-leg hop steps."""
        return self.atomic_action("hop_step")

    def groove(self, show_progress=True):
        """Rhythmic sway."""
        return self.atomic_action("groove")

    def bounce(self, show_progress=True):
        """Nod and bounce."""
        return self.atomic_action("bounce")

    def body_wave(self, show_progress=True):
        """Body wave."""
        return self.atomic_action("body_wave")

    def hip_circle(self, show_progress=True):
        """Hip circles."""
        return self.atomic_action("hip_circle")

    def head_circle(self, show_progress=True):
        """Head circles."""
        return self.atomic_action("head_circle")

    # ---------- wheeled-only (not available here) ----------
    def _wheel_only(self, name):
        raise ValueError(f"{name}() only exists on the wheel-legged robot (miniQuadW); "
                         "the Rover X1 in this simulator is legged (miniQuad)")

    def wheel_loco(self, show_progress=True):
        """(wheel-legged robot only)"""
        self._wheel_only("wheel_loco")

    def drift(self, show_progress=True):
        """(wheel-legged robot only)"""
        self._wheel_only("drift")

    def handstand(self, show_progress=True):
        """(wheel-legged robot only)"""
        self._wheel_only("handstand")

    def climb(self, show_progress=True):
        """(wheel-legged robot only)"""
        self._wheel_only("climb")

    # ======================================================================
    # 2.6 Velocity sequence
    # ======================================================================
    @staticmethod
    def make_velocity_string(steps):
        """Convert [(vx, vy, vyaw, dur), ...] to 'vx,vy,vyaw,dur;...'."""
        if not steps:
            return ""
        for i, s in enumerate(steps):
            if not isinstance(s, (list, tuple)) or len(s) != 4:
                n = len(s) if isinstance(s, (list, tuple)) else 1
                raise ValueError(f"step {i + 1} needs 4 values (vx, vy, vyaw, duration) but got {n}: {s!r}")
        return ";".join(f"{vx},{vy},{vyaw},{dur}" for vx, vy, vyaw, dur in steps) + ";"

    def velocity_sequence(self, vel_seq, gait="walk", speed_ratio=None, stand_down_after=True,
                          show_progress=True):
        """Walk a velocity sequence [(vx m/s, vy m/s, vyaw rad/s, seconds), ...].
        gait: 'walk' or 'flying_trot'. stand_down_after=True lies down at the end."""
        if isinstance(vel_seq, (list, tuple)):
            vel_seq = self.make_velocity_string(vel_seq)
        if isinstance(gait, str) and gait.strip().lower() == "wheel_loco":
            self._wheel_only("velocity_sequence(gait='wheel_loco')")
        g = self._choose("gait", gait, GAITS)
        prev_sr = self._r["speed_ratio"]
        ratio = self._with_ratio(speed_ratio)
        self._r["speed_ratio"] = ratio
        try:
            self._ensure_loco()
            if self._r["fsm"] != g:
                self._set_state(g, sleep=0.5, quiet=True)
            mode = MODE_SPEED[g]
            vmax = mode["vmax"] * ratio / 100.0
            ok = True
            parts = [p for p in str(vel_seq).split(";") if p.strip()]
            self._progress(f"{g}_velocity_seq  {len(parts)} steps  speed_ratio={ratio}")
            limited = False
            for i, part in enumerate(parts):
                vals = [float(x) for x in part.split(",")]
                if len(vals) != 4:
                    raise ValueError(f"step {i + 1} needs 4 values (vx, vy, vyaw, duration) but got {len(vals)}")
                vx, vy, vyaw, dur = vals
                cvx = max(-vmax, min(vmax, vx))
                cvy = max(-vmax * 0.5, min(vmax * 0.5, vy))
                if (cvx, cvy) != (vx, vy) and not limited:
                    limited = True
                    self._fix(f"speed limited to {vmax:.2f} m/s forward / {vmax * 0.5:.2f} m/s sideways "
                              f"at speed_ratio {ratio} (simulator estimate)")
                if not self._rv.move(cvx, cvy, math.degrees(vyaw), dur):
                    ok = False
            if stand_down_after:
                self.stand_down()
            return self._done(ok)
        finally:
            self._r["speed_ratio"] = prev_sr

    # ======================================================================
    # 2.7 Line walk
    # ======================================================================
    def line_walk(self, direction=0, distance=3.0, speed_ratio=None, show_progress=True):
        """Walk straight. direction: 0/'forward', 1/'backward', 2/'left', 3/'right'. distance 0-3 m."""
        if isinstance(direction, str):
            direction = DIRECTION_MAP[self._choose("direction", direction, list(DIRECTION_MAP))]
        else:
            direction = int(direction)
            if direction not in (0, 1, 2, 3):
                raise ValueError(f"direction must be 0(front)/1(back)/2(left)/3(right), got {direction}")
        distance = self._clamp("distance", distance, 0.0, 3.0)
        prev_sr = self._r["speed_ratio"]
        ratio = self._with_ratio(speed_ratio)
        self._r["speed_ratio"] = ratio
        try:
            self._ensure_loco()
            self._progress(f"line_walk  {['forward', 'backward', 'left', 'right'][direction]} "
                           f"{distance:g} m  speed_ratio={ratio}")
            v = max(0.1, self._mode()["vmax"] * ratio / 100.0)
            vx, vy = [(v, 0), (-v * 0.6, 0), (0, v * 0.5), (0, -v * 0.5)][direction]
            ok = self._rv.travel(distance, vx, vy) if distance > 0 else True
            return self._done(ok)
        finally:
            self._r["speed_ratio"] = prev_sr

    def walk_forward(self, distance=3.0, speed_ratio=None, show_progress=True):
        """Walk forward `distance` meters (max 3)."""
        return self.line_walk(0, distance, speed_ratio)

    def walk_backward(self, distance=3.0, speed_ratio=None, show_progress=True):
        """Walk backward `distance` meters (max 3)."""
        return self.line_walk(1, distance, speed_ratio)

    def move_left(self, distance=3.0, speed_ratio=None, show_progress=True):
        """Side-step left `distance` meters (max 3)."""
        return self.line_walk(2, distance, speed_ratio)

    def move_right(self, distance=3.0, speed_ratio=None, show_progress=True):
        """Side-step right `distance` meters (max 3)."""
        return self.line_walk(3, distance, speed_ratio)

    # ======================================================================
    # 2.8 Rotation
    # ======================================================================
    def rotate(self, direction="left", angle=90.0, show_progress=True):
        """Turn in place. direction: 'left'/'right' or 0/1. angle 0-3600 degrees."""
        if isinstance(direction, str):
            direction = self._choose("rotate direction", direction, ["left", "right"])
        else:
            d = int(direction)                     # like the real SDK: 1.7 -> 1
            if d not in (0, 1):
                raise ValueError(f"rotate direction must be 0(left) or 1(right), got {direction!r}")
            direction = ["left", "right"][d]
        angle = float(angle)
        if angle < 0:
            self._fix(f"negative angle ({angle:g}) - the SDK uses `direction` for the side; it will be clamped to 0")
        angle = self._clamp("angle", angle, 0.0, 3600.0)
        self._ensure_loco()
        self._progress(f"rotation  {direction} {angle:g}°")
        speed = max(20.0, self._mode()["wmax"] * max(self._r["speed_ratio"], 30) / 100.0)
        self._rv.turn(angle if direction == "left" else -angle, speed)
        return _result(True)

    def rotate_left(self, angle=90.0, show_progress=True):
        """Turn left `angle` degrees."""
        return self.rotate("left", angle)

    def rotate_right(self, angle=90.0, show_progress=True):
        """Turn right `angle` degrees."""
        return self.rotate("right", angle)

    def circle(self, direction="left", turns=1, show_progress=True):
        """Spin `turns` full turns (1-10)."""
        turns = int(self._clamp("turns", int(turns), 1, 10))
        return self.rotate(direction, angle=turns * 360.0)

    def rotate_walk(self, angle=0.0, distance=0.0, speed_ratio=None, show_progress=True):
        """Turn toward `angle` (-180..180, positive = right, negative = left), then walk `distance` m."""
        angle = self._clamp("angle", angle, -180.0, 180.0)
        distance = self._clamp("distance", distance, 0.0, 3.0)
        if angle >= 0:
            self.rotate("right", angle)
        else:
            self.rotate("left", -angle)
        return self.walk_forward(distance, speed_ratio)

    # ======================================================================
    # 2.9 / 2.10 Balance & pose
    # ======================================================================
    def _ensure_balance(self):
        # like the real SDK, every balance motion first switches to balance_stand
        if self._r["fsm"] != "balance_stand":
            self._set_state("balance_stand", sleep=0.5)

    def _pose_anim(self, target, duration, mode):
        """target: dict index -> value  (0 roll, 1 pitch, 2 yaw, 3 height)"""
        w = self._world
        start = list(self._r["bal"])
        dt = CONFIG["dt"]
        t = 0.0
        while t < duration:
            t = min(duration, t + dt)
            u = t / duration
            with w.lock:
                for i, v in target.items():
                    if mode == "dynamic":       # sine sweep to the target, then hold
                        k = math.sin(math.pi / 2 * min(1.0, u * 1.25))
                    else:                       # static: ramp up - hold - ramp back
                        k = min(1.0, u / 0.25) if u < 0.75 else max(0.0, (1 - u) / 0.25)
                    self._r["bal"][i] = start[i] + (v - start[i]) * k
            self._rv.sleep(dt)
            with w.lock:
                w.sim_time += dt

    def _balance(self, axis, value, duration, mode):
        mode = self._choose("balance mode", mode, POSE_MODES, lower=False)
        duration = self._clamp("duration", duration, 0.5, 5.0)
        if axis == "neutral":
            self._ensure_balance()
            self._pose_anim({0: 0.0, 1: 0.0, 2: 0.0, 3: 0.0}, duration, "dynamic")
            return _result(True)
        if axis == "height":
            value = self._clamp("height", value, HEIGHT_MIN, HEIGHT_MAX)
            idx = 3
        else:
            lim = LIMITS[axis]
            value = self._clamp(axis, value, -lim, lim)
            idx = {"roll": 0, "pitch": 1, "yaw": 2}[axis]
        self._ensure_balance()
        self._progress(f"balance_{axis}  {value:g}{' m' if axis == 'height' else '°'}  {duration:g}s {mode}")
        self._pose_anim({idx: value}, duration, mode)
        return _result(True)

    def balance_pitch(self, value, duration=2.0, mode="dynamic", show_progress=True):
        """Pitch in degrees (-11.5..11.5, positive = lean forward)."""
        return self._balance("pitch", value, duration, mode)

    def balance_yaw(self, value, duration=2.0, mode="dynamic", show_progress=True):
        """Yaw in degrees (-11.5..11.5, positive = look right)."""
        return self._balance("yaw", value, duration, mode)

    def balance_roll(self, value, duration=2.0, mode="dynamic", show_progress=True):
        """Roll in degrees (-17..17, positive = lean left)."""
        return self._balance("roll", value, duration, mode)

    def balance_height(self, value, duration=2.0, mode="dynamic", show_progress=True):
        """Squat in meters (-0.08..0, negative = lower)."""
        return self._balance("height", value, duration, mode)

    def balance_neutral(self, duration=0.5, show_progress=True):
        """Return to the neutral pose."""
        return self._balance("neutral", 0.0, duration, "dynamic")

    def balance_sequence(self, motions, show_progress=True):
        """Several balance motions: [('balance_pitch', 10, 2.0, 'dynamic'), ...] (4 values each)."""
        for i, m in enumerate(motions):
            if not isinstance(m, (list, tuple)) or len(m) != 4:
                raise ValueError(f"balance step {i + 1} needs 4 values (motion_id, value, duration, mode), got {m!r}")
            motion_id, value, duration, mode = m
            if motion_id not in BALANCE_MOTIONS:       # exact, like the real SDK
                raise ValueError(f"unknown balance motion {motion_id!r} - valid: {BALANCE_MOTIONS}."
                                 f"{_hint(motion_id, BALANCE_MOTIONS)}")
            self._balance(motion_id.replace("balance_", ""), value, duration, mode)
        return _result(True)

    def _pose(self, duration, roll_deg, pitch_deg, yaw_deg, height_m, mode):
        duration = self._clamp("duration", duration, 1.0, 5.0)
        tgt = {}
        for i, (name, v) in enumerate((("roll", roll_deg), ("pitch", pitch_deg), ("yaw", yaw_deg))):
            if v:
                tgt[i] = self._clamp(name, v, -LIMITS[name], LIMITS[name])
        if height_m:
            tgt[3] = self._clamp("height", height_m, HEIGHT_MIN, HEIGHT_MAX)
        self._ensure_balance()
        if tgt:
            names = ["roll", "pitch", "yaw", "height"]
            self._progress(f"{mode}_pose  " + ", ".join(f"{names[i]}={v:g}" for i, v in tgt.items())
                           + f"  {duration:g}s")
            self._pose_anim(tgt, duration, mode)
        return _result(True)

    def dynamic_pose(self, duration=2.0, roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0, height_m=0.0,
                     show_progress=True):
        """Multi-axis pose (move to the target and hold). 0 = unchanged."""
        return self._pose(duration, roll_deg, pitch_deg, yaw_deg, height_m, "dynamic")

    def static_pose(self, duration=2.0, roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0, height_m=0.0,
                    show_progress=True):
        """Multi-axis pose (ramp up - hold - return)."""
        return self._pose(duration, roll_deg, pitch_deg, yaw_deg, height_m, "static")

    # ======================================================================
    # 2.13 LEDs (lights on the 4 legs) - same rules as the real SDK
    # ======================================================================
    @staticmethod
    def _main_leg(leg):
        if isinstance(leg, str):
            raise ValueError(f"leg must be Leg.FL / Leg.FR / Leg.RL / Leg.RR, not the text {leg!r}")
        if isinstance(leg, bool) or leg not in MAIN_LEGS:
            raise ValueError("Leg LED set APIs only support FL, FR, RL, RR; "
                             "fill lights (FILL_FRONT/FILL_BACK) are not supported")
        return Leg(int(leg))

    @staticmethod
    def _rgb_of(color):
        if isinstance(color, str):
            raise ValueError(f"use a Color constant such as Color.{color.strip().upper() or 'GREEN'} - "
                             f"on the real robot the text {color!r} silently turns the light OFF")
        if isinstance(color, bool) or color not in COLOR_RGB:
            raise ValueError(f"unknown color {color!r} - use Color.RED, Color.GREEN, ...")
        return COLOR_RGB[Color(int(color))]

    def _push_leds(self):
        out = []
        for lg in MAIN_LEGS:
            bri, r, g, b = self._led_cache.get(lg, (255, 0, 0, 0))
            out.append("#%02x%02x%02x" % tuple((int(v) * bri + 127) // 255 if bri < 255 else int(v)
                                               for v in (r, g, b)))
        with self._world.lock:
            self._r["leg_led"] = out

    def set_legs_rgb(self, configs, default_brightness=-1, verbose=False):
        """Several leg lights at once: set_legs_rgb([LegLedConfig(Leg.FL, 0, 0, 255), ...])."""
        if not configs:
            return True
        explicit = {}
        for cfg in configs:
            if not all(hasattr(cfg, a) for a in ("leg", "r", "g", "b", "brightness")):
                raise AttributeError(
                    f"set_legs_rgb() needs LegLedConfig items, got {cfg!r}. "
                    "💡 use set_legs_rgb([LegLedConfig(Leg.FL, 0, 0, 255), ...]) or set_leg_color(Leg.FL, Color.BLUE) "
                    "(the docs example set_legs_rgb([Leg.FL], Color.BLUE) fails on the real robot)")
            leg = self._main_leg(cfg.leg)
            if not (0 <= cfg.r <= 255) or not (0 <= cfg.g <= 255) or not (0 <= cfg.b <= 255):
                raise ValueError(f"RGB values must be 0-255, got ({cfg.r},{cfg.g},{cfg.b})")
            if cfg.brightness >= 0:
                bri = int(max(0, min(255, cfg.brightness)))
            elif default_brightness >= 0:
                bri = int(max(0, min(255, default_brightness)))
            else:
                bri = self._led_cache.get(leg, (255,))[0]
            explicit[leg] = (bri, int(cfg.r), int(cfg.g), int(cfg.b))
        for leg in MAIN_LEGS:              # like the real SDK, legs never set are sent as OFF
            self._led_cache[leg] = explicit.get(leg) or self._led_cache.get(leg, (255, 0, 0, 0))
        self._push_leds()
        return True

    def set_leg_rgb(self, leg, r, g, b, brightness=-1, verbose=False):
        """One leg light, RGB 0-255."""
        return self.set_legs_rgb([LegLedConfig(leg=leg, r=r, g=g, b=b, brightness=brightness)])

    def set_leg_color(self, leg, color, verbose=False):
        """One leg light with a preset Color.xxx."""
        r, g, b = self._rgb_of(color)
        return self.set_leg_rgb(leg, r, g, b, brightness=-1)

    def set_leg_brightness(self, leg, brightness, verbose=False):
        """Brightness of one leg light, 0-255 (keeps its color; white if never set)."""
        cached = self._led_cache.get(self._main_leg(leg))
        _, r, g, b = cached if cached else (255, 255, 255, 255)
        return self.set_leg_rgb(leg, r, g, b, brightness=brightness)

    def set_all_legs_rgb(self, r, g, b, brightness=255, verbose=False):
        """All 4 leg lights with the same RGB."""
        return self.set_legs_rgb([LegLedConfig(leg=lg, r=r, g=g, b=b, brightness=brightness) for lg in MAIN_LEGS])

    def set_all_legs_color(self, color, brightness=255, verbose=False):
        """All 4 leg lights with a preset Color.xxx."""
        r, g, b = self._rgb_of(color)
        return self.set_all_legs_rgb(r, g, b, brightness)

    def turn_off_leg(self, leg, verbose=False):
        """Turn one leg light off."""
        cached = self._led_cache.get(self._main_leg(leg))
        return self.set_leg_rgb(leg, 0, 0, 0, brightness=cached[0] if cached else 255)

    def reset_legs(self, legs=None, verbose=False):
        """Give the leg lights back to the robot's default behavior (None = all lights)."""
        if legs:
            for lg in legs:
                if not isinstance(lg, Leg):
                    raise AttributeError(f"reset_legs() needs Leg values such as Leg.FL, got {lg!r}")
            for lg in legs:
                self._led_cache.pop(lg, None)
        else:
            self._led_cache.clear()
        if not self._led_cache:
            with self._world.lock:
                self._r["leg_led"] = None
        else:
            self._push_leds()
        return True

    # ======================================================================
    # 2.12 Safety
    # ======================================================================
    def enable_safety_ready(self):
        """(real robot) Ctrl+C makes the robot lie down safely - in the simulator use the Stop button."""
        self._log("🛡 enable_safety_ready(): in the simulator, press ■ Stop instead of Ctrl+C", "info")

    # ---------- unknown attribute: fail like the real client, with a hint ----------
    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        target = resolve_sdk_name(name)
        hint = f" 💡 did you mean robot.{target}()?" if target and target != name else ""
        if not hint:
            sug = difflib.get_close_matches(name, SDK_FUNCTIONS, 3, 0.4)
            hint = f" 💡 did you mean {', '.join(sug)}?" if sug else ""
        raise AttributeError(f"'RobotClient' object has no attribute '{name}'.{hint}")


class _SimVideo:
    """robot.video - the robot's RGB camera streams (VideoStreamManager in the real SDK).

    The images are rendered from the 3D scene by the simulator page. open() blocks until frames
    really flow, like on the robot, and returns an http:// MJPEG address that cv2.VideoCapture
    and ffmpeg play the same way as the robot's rtsp://<ip>:8554/camera1 address.
    """
    READY_TIMEOUT = 8.0          # how long open() waits for the stream (same as the real SDK)

    def __init__(self, client):
        self._c = client
        self._lock = threading.Lock()
        self._refs = {}          # CameraId -> open() calls not closed yet
        self._info = {}          # CameraId -> last CameraStreamInfo
        self._listeners = []
        self._shut = False
        self._told = set()

    @property
    def _hub(self):
        return self._c._world.cams

    @property
    def host(self):
        return str(self._c._addr).rsplit(":", 1)[0]

    def uri(self, camera):
        """Address of this camera's stream; does not touch the robot."""
        return self._hub.uri(CameraId(camera))

    def list_cameras(self):
        return list(CAMERA_SPECS)

    def _set(self, cam, state, kind=None, reason=LossReason.NONE, uri="", message=""):
        old = self._info.get(cam, CameraStreamInfo(camera=cam)).state
        info = CameraStreamInfo(camera=cam, state=state, owner=StreamOwner.SDK if uri else StreamOwner.UNKNOWN,
                                uri=uri, reason=reason, message=message)
        self._info[cam] = info
        if kind is not None:
            event = StreamEvent(kind=kind, camera=cam, old_state=old, new_state=state, reason=reason,
                                uri=uri, message=message)
            for cb in list(self._listeners):
                try:
                    cb(event)
                except StopSim:
                    raise
                except Exception as e:  # noqa: BLE001 - a broken callback must not break streaming
                    self._c._log(f"⚠ video callback failed: {type(e).__name__}: {e}", "warn")

    def open(self, camera):
        """Starts this camera's stream and blocks until data is really flowing; returns the
        address to hand to cv2.VideoCapture. Raises VideoTimeoutError if no image arrives."""
        cam = CameraId(camera)
        hub = self._hub
        with self._lock:
            if self._shut:
                raise VideoError(f"{cam.name}: the video manager was shut down", camera=cam)
            self._refs[cam] = self._refs.get(cam, 0) + 1
            already = self._refs[cam] > 1 and self._info.get(cam) and self._info[cam].state is StreamState.READY
        hub.want(int(cam), +1)
        if already:
            return self._info[cam].uri
        self._set(cam, StreamState.PREPARING)
        after = hub.last_seq(int(cam))
        deadline = time.monotonic() + self.READY_TIMEOUT
        while hub.wait_new(int(cam), after, 0.2) is None:
            self._c._rv.check_stop()
            if time.monotonic() >= deadline:
                with self._lock:
                    self._refs[cam] -= 1
                    if self._refs[cam] <= 0:
                        del self._refs[cam]
                hub.want(int(cam), -1)
                self._set(cam, StreamState.STOPPED, reason=LossReason.NO_DATA,
                          message="open() failed; local state released")
                self._c._log("💡 [sim] camera images come from the simulator page in the browser: "
                              "keep it open (not minimised) while the program runs", "warn")
                raise VideoTimeoutError(
                    f"{cam.name}: the stream was not ready within {self.READY_TIMEOUT:.1f} s. "
                    f"It may still be starting up: try again in a few seconds.",
                    reason=LossReason.NO_DATA, camera=cam)
        uri = hub.uri(cam)
        self._set(cam, StreamState.READY, StreamEventKind.READY, uri=uri)
        if cam not in self._told:
            self._told.add(cam)
            self._c._log(f"📷 [sim] {cam.name} streams the 3D view at {uri} "
                         f"(the real robot gives rtsp://<robot ip>:8554/{cam.rtsp_name})", "info")
        return uri

    def is_streaming(self, camera):
        cam = CameraId(camera)
        with self._lock:
            open_ = self._refs.get(cam, 0) > 0
        return open_ and self._info.get(cam) is not None and self._info[cam].state is StreamState.READY

    def get_stream_info(self, camera):
        cam = CameraId(camera)
        return self._info.get(cam) or CameraStreamInfo(camera=cam)

    def on_state_change(self, callback):
        """Subscribes to stream events; the returned function unsubscribes."""
        if not callable(callback):
            raise TypeError("callback must be callable")
        self._listeners.append(callback)

        def _unsubscribe():
            try:
                self._listeners.remove(callback)
            except ValueError:
                pass
        return _unsubscribe

    def close(self, camera):
        """Stops using this camera (idempotent); the stream stops when the last user releases it."""
        cam = CameraId(camera)
        with self._lock:
            n = self._refs.get(cam, 0)
            if n == 0:
                return
            if n > 1:
                self._refs[cam] = n - 1
            else:
                del self._refs[cam]
        self._hub.want(int(cam), -1)
        if n == 1:
            self._set(cam, StreamState.STOPPED, StreamEventKind.CLOSED, reason=LossReason.CLOSED,
                      uri=self._info.get(cam, CameraStreamInfo(camera=cam)).uri,
                      message="Stream stopped, resources released")

    def stop_stream(self, camera):
        cam = CameraId(camera)
        with self._lock:
            n = self._refs.pop(cam, 0)
        if n:
            self._hub.want(int(cam), -n)
            self._set(cam, StreamState.STOPPED, StreamEventKind.CLOSED, reason=LossReason.CLOSED,
                      message="Stream stopped")

    def close_all(self, *, reason=LossReason.CLOSED, message=""):
        """Releases every camera (idempotent)."""
        with self._lock:
            refs, self._refs = self._refs, {}
        for cam, n in refs.items():
            self._hub.want(int(cam), -n)
            self._set(cam, StreamState.STOPPED, StreamEventKind.CLOSED, reason=reason, message=message)

    def shutdown(self):
        self.close_all()
        self._shut = True

    close_manager = shutdown

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        self.shutdown()


class _GrpcFutureTimeoutError(Exception):
    pass


class _GrpcRpcError(Exception):
    pass


class _SimChannel:
    def __init__(self, target):
        self.target = target
        self._closed = False

    def close(self):
        self._closed = True

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        self.close()


class _SimReadyFuture:
    def __init__(self, channel):
        self._ch = channel

    def result(self, timeout=None):
        if not isinstance(self._ch, _SimChannel):
            raise TypeError("channel_ready_future() needs a channel from grpc.insecure_channel()")
        if self._ch._closed:
            raise ValueError("Cannot invoke RPC on closed channel!")
        return None

    def done(self):
        return True

    def cancel(self):
        return False


def make_grpc_module():
    """The part of grpc that robot programs use to wait for the robot; in the simulator every
    channel reaches the simulated robot at once. Anything else is reported as missing."""
    mod = types.ModuleType("grpc")
    mod.__doc__ = "Simulated grpc (Rover X1 Simulator): channels reach the simulated robot"
    mod.insecure_channel = lambda target, options=None, compression=None: _SimChannel(target)
    mod.channel_ready_future = _SimReadyFuture
    mod.FutureTimeoutError = _GrpcFutureTimeoutError
    mod.RpcError = _GrpcRpcError

    def __getattr__(name):
        raise AttributeError(f"module 'grpc' has no attribute '{name}' in the simulator - it only provides "
                             f"insecure_channel, channel_ready_future, FutureTimeoutError and RpcError. "
                             f"💡 Talk to the robot through dobot_quad.RobotClient")
    mod.__getattr__ = __getattr__
    return mod


# ----------------------------------------------------------------------------
# Command lists / auto-correct helpers
# ----------------------------------------------------------------------------
MOTION_IDS = (set(LEGGED_STATES) | set(ATOMIC_ACTIONS) | set(BALANCE_MOTIONS) |
              {"line_walk", "rotation", "walk_velocity_seq", "flying_trot_velocity_seq",
               "dynamic_pose", "static_pose", "change_mode"})

SDK_FUNCTIONS = sorted(n for n, v in SimRobotClient.__dict__.items()
                       if callable(v) and not n.startswith("_"))
_HIDE = {"show_progress", "verbose", "self", "_quiet"}
SDK_PARAMS = {}        # shown in the editor hints
SDK_ALL_PARAMS = {}    # every keyword the real SDK accepts (incl. show_progress / verbose)
for _n in SDK_FUNCTIONS:
    _sig = inspect.signature(getattr(SimRobotClient, _n)).parameters
    SDK_PARAMS[_n] = [p for p in _sig if p not in _HIDE]
    SDK_ALL_PARAMS[_n] = [p for p in _sig if p not in ("self", "_quiet")]
SDK_ATTRIBUTES = {"video"}                     # non-method attributes of the real client

# values the real SDK accepts for text parameters: (method, parameter) -> choices
TEXT_VALUES = {
    ("rotate", "direction"): ["left", "right"],
    ("circle", "direction"): ["left", "right"],
    ("line_walk", "direction"): list(DIRECTION_MAP),
    ("velocity_sequence", "gait"): GAITS,
    ("set_target_state", "state_name"): LEGGED_STATES + list(STATE_ALIASES),
    ("atomic_action", "action_name"): list(ATOMIC_ACTIONS),
    ("set_obstacle_avoidance", "enable"): ["on", "off"],
    ("balance_pitch", "mode"): POSE_MODES, ("balance_yaw", "mode"): POSE_MODES,
    ("balance_roll", "mode"): POSE_MODES, ("balance_height", "mode"): POSE_MODES,
}
CASE_SENSITIVE_VALUES = {"mode"}               # passed to the robot unchanged
COLOR_PARAMS = {("set_all_legs_color", "color"), ("set_leg_color", "color")}

SDK_ALIASES = {
    "walk_left": "move_left", "walk_right": "move_right",       # old names in the SDK
    "forward": "walk_forward", "backward": "walk_backward", "move_forward": "walk_forward",
    "move_backward": "walk_backward", "go_forward": "walk_forward",
    "turn": "rotate", "turn_left": "rotate_left", "turn_right": "rotate_right",
    "stand": "balance_stand", "stand_up": "balance_stand", "standup": "balance_stand",
    "sit": "stand_down", "lie_down": "stand_down", "laydown": "stand_down",
    "run": "flying_trot", "wave_hello": "wave_hand", "hello": "wave_hand",
    "flip": "backflip", "back_flip": "backflip", "dance_0": "dance0",
    "stop": "ready", "estop": "emergency", "e_stop": "emergency",
    "set_speed": "set_speed_ratio", "speed_ratio": "set_speed_ratio",
    "get_speed": "get_speed_ratio", "state": "get_current_state_name",
    "set_led": "set_all_legs_color", "led": "set_all_legs_color", "set_color": "set_all_legs_color",
}


def resolve_sdk_name(name):
    if name in SDK_FUNCTIONS:
        return name
    low = name.lower()
    snake = "".join("_" + c.lower() if c.isupper() else c for c in name).lstrip("_")   # camelCase
    for cand in (low, snake):
        if cand in SDK_FUNCTIONS:
            return cand
        if cand in SDK_ALIASES:
            return SDK_ALIASES[cand]
        if cand.startswith("set_") and cand[4:] in SDK_FUNCTIONS and cand[4:] in LEGGED_STATES + ["balance_stand"]:
            return cand[4:]      # the SDK dropped the set_ prefix
    squashed = low.replace("_", "")
    for n in SDK_FUNCTIONS:
        if n.replace("_", "") == squashed:
            return n
    m = difflib.get_close_matches(low, SDK_FUNCTIONS + list(SDK_ALIASES), 1, 0.75)
    if m:
        return SDK_ALIASES.get(m[0], m[0])
    return None


SDK_EXPORTS = ["RobotClient", "Leg", "Color", "LegLedConfig", "VALID_STATES", "VALID_BALANCE_MOTIONS",
               "VALID_GAITS", "VALID_WHEEL_STATES", "VALID_WHEEL_GAITS", "VALID_ATOMIC_ACTIONS",
               "CameraId", "CameraStreamInfo", "StreamEvent", "StreamState", "VideoConnectionError",
               "VideoError", "VideoTimeoutError"]
SDK_CONSTS = {"Leg": [m.name for m in Leg], "Color": [m.name for m in Color],
              "CameraId": [m.name for m in CameraId]}


def make_module(world):
    """Build a fake 'dobot_quad' module bound to the simulated world."""
    mod = types.ModuleType("dobot_quad")
    mod.__doc__ = "Simulated dobot_quad (Rover X1 Simulator)"
    mod.RobotClient = type("RobotClient", (SimRobotClient,), {"_world": world, "__doc__": SimRobotClient.__doc__})
    g = globals()
    for name in SDK_EXPORTS[1:]:
        setattr(mod, name, g[name])
    mod.__all__ = list(SDK_EXPORTS)
    return mod


def sdk_docs():
    out = []
    for n in SDK_FUNCTIONS:
        fn = getattr(SimRobotClient, n)
        doc = (fn.__doc__ or "").strip().split("\n")[0]
        out.append({"name": n, "sig": "(" + ", ".join(SDK_PARAMS[n]) + ")", "doc": doc})
    return out
