# Rover X1 Simulator 🐕

A 3D simulator for the **Dobot Rover X1** quadruped. Write Python with the **same commands as the
real robot** ([dobot_quad SDK](https://github.com/Dobot-Team/dobot_quad_sdk)), with Auto-correct,
a map editor and a built-in manual (Thai).

> **Unofficial project.** This simulator is not affiliated with, endorsed by or sponsored by Dobot.
> "Dobot" and "Rover X1" are trademarks of their owners and are used here only to say which robot and
> SDK the simulator is compatible with. It contains no code from the Dobot SDK: the simulated
> `dobot_quad` module is written from scratch so that the same program can run on the real robot.

## Start

- Double-click **`RoverX1Sim.exe`** on the Desktop, or
- run `python server.py` (or `run.bat`) in this folder

The browser opens http://localhost:8765. Close the console window to quit.
Needs Python 3.10+ only when running from source. **Works offline**: three.js r160 and CodeMirror
5.65.16 are bundled in `static/vendor/` (only the web fonts come from the internet; without it the
page uses the Thai fonts built into Windows).

## Real robot commands first

`robot.*` is a simulated `RobotClient` with the command names, parameters and value ranges of the
real high-level SDK. Test here, then run the same file on the real robot (change only the IP).

```python
from dobot_quad import RobotClient

robot = RobotClient("192.168.5.2:50051")
robot.balance_stand()
robot.walk_forward(2.0)
robot.rotate("left", 90)
robot.stand_down()
```

- The robot starts lying down (`stand_down`), so call `balance_stand()` first, like the real one.
- Obstacle avoidance is on in the simulator; on the real robot call `robot.set_obstacle_avoidance(True)`
  yourself (the current SDK code does not switch it on). Walk commands return `.success`.
- The Console shows each command as it runs (▶) and tells you when a value was clamped.

**Poka-yoke - the simulator is at least as strict as the real SDK** (rules taken from the SDK source):

- no simulator-only commands, and nothing pre-loaded: every program imports `dobot_quad` and creates
  `robot = RobotClient(...)` itself (Auto-correct adds missing lines)
- whatever the real SDK rejects fails here too, with a 💡 hint: wrong text values such as
  `rotate("lefft")`, `dance(duration=1)`, RGB outside 0-255, unknown methods
- things that silently misbehave on the robot are errors here: text colors (`"green"` turns the light
  off on the robot), the docs form `set_legs_rgb([Leg.FL], Color.BLUE)` (crashes on the robot)

## Auto-correct

Fixes typos in command / argument / import names, missing `:` and `)`, curly quotes used as quotes,
text values the SDK rejects (`"lefft"` → `"left"`, `"ซ้าย"` → `"left"`, `"green"` → `Color.GREEN`),
missing `import` / `robot = RobotClient(...)` lines, and Thai input mistakes: a line typed with the
Thai keyboard layout, Thai digits, Thai notes without `#`. All fixes change the source code, so the
corrected program runs on the real robot. SDK commands called without `robot.` get the prefix added;
the old `rover.*` helpers are reported as errors together with the SDK command to use instead.
Thai variable names, strings, docstrings and comments are left alone.

## Tests

131 automated tests (standard `unittest`, nothing to install) cover Auto-correct (incl. a fuzz test),
real-SDK fidelity, stairs / ledges / collisions / obstacle avoidance, the program runner (Stop,
`time.sleep`, `input`, `exit`), the web server (CSRF / DNS rebinding, bad requests, file access, several
windows) and run every example and every code sample of the manual.

```
python -m unittest discover -s tests -v
```

or double-click `run_tests.bat`. Run them after every change, before rebuilding the .exe.

## Files

| File | Purpose |
|---|---|
| `server.py` | web server (standard library only) + API |
| `rover_sim.py` | simple physics, map, collisions, stairs, program runner |
| `sdk_compat.py` | simulated `RobotClient` following the Dobot high-level SDK |
| `autocorrect.py` | code checker / fixer |
| `static/` | 3D page (three.js), editor (CodeMirror), manual, examples, preset maps |
| `build_exe.bat` | rebuild `RoverX1Sim.exe` (needs `pip install pyinstaller`) |
| `tests/`, `run_tests.bat` | automated tests |

Tune speeds and body size in `CONFIG` (`rover_sim.py`) and `MODE_SPEED` (`sdk_compat.py`).
They are estimates - the SDK documentation does not list real speeds.

## Credits

- three.js and CodeMirror (MIT License) are bundled in `static/vendor/`; their license notices are in
  [`static/vendor/THIRD_PARTY_NOTICES.md`](static/vendor/THIRD_PARTY_NOTICES.md).
- The app icon was generated with Figma AI.
