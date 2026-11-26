// Example Python programs for the Rover X1.
// All of them use only the real robot API (dobot_quad), so they also run on the real robot.
window.EXAMPLES = {
  "1. Getting started": `# This program uses the same commands as Dobot's real SDK
# https://github.com/Dobot-Team/dobot_quad_sdk
from dobot_quad import RobotClient

robot = RobotClient("192.168.5.2:50051")   # real robot: LAN 192.168.5.2 / WiFi 192.168.1.6

robot.balance_stand()           # stand up and balance
robot.walk_forward(2.0)         # walk forward 2 m (max 3 m per command)
robot.move_left(1.0)            # side-step left 1 m
robot.rotate("left", 90)        # turn left 90°
robot.walk_forward(1.5)
robot.wave_hand(3)              # wave a paw
robot.stand_down()              # lie down
print("state:", robot.get_current_state_name())
`,

  "2. Loops: square and star": `from dobot_quad import RobotClient, Color

robot = RobotClient("192.168.5.2:50051")
robot.balance_stand()

# a 2 x 2 m square
robot.set_all_legs_color(Color.GREEN)
for side in range(4):
    robot.walk_forward(2.0)
    robot.rotate_left(90)
    print("side", side + 1, "done")

# a 5-pointed star - first move away from the field edge, the star needs room
robot.move_left(1.5)
robot.set_all_legs_color(Color.YELLOW)
for point in range(5):
    robot.walk_forward(1.5)
    robot.rotate_right(144)

x, y, z = robot.get_state().robot_state.pos_body
print(f"back where the star started: x={x:.2f} y={y:.2f}")
robot.reset_legs()
robot.stand_down()
`,

  "3. velocity_sequence + speed_ratio": `from dobot_quad import RobotClient

robot = RobotClient("192.168.5.2:50051")
robot.balance_stand()
robot.set_speed_ratio(60)       # speed 10-100 %

# (vx m/s, vy m/s, vyaw rad/s, seconds)
steps = [
    (0.8, 0, 0, 2),      # forward for 2 s
    (0, 0, 0, 1),        # stop for 1 s
    (0.5, 0, 0.6, 3),    # curve to the left
    (0, 0.3, 0, 2),      # side-step left
    (-0.5, 0, 0, 2),     # backward
]
robot.velocity_sequence(steps, gait="walk", speed_ratio=60, stand_down_after=False)

robot.change_mode()             # switch to running (flying_trot)
robot.walk_forward(2.0, speed_ratio=100)
robot.change_mode()             # back to walking
robot.circle("right", 1)
robot.stand_down()
`,

  "4. Write go_to(x, y) and collect the flags": `# Uses get_state() + rotate + walk_forward
# (the flags are on the "Starter field" map - press Reset if you changed the map)
import math
from dobot_quad import RobotClient

robot = RobotClient("192.168.5.2:50051")
robot.balance_stand()

def pose():
    s = robot.get_state().robot_state
    x, y, _ = s.pos_body
    yaw = math.degrees(s.ori_body[2])          # ori_body = [roll, pitch, yaw] in radians
    return x, y, yaw

def go_to(tx, ty):
    x, y, yaw = pose()
    target = math.degrees(math.atan2(ty - y, tx - x))
    diff = (target - yaw + 180) % 360 - 180    # -180..180, positive = turn left
    if diff > 1:
        robot.rotate_left(diff)
    elif diff < -1:
        robot.rotate_right(-diff)
    dist = math.hypot(tx - x, ty - y)
    while dist > 0.05:
        step = min(dist, 3.0)                  # the SDK walks at most 3 m per command
        if not robot.walk_forward(step).success:
            print("blocked by an obstacle")
            return False
        dist -= step
    return True

flags = [(8.25, 1.75), (8.25, 8.25), (1.75, 8.25)]
for fx, fy in flags:
    print("go to", (fx, fy), "->", "ok" if go_to(fx, fy) else "failed")
robot.wave_hand(2)
robot.stand_down()
`,

  "5. Obstacle avoidance with .success": `# With obstacle avoidance on, the robot stops before it hits something and the
# walk command returns success=False. Use that to decide where to go next.
from dobot_quad import RobotClient, Color

robot = RobotClient("192.168.5.2:50051")
robot.set_obstacle_avoidance(True)     # turn it on yourself - do not rely on the default
robot.balance_stand()
robot.set_speed_ratio(60)

for attempt in range(12):
    result = robot.walk_forward(1.0)
    if result.success:
        robot.set_all_legs_color(Color.GREEN)
    else:
        print("obstacle ahead -> turn right")
        robot.set_all_legs_color(Color.RED)
        robot.walk_backward(0.3)
        robot.rotate_right(90)

x, y, z = robot.get_state().robot_state.pos_body
print(f"final position x={x:.2f} y={y:.2f}")
robot.reset_legs()
robot.stand_down()
`,

  "6. Balance + leg lights + tricks": `from dobot_quad import RobotClient, Leg, Color, LegLedConfig

robot = RobotClient("192.168.5.2:50051")
robot.balance_stand()

# lights on the 4 legs
robot.set_all_legs_color(Color.GREEN)
robot.set_leg_color(Leg.FL, Color.WHITE)
robot.set_leg_color(Leg.FR, Color.WHITE)
robot.set_leg_rgb(Leg.RR, 255, 0, 128)
# several legs in one call need LegLedConfig items (r, g, b = 0-255)
robot.set_legs_rgb([LegLedConfig(Leg.RL, 0, 0, 255), LegLedConfig(Leg.RR, 255, 0, 128)])

# balance poses (only in balance_stand)
robot.balance_pitch(10, 2.0, "dynamic")     # lean forward
robot.balance_roll(-15, 2.0, "static")      # lean right and come back
robot.balance_height(-0.06, 1.5)            # squat
robot.dynamic_pose(3.0, roll_deg=10, pitch_deg=-8, yaw_deg=10, height_m=-0.03)
robot.balance_neutral()

# tricks
robot.groove()
robot.jump()
robot.twirl_jump()
robot.dance(duration=8)
robot.backflip()
robot.reset_legs()
robot.stand_down()
`,

  "7. States + safety": `from dobot_quad import RobotClient

robot = RobotClient("192.168.5.2:50051")
robot.enable_safety_ready()             # real robot: Ctrl+C makes it lie down safely
print("start:", robot.get_current_state_name())

for st in ["balance_stand", "walk", "flying_trot", "balance_stand"]:
    robot.set_target_state(st)
    print("now:", robot.get_current_state_name())

robot.set_speed_ratio(30)               # go slow the first time
robot.walk_forward(1.0)
robot.change_mode()
print("after change_mode:", robot.get_current_state_name())
robot.walk_forward(1.0, speed_ratio=100)
s = robot.get_state()
print("speed_ratio:", s.current_speed_ratio, "obstacle avoidance:", s.obstacle_avoidance_enabled)
robot.ready()                           # lie down safely
`,

  "8. Climb the Stairs": `# Choosing this example loads the "Stairs" map automatically
from dobot_quad import RobotClient

robot = RobotClient()
robot.balance_stand()
robot.set_speed_ratio(40)
print(robot.walk_forward(3).success)    # up the steps: 0.10 -> 0.14 -> 0.24 m
print(robot.walk_forward(3).success)    # over the top (flag 1) and down again
print(robot.walk_forward(2).success)    # off the last step to flag 2
x, y, z = robot.get_state().robot_state.pos_body
print(f"x={x:.2f}  body height z={z:.2f}")
robot.wave_hand(2)
`,

  "9. Typos on purpose (Auto-correct)": `# Several mistakes on purpose - press "🔧 Check code" or Run
from dobotquad import RobotClinet, Colour

robot = RobotClinet("192.168.5.2:50051")
robot.balanceStand()
robot.walk_forwrd(5, sped_ratio=70)    # the SDK walks at most 3 m per command
robot.walk_left(1)                      # old SDK name -> move_left
rotate_left(90)                         # forgot "robot."
robot.rotate("lefft", 90)
robot.set_all_legs_color(Colour.GREN)
robot.balance_pitch(20)                 # above ±11.5° gets clamped
# the next line is robot.walk_forward(1) typed with the Thai keyboard layout by mistake:
พนินะใไฟสา๘ดนพไฟพก๖ๅ๗
robot.stand_down() หมอบตอนจบ
`,
};

// ---------------------------------------------------------------------------
// Preset maps (meters, 1 cell = 0.5 m, positions are cell centers)
// ---------------------------------------------------------------------------
(function () {
  const C = (i) => i * 0.5 + 0.25;       // cell index -> center coordinate
  const box = (i, j, h, color) => ({ x: C(i), y: C(j), w: 0.5, d: 0.5, h, color });
  const WALL = "#8d99ae", BLOCK = "#e07a5f", STEP = "#f2cc8f", STEP2 = "#e9b872";

  function starter() {
    const boxes = [];
    for (let k = 0; k < 4; k++) boxes.push(box(10, 4 + k, 0.6, WALL));
    boxes.push(box(5, 13, 0.3, BLOCK));
    boxes.push(box(14, 14, 0.1, STEP));
    boxes.push(box(15, 14, 0.1, STEP));
    return { name: "Starter field", size: [10, 10], start: { x: 1.25, y: 1.25, yaw: 0 }, boxes,
      goals: [{ x: 8.25, y: 1.75 }, { x: 8.25, y: 8.25 }, { x: 1.75, y: 8.25 }] };
  }

  function empty() {
    return { name: "Empty", size: [10, 10], start: { x: 1.25, y: 1.25, yaw: 0 }, boxes: [], goals: [{ x: 8.25, y: 8.25 }] };
  }

  function maze() {
    // 0 = path, 1 = wall  (top row = largest y)  20x20 cells = 10x10 m
    const g = [
      "11111111111111111111",
      "10000001000000000001",
      "10000001000000000001",
      "10011111001111111001",
      "10010000001000000001",
      "10010000001000000001",
      "10010011111001111111",
      "10000010000001000001",
      "10000010000001000001",
      "11111110011111001001",
      "10000000010000001001",
      "10000000010000001001",
      "10011111110011111001",
      "10010000000010000001",
      "10010000000010000001",
      "10010011111110011111",
      "10000010000000000001",
      "10000010000000000001",
      "10000010000000000001",
      "11111111111111111111",
    ];
    const boxes = [];
    for (let r = 0; r < 20; r++) for (let c = 0; c < 20; c++) {
      if (g[r][c] === "1") boxes.push(box(c, 19 - r, 0.6, WALL));
    }
    return { name: "Maze", size: [10, 10], start: { x: C(2), y: C(2), yaw: 0 }, boxes, goals: [{ x: C(17), y: C(17) }] };
  }

  function slalom() {
    const boxes = [];
    for (let k = 0; k < 6; k++) {
      const i = 3 + k * 3;
      const j = k % 2 === 0 ? 4 : 11;
      for (let t = 0; t < 5; t++) boxes.push(box(i, j + t, 0.3, BLOCK));
    }
    return { name: "Slalom", size: [10, 8], start: { x: 0.75, y: 4, yaw: 0 }, boxes, goals: [{ x: 9.25, y: 4.25 }] };
  }

  function stairs() {
    const boxes = [];
    for (let j = 6; j < 10; j++) {
      boxes.push(box(6, j, 0.10, STEP));
      boxes.push(box(7, j, 0.10, STEP));
      boxes.push(box(8, j, 0.14, STEP2));
      boxes.push(box(9, j, 0.14, STEP2));
      boxes.push(box(10, j, 0.24, STEP2));     // top step: +0.10 above the 2nd one, still climbable
      boxes.push(box(11, j, 0.24, STEP2));
      boxes.push(box(12, j, 0.14, STEP2));     // and down again on the other side
      boxes.push(box(13, j, 0.14, STEP2));
      boxes.push(box(14, j, 0.10, STEP));
      boxes.push(box(15, j, 0.10, STEP));
    }
    for (let i = 4; i < 18; i++) { boxes.push(box(i, 5, 0.6, WALL)); boxes.push(box(i, 10, 0.6, WALL)); }
    return { name: "Stairs", size: [10, 8], start: { x: 1.25, y: 4, yaw: 0 }, boxes, goals: [{ x: C(11), y: 4 }, { x: 9.25, y: 4 }] };
  }

  function arena() {
    const boxes = [];
    const pillars = [[4, 4], [4, 15], [15, 4], [15, 15], [9, 9], [10, 9], [9, 10], [10, 10]];
    pillars.forEach(([i, j]) => boxes.push(box(i, j, 1.2, WALL)));
    for (let i = 6; i < 14; i++) boxes.push(box(i, 2, 0.3, BLOCK));
    for (let j = 6; j < 14; j++) boxes.push(box(17, j, 0.3, BLOCK));
    return {
      name: "Arena", size: [10, 10], start: { x: 1.25, y: 1.25, yaw: 45 }, boxes,
      goals: [{ x: 8.75, y: 1.25 }, { x: 8.75, y: 8.75 }, { x: 1.25, y: 8.75 }, { x: 3.75, y: 5.25 }],
    };
  }

  window.PRESETS = { "Starter field": starter, Empty: empty, Maze: maze, Slalom: slalom, Stairs: stairs, Arena: arena };
})();
