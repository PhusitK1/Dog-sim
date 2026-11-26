// Rover X1 Simulator - frontend
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

const $ = (id) => document.getElementById(id);
const api = async (path, body) => {
  const opt = body === undefined ? {} : { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) };
  const r = await fetch(path, opt);
  try {
    return await r.json();
  } catch (e) {
    throw new Error(`HTTP ${r.status}`);
  }
};
const store = {
  get(k, d) { try { const v = localStorage.getItem(k); return v === null ? d : v; } catch { return d; } },
  set(k, v) { try { localStorage.setItem(k, v); } catch { /* ignore */ } },
};

// sim (x, y, z-up)  ->  three (x, y-up, z = -y)
const S2T = (x, y, h = 0) => new THREE.Vector3(x, h, -y);
const CELL = 0.5;

// ============================================================================
// Three.js scene
// ============================================================================
const view = $("view");
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.outputColorSpace = THREE.SRGBColorSpace;
view.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0f1216);
scene.fog = new THREE.Fog(0x0f1216, 18, 45);

const camera = new THREE.PerspectiveCamera(50, 1, 0.05, 200);
// Layer 1 = helpers that exist only in the editor view (axes, start marker, trail, cursor):
// the main view shows them, the robot's own cameras do not.
const EDITOR_LAYER = 1;
camera.layers.enable(EDITOR_LAYER);
const editorOnly = (o) => { o.traverse((c) => c.layers.set(EDITOR_LAYER)); return o; };
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.maxPolarAngle = Math.PI * 0.495;
controls.minDistance = 1;
controls.maxDistance = 40;

scene.add(new THREE.HemisphereLight(0xcfe3ff, 0x2a2018, 0.9));
const sun = new THREE.DirectionalLight(0xffffff, 1.6);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
sun.shadow.bias = -0.0005;
scene.add(sun, sun.target);

function resize() {
  const w = view.clientWidth, h = view.clientHeight;
  renderer.setSize(w, h, false);
  renderer.domElement.style.width = w + "px";
  renderer.domElement.style.height = h + "px";
  camera.aspect = w / Math.max(h, 1);
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(view);

// ============================================================================
// Map
// ============================================================================
let MAP = null;
let mapVersion = -1;
const mapGroup = new THREE.Group();
scene.add(mapGroup);
let goalMeshes = [];
let groundMesh = null;
let boxMeshes = [];

function disposeGroup(g) {
  g.traverse((o) => { if (o.geometry) o.geometry.dispose(); if (o.material) [].concat(o.material).forEach((m) => m.dispose()); });
  g.clear();
}

function buildMap(m) {
  MAP = m;
  disposeGroup(mapGroup);
  goalMeshes = [];
  boxMeshes = [];
  const [W, D] = m.size;

  // ground
  const ground = new THREE.Mesh(new THREE.PlaneGeometry(W, D),
    new THREE.MeshStandardMaterial({ color: 0x2b323c, roughness: 0.95 }));
  ground.rotation.x = -Math.PI / 2;
  ground.position.set(W / 2, 0, -D / 2);
  ground.receiveShadow = true;
  ground.name = "ground";
  mapGroup.add(ground);
  groundMesh = ground;

  // outer floor
  const outer = new THREE.Mesh(new THREE.PlaneGeometry(W + 60, D + 60),
    new THREE.MeshStandardMaterial({ color: 0x14181e, roughness: 1 }));
  outer.rotation.x = -Math.PI / 2;
  outer.position.set(W / 2, -0.002, -D / 2);
  outer.receiveShadow = true;
  mapGroup.add(outer);

  // grid: 0.5 m lines and 1 m major lines
  const pts = [], ptsMajor = [];
  for (let x = 0; x <= W + 1e-6; x += CELL) (Math.abs(x % 1) < 1e-6 ? ptsMajor : pts).push(x, 0.002, 0, x, 0.002, -D);
  for (let y = 0; y <= D + 1e-6; y += CELL) (Math.abs(y % 1) < 1e-6 ? ptsMajor : pts).push(0, 0.002, -y, W, 0.002, -y);
  const mkLines = (arr, color, op) => {
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.Float32BufferAttribute(arr, 3));
    return new THREE.LineSegments(g, new THREE.LineBasicMaterial({ color, transparent: true, opacity: op }));
  };
  mapGroup.add(mkLines(pts, 0x55606e, 0.35), mkLines(ptsMajor, 0x7a8696, 0.6));

  // X axis (red), Y axis (green)
  const axis = (dx, dy, color) => {
    const a = new THREE.ArrowHelper(S2T(dx, dy).normalize(), new THREE.Vector3(0, 0.01, 0), 1, color, 0.18, 0.1);
    mapGroup.add(editorOnly(a));
  };
  axis(1, 0, 0xff5555); axis(0, 1, 0x55ff77);

  // field border
  const rimMat = new THREE.MeshStandardMaterial({ color: 0xff8a00, roughness: 0.6 });
  [[W / 2, -0.03, W + 0.06, 0.06], [W / 2, D + 0.03, W + 0.06, 0.06], [-0.03, D / 2, 0.06, D], [W + 0.03, D / 2, 0.06, D]].forEach(([x, y, w, d]) => {
    const r = new THREE.Mesh(new THREE.BoxGeometry(w, 0.08, d), rimMat);
    r.position.copy(S2T(x, y, 0.04));
    mapGroup.add(r);
  });

  // boxes
  for (const b of m.boxes) {
    const mat = new THREE.MeshStandardMaterial({ color: b.color || 0x8d99ae, roughness: 0.75 });
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(b.w * 0.98, b.h, b.d * 0.98), mat);
    mesh.position.copy(S2T(b.x, b.y, b.h / 2));
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    mesh.userData.boxCell = { x: b.x, y: b.y };
    boxMeshes.push(mesh);
    mapGroup.add(mesh);
    const edges = new THREE.LineSegments(new THREE.EdgesGeometry(mesh.geometry),
      new THREE.LineBasicMaterial({ color: 0x000000, transparent: true, opacity: 0.25 }));
    edges.position.copy(mesh.position);
    mapGroup.add(edges);
  }

  // goal flags
  m.goals.forEach((g, i) => {
    const grp = new THREE.Group();
    const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.015, 0.015, 0.7), new THREE.MeshStandardMaterial({ color: 0xdddddd }));
    pole.position.y = 0.35;
    pole.castShadow = true;
    const shape = new THREE.Shape();
    shape.moveTo(0, 0); shape.lineTo(0.28, -0.09); shape.lineTo(0, -0.18);
    const flagMat = new THREE.MeshStandardMaterial({ color: 0xff3b3b, side: THREE.DoubleSide, emissive: 0x330000 });
    const flag = new THREE.Mesh(new THREE.ShapeGeometry(shape), flagMat);
    flag.position.set(0.015, 0.7, 0);
    const ring = new THREE.Mesh(new THREE.RingGeometry(0.3, 0.42, 40),
      new THREE.MeshBasicMaterial({ color: 0xff3b3b, transparent: true, opacity: 0.35, side: THREE.DoubleSide }));
    ring.rotation.x = -Math.PI / 2;
    ring.position.y = 0.004;
    grp.add(pole, flag, ring);
    grp.position.copy(S2T(g.x, g.y, 0));
    grp.userData = { flagMat, ring, flag, idx: i };
    mapGroup.add(grp);
    goalMeshes.push(grp);
  });

  // start marker (arrow on the ground)
  const st = m.start;
  const arrowShape = new THREE.Shape();
  arrowShape.moveTo(0.3, 0); arrowShape.lineTo(-0.2, 0.2); arrowShape.lineTo(-0.1, 0); arrowShape.lineTo(-0.2, -0.2);
  const arrow = new THREE.Mesh(new THREE.ShapeGeometry(arrowShape),
    new THREE.MeshBasicMaterial({ color: 0x00d0ff, transparent: true, opacity: 0.4, side: THREE.DoubleSide }));
  arrow.rotation.x = -Math.PI / 2;
  const ag = new THREE.Group();
  ag.add(arrow);
  ag.position.copy(S2T(st.x, st.y, 0.005));
  ag.rotation.y = THREE.MathUtils.degToRad(st.yaw || 0);
  mapGroup.add(editorOnly(ag));

  // fit light and shadows to the field size
  const c = S2T(W / 2, D / 2);
  sun.position.set(c.x + W * 0.4, Math.max(W, D) * 1.2, c.z + D * 0.6);
  sun.target.position.copy(c);
  const s = Math.max(W, D) * 0.8;
  Object.assign(sun.shadow.camera, { left: -s, right: s, top: s, bottom: -s, near: 0.5, far: Math.max(W, D) * 4 });
  sun.shadow.camera.updateProjectionMatrix();

  $("mapW").value = W; $("mapD").value = D;
  $("mapInfo").textContent = `"${m.name || "Untitled"}" · ${W}×${D} m · ${m.boxes.length} boxes · ${m.goals.length} flags`;
  lastReached = "";
}

function resetCamera() {
  if (!MAP) return;
  const [W, D] = MAP.size;
  const c = S2T(W / 2, D / 2);
  controls.target.copy(c);
  camera.position.set(c.x - W * 0.15, Math.max(W, D) * 0.85, c.z + D * 0.95);
  controls.update();
}
function topCamera() {
  if (!MAP) return;
  const [W, D] = MAP.size;
  const c = S2T(W / 2, D / 2);
  controls.target.copy(c);
  camera.position.set(c.x, Math.max(W, D) * 1.25, c.z + 0.01);
  controls.update();
}

// ============================================================================
// Robot model (quadruped, point feet)
// ============================================================================
const L1 = 0.21, L2 = 0.21;
const HIP_X = 0.22, HIP_Z = 0.13, HIP_V = -0.02;
const robot = new THREE.Group();
const torso = new THREE.Group();
robot.add(torso);
scene.add(robot);

const matShell = new THREE.MeshStandardMaterial({ color: 0xdfe3e8, roughness: 0.45, metalness: 0.1 });
const matDark = new THREE.MeshStandardMaterial({ color: 0x23272e, roughness: 0.6, metalness: 0.3 });
const matAccent = new THREE.MeshStandardMaterial({ color: 0xff8a00, roughness: 0.5 });
const matLed = new THREE.MeshStandardMaterial({ color: 0x00d0ff, emissive: 0x00d0ff, emissiveIntensity: 1.6 });

function addMesh(parent, geo, mat, x = 0, y = 0, z = 0) {
  const m = new THREE.Mesh(geo, mat);
  m.position.set(x, y, z);
  m.castShadow = true;
  parent.add(m);
  return m;
}
// body
addMesh(torso, new THREE.BoxGeometry(0.56, 0.12, 0.22), matShell);
addMesh(torso, new THREE.BoxGeometry(0.5, 0.03, 0.18), matDark, -0.01, 0.075, 0);
addMesh(torso, new THREE.BoxGeometry(0.58, 0.035, 0.228), matDark, 0, -0.06, 0);
addMesh(torso, new THREE.BoxGeometry(0.3, 0.015, 0.232), matAccent, 0.03, 0.02, 0);
// LiDAR / handle
addMesh(torso, new THREE.CylinderGeometry(0.045, 0.05, 0.05, 24), matDark, -0.14, 0.115, 0);
addMesh(torso, new THREE.CylinderGeometry(0.035, 0.035, 0.012, 24), matLed, -0.14, 0.145, 0);
// head
const head = new THREE.Group();
head.position.set(0.28, 0.0, 0);
torso.add(head);
addMesh(head, new THREE.BoxGeometry(0.1, 0.1, 0.18), matDark, 0.04, 0, 0);
addMesh(head, new THREE.BoxGeometry(0.01, 0.025, 0.05), matLed, 0.092, 0.012, 0.045);
addMesh(head, new THREE.BoxGeometry(0.01, 0.025, 0.05), matLed, 0.092, 0.012, -0.045);
addMesh(head, new THREE.BoxGeometry(0.012, 0.03, 0.03), matDark, 0.093, -0.025, 0);   // camera

// legs: FL, FR, RL, RR  (robot's left = sim +y = three -z)
const LEGS = [
  { name: "FL", x: HIP_X, y: HIP_Z },
  { name: "FR", x: HIP_X, y: -HIP_Z },
  { name: "RL", x: -HIP_X, y: HIP_Z },
  { name: "RR", x: -HIP_X, y: -HIP_Z },
];
for (const leg of LEGS) {
  const hip = new THREE.Group();
  hip.position.set(leg.x, HIP_V, -leg.y);
  torso.add(hip);
  const motor = addMesh(hip, new THREE.CylinderGeometry(0.045, 0.045, 0.06, 20), matDark, 0, 0, 0);
  motor.rotation.x = Math.PI / 2;
  const thigh = new THREE.Group();
  thigh.position.z = -Math.sign(leg.y) * 0.05;
  hip.add(thigh);
  addMesh(thigh, new THREE.BoxGeometry(0.05, L1, 0.04), matShell, 0, -L1 / 2, 0);
  const ledMat = matAccent.clone();      // light strip of each leg (color set by the SDK)
  leg.ledMat = ledMat;
  addMesh(thigh, new THREE.BoxGeometry(0.052, L1 * 0.5, 0.042), ledMat, 0, -L1 * 0.35, 0);
  const knee = new THREE.Group();
  knee.position.y = -L1;
  thigh.add(knee);
  const kj = addMesh(knee, new THREE.CylinderGeometry(0.025, 0.025, 0.045, 16), matDark);
  kj.rotation.x = Math.PI / 2;
  addMesh(knee, new THREE.BoxGeometry(0.03, L2, 0.03), matDark, 0, -L2 / 2, 0);
  addMesh(knee, new THREE.SphereGeometry(0.024, 16, 12), matDark, 0, -L2, 0);
  Object.assign(leg, { hip, thigh, knee });
}

function legIK(leg, fx, fy, fz) {
  // fz: sideways offset (three z) from the hip, fy < 0 = down
  const r = -Math.atan2(fz, -fy);
  const h = Math.hypot(fy, fz);
  fy = -h;
  const d = THREE.MathUtils.clamp(Math.hypot(fx, fy), 0.06, L1 + L2 - 0.005);
  const a = Math.atan2(fx, -fy);
  const alpha = Math.acos(THREE.MathUtils.clamp((L1 * L1 + d * d - L2 * L2) / (2 * L1 * d), -1, 1));
  const t1 = a - alpha;                    // knees point backward
  const kx = L1 * Math.sin(t1), ky = -L1 * Math.cos(t1);
  const ex = (fx / Math.hypot(fx, fy)) * d, ey = (fy / Math.hypot(fx, fy)) * d;
  const t2 = Math.atan2(ex - kx, -(ey - ky));
  leg.hip.rotation.x = r;
  leg.thigh.rotation.z = t1;
  leg.knee.rotation.z = t2 - t1;
}

const GAIT = {
  walk: { off: [0, 0.5, 0.75, 0.25], duty: 0.72, freq: 1.6, lift: 0.05 },
  trot: { off: [0, 0.5, 0.5, 0], duty: 0.5, freq: 2.4, lift: 0.065 },
  run:  { off: [0, 0.1, 0.5, 0.6], duty: 0.38, freq: 3.2, lift: 0.08 },
};

// displayed state (smoothed with lerp)
// base = height of the body's ground reference, tp/tr = pitch/roll caused by uneven ground (stairs)
const disp = { x: 1, y: 1, z: 0, base: 0, tp: 0, tr: 0, yaw: 0, H: 0.28, pitch: 0, vx: 0, vy: 0, wz: 0, phase: 0, amp: 0 };
let RS = null;   // latest robot state from the server
const STEP_LIMIT = 0.15;

// Height of the walkable surface under one point (same rule as the server: boxes reachable from zRef)
function groundAt(x, y, zRef) {
  let z = 0;
  if (!MAP) return 0;
  for (const b of MAP.boxes) {
    if (b.h - zRef > STEP_LIMIT + 1e-6) continue;
    if (Math.abs(x - b.x) <= b.w / 2 + 1e-6 && Math.abs(y - b.y) <= b.d / 2 + 1e-6) z = Math.max(z, b.h);
  }
  return z;
}
let speedFactor = 1;
let actionStart = 0, lastActionId = -1;

function lerpAngle(a, b, t) {
  let d = ((b - a + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI;
  return a + d * t;
}

function updateRobot(dt, now) {
  if (!RS) return;
  const k = 1 - Math.exp(-dt * 14);
  disp.x += (RS.x - disp.x) * k;
  disp.y += (RS.y - disp.y) * k;
  disp.yaw = lerpAngle(disp.yaw, RS.yaw, k);

  // ground under each foot -> body height and tilt follow the stairs (feet never sink into a step)
  const cy = Math.cos(disp.yaw), sy = Math.sin(disp.yaw);
  const footGround = LEGS.map((leg) =>
    groundAt(disp.x + cy * leg.x - sy * leg.y, disp.y + sy * leg.x + cy * leg.y, RS.z));
  const [gFL, gFR, gRL, gRR] = footGround;
  const base = (gFL + gFR + gRL + gRR) / 4;
  const clampT = (v) => THREE.MathUtils.clamp(v, -0.4, 0.4);
  const tPitch = clampT(Math.atan2((gFL + gFR) / 2 - (gRL + gRR) / 2, 2 * HIP_X));
  const tRollG = clampT(Math.atan2((gFL + gRL) / 2 - (gFR + gRR) / 2, 2 * HIP_Z));
  const kg = 1 - Math.exp(-dt * 10);
  disp.base += (base - disp.base) * kg;
  disp.tp += (tPitch - disp.tp) * kg;
  disp.tr += (tRollG - disp.tr) * kg;
  disp.z = disp.base;
  disp.vx += (RS.vx - disp.vx) * k;
  disp.vy += (RS.vy - disp.vy) * k;
  disp.wz += (RS.wz - disp.wz) * k;

  const g = GAIT[RS.gait] || GAIT.trot;
  const moving = Math.hypot(disp.vx, disp.vy) + Math.abs(disp.wz) * 0.25 > 0.02;
  disp.amp += ((moving ? 1 : 0) - disp.amp) * (1 - Math.exp(-dt * 8));
  if (disp.amp > 0.02) disp.phase = (disp.phase + g.freq * dt * speedFactor) % 1;

  // posture
  let tH = RS.body_height, tP = 0, tRoll = RS.roll || 0, torsoYaw = 0, headNod = 0;
  if (RS.posture === "lie") { tH = 0.11; tP = 0; }
  const act = RS.action;
  if (RS.action_id !== lastActionId) { lastActionId = RS.action_id; actionStart = now; }
  const at = ((now - actionStart) / 1000) * speedFactor;

  // ---- motions of the real SDK (dance, jump, backflip, wave, ...) ----
  let air = 0, flip = 0, spin = 0, tuck = false, wave2 = false;
  const S = Math.sin;
  if (act === "wave") { tH = 0.22; tP = 0.38; }
  if (act === "gongxi") { tH = 0.22; tP = 0.45 + 0.08 * S(at * 6); wave2 = true; }
  if (act === "dance") { tRoll = 0.15 * S(at * 6); torsoYaw = 0.15 * S(at * 3); tH += 0.035 * S(at * 12); tP = 0.08 * S(at * 6 + 1); }
  if (act === "groove") { tH += 0.04 * S(at * 8); tRoll = 0.08 * S(at * 4); }
  if (act === "bounce") { tH += 0.05 * S(at * 10); headNod = 0.3 * S(at * 10); }
  if (act === "body_wave") { tP = 0.2 * S(at * 5); tH += 0.03 * S(at * 5 + 1); }
  if (act === "hip_circle") { tRoll = 0.12 * S(at * 3); tP = 0.1 * Math.cos(at * 3); }
  if (act === "head_circle") { torsoYaw = 0.15 * S(at * 3); tP = 0.1 * Math.cos(at * 3); }
  if (act === "diag_step") { tRoll = 0.1 * S(at * 4); torsoYaw = 0.08 * S(at * 4); }
  if (act === "hop_step") { tP = 0.18 * Math.max(0, S(at * 8)); }
  if (act === "recovery") { if (at < 1.2) { tH = 0.11; tRoll = 0.3 * (1 - at / 1.2); } }
  const jumpArc = (t0, t1, h) => (at > t0 && at < t1 ? h * S(Math.PI * (at - t0) / (t1 - t0)) : 0);
  if (act === "jump") { if (at < 0.5) tH = 0.2; air = jumpArc(0.5, 1.1, 0.3); tuck = air > 0.02; }
  if (act === "twirl_jump") { if (at < 0.3) tH = 0.2; air = jumpArc(0.3, 0.9, 0.3); tuck = air > 0.02; spin = at > 0.3 ? Math.min(1, (at - 0.3) / 0.6) * Math.PI * 2 : 0; }
  if (act === "backflip") {
    if (at < 0.5) { tH = 0.19; tP = 0.2; }
    air = jumpArc(0.5, 1.5, 0.6); tuck = air > 0.02;
    flip = at > 0.5 ? Math.min(1, (at - 0.5) / 1.0) * Math.PI * 2 : 0;
  }
  if (act === "dance" || act === "jump" || act === "backflip" || act === "twirl_jump") disp.amp *= 0.9;

  // balance_* from the SDK: bal = [roll°, pitch°, yaw°, height m]
  const bal = RS.bal || [0, 0, 0, 0];
  const D2R = Math.PI / 180;
  tH += bal[3];
  const kp = 1 - Math.exp(-dt * 6);
  disp.H += (tH - disp.H) * kp;
  disp.pitch += (tP - disp.pitch) * kp;

  robot.position.copy(S2T(disp.x, disp.y, disp.base));
  robot.position.y += air;
  robot.rotation.y = disp.yaw + spin;
  torso.position.y = disp.H;
  // SDK convention: roll>0 = lean left, pitch>0 = lean forward, yaw>0 = look right
  torso.rotation.set(tRoll + disp.tr - bal[0] * D2R, torsoYaw - bal[2] * D2R,
    disp.pitch + disp.tp + (RS.pitch || 0) - bal[1] * D2R + flip, "YZX");
  head.rotation.z = headNod;

  const p = disp.pitch + disp.tp, cp = Math.cos(p), sp = Math.sin(p);
  LEGS.forEach((leg, i) => {
    // velocity of the hip point (body frame)
    const lvx = disp.vx - disp.wz * leg.y;
    const lvy = disp.vy + disp.wz * leg.x;
    const stance = g.duty / g.freq;
    const Sx = lvx * stance * disp.amp, Sy = lvy * stance * disp.amp;
    const ph = (disp.phase + g.off[i]) % 1;
    let ox, oy, lift = 0;
    if (ph < 1 - g.duty) {
      const s = ph / (1 - g.duty);
      const e = s * s * (3 - 2 * s);
      ox = -Sx / 2 + Sx * e; oy = -Sy / 2 + Sy * e;
      lift = Math.sin(Math.PI * s) * g.lift * disp.amp;
    } else {
      const s = (ph - (1 - g.duty)) / g.duty;
      ox = Sx / 2 - Sx * s; oy = Sy / 2 - Sy * s;
    }
    // hip position in the robot frame (after the body pitch)
    const hipX = leg.x * cp - HIP_V * sp;
    const hipY = disp.H + leg.x * sp + HIP_V * cp;
    if (tuck) { legIK(leg, 0.02, -0.17, 0); return; }        // in the air: tuck the legs
    // each foot stands on the ground under it (step height relative to the body base)
    let footX = leg.x + ox, footY = (footGround[i] - disp.base) + lift;
    const waving = act === "wave" && leg.name === "FR";
    if (waving || (wave2 && leg.x > 0)) {                     // raise a paw and wave
      footX = leg.x + 0.2; footY = disp.H + 0.02 + 0.06 * Math.sin(at * 9 + (leg.y > 0 ? 1.5 : 0));
    }
    if (RS.posture === "lie" && !act) footX = leg.x + (leg.x > 0 ? 0.12 : -0.12);
    const wx = footX - hipX, wy = footY - hipY;
    const fx = wx * cp + wy * sp;
    const fy = -wx * sp + wy * cp;
    const fz = -oy;    // sim +y = three -z
    legIK(leg, fx, fy, fz);
  });

  const c = new THREE.Color(RS.led);
  matLed.color.copy(c);
  matLed.emissive.copy(c);

  // leg lights (set_all_legs_color / set_leg_rgb ...)
  const key = RS.leg_led ? RS.leg_led.join() : "";
  if (key !== lastLegLed) {
    lastLegLed = key;
    LEGS.forEach((leg, i) => {
      if (RS.leg_led) {
        const lc = new THREE.Color(RS.leg_led[i]);
        leg.ledMat.color.copy(lc.r + lc.g + lc.b < 0.05 ? new THREE.Color(0x333333) : lc);
        leg.ledMat.emissive.copy(lc);
        leg.ledMat.emissiveIntensity = 1.8;
      } else {
        leg.ledMat.color.set(0xff8a00);
        leg.ledMat.emissive.set(0x000000);
      }
    });
  }
}
let lastLegLed = null;

// ============================================================================
// trail
// ============================================================================
const MAX_TRAIL = 4000;
const trailGeo = new THREE.BufferGeometry();
const trailPos = new Float32Array(MAX_TRAIL * 3);
trailGeo.setAttribute("position", new THREE.BufferAttribute(trailPos, 3));
trailGeo.setDrawRange(0, 0);
const trailLine = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({ color: 0x00d0ff, transparent: true, opacity: 0.7 }));
trailLine.frustumCulled = false;
scene.add(editorOnly(trailLine));
let trailCount = 0, trailId = -1;

function pushTrail(state) {
  if (state.trail_id !== trailId) { trailId = state.trail_id; trailCount = 0; }
  if (state.trail_from === 0) trailCount = 0;
  for (const [x, y] of state.trail) {
    if (trailCount >= MAX_TRAIL) break;
    trailPos[trailCount * 3] = x; trailPos[trailCount * 3 + 1] = 0.012 + (RS ? RS.z : 0); trailPos[trailCount * 3 + 2] = -y;
    trailCount++;
  }
  trailGeo.setDrawRange(0, trailCount);
  trailGeo.attributes.position.needsUpdate = true;
}


// ============================================================================
// Polling the state from the server
// ============================================================================
let logSeq = 0;
let lastReached = "";
let running = false, runningKey = "";
let curLine = null;
let myRunId = null;   // id of the program started from THIS window (another window may run its own)
let bootId = null;    // changes when the simulator (server) restarts
let online = null, failCount = 0;

function setOnline(ok) {
  if (ok === online) return;
  online = ok;
  $("offline").classList.toggle("hidden", ok);
  $("btnRun").disabled = !ok || running;
}

async function poll() {
  for (;;) {
    let delay = 45;
    try {
      const s = await api(`/api/state?since=${logSeq}&trail=${trailCount}&tid=${trailId}`);
      setOnline(true);
      failCount = 0;
      if (s.boot !== bootId) {
        if (bootId !== null) {
          // restarted simulator: logs, trail and map numbering start again from the beginning
          logSeq = 0; trailId = -1; trailCount = 0; mapVersion = -1; myRunId = null;
          logLine("↻ Reconnected to a restarted simulator", "sys");
          bootId = s.boot;
          continue;
        }
        bootId = s.boot;
      }
      await applyState(s);
    } catch (e) {
      failCount++;
      setOnline(false);
      delay = Math.min(2000, 200 * failCount);     // do not flood the console while it is closed
    }
    await new Promise((r) => setTimeout(r, delay));
  }
}

async function applyState(s) {
  if (s.map_version !== mapVersion && !mapBusy && !mapPending) {
    const m = await api("/api/map");
    mapVersion = m.version;
    const first = MAP === null;
    buildMap(m.map);
    if (first) resetCamera();
  }
  const firstRobot = RS === null;
  RS = s.robot;
  if (firstRobot) Object.assign(disp, { x: RS.x, y: RS.y, z: RS.z, base: RS.z, yaw: RS.yaw, H: RS.body_height });
  // teleport (reset) -> no lerp
  if (Math.hypot(RS.x - disp.x, RS.y - disp.y) > 1.5) Object.assign(disp, { x: RS.x, y: RS.y, yaw: RS.yaw });
  speedFactor = s.speed_factor;
  const sel = $("simSpeed");
  if (document.activeElement !== sel && +sel.value !== s.speed_factor &&
      [...sel.options].some((o) => +o.value === s.speed_factor)) sel.value = String(s.speed_factor);
  pushTrail(s);
  for (const [seq, kind, text] of s.logs) { logSeq = seq; logLine(text, kind); }
  const reachedKey = s.reached.join(",");
  if (reachedKey !== lastReached) {
    lastReached = reachedKey;
    goalMeshes.forEach((gm) => {
      const ok = s.reached.includes(gm.userData.idx);
      gm.userData.flagMat.color.set(ok ? 0x2bd46a : 0xff3b3b);
      gm.userData.flagMat.emissive.set(ok ? 0x053312 : 0x330000);
      gm.userData.ring.material.color.set(ok ? 0x2bd46a : 0xff3b3b);
    });
  }
  showCams(s.cams);
  const mine = s.running && s.run_id === myRunId;
  setRunning(s.running, mine);
  setCurLine(mine ? s.line : null);   // never highlight a line of another window's program
  // HUD
  $("hPos").textContent = `(${RS.x.toFixed(2)}, ${RS.y.toFixed(2)}) m`;
  $("hYaw").textContent = `${RS.yaw_deg.toFixed(1)}°`;
  $("hGait").textContent = RS.gait;
  $("hPost").textContent = RS.action ? `${RS.posture} · ${RS.action}` : RS.posture;
  $("hSdk").textContent = `${RS.fsm} · ${RS.speed_ratio}%${RS.avoid ? " · 🛡" : ""}`;
  $("hVel").textContent = `${Math.hypot(RS.vx, RS.vy).toFixed(2)} m/s  ${(RS.wz * 57.3).toFixed(0)}°/s`;
  $("hOdo").textContent = `${RS.odometer.toFixed(2)} m`;
  $("hBat").textContent = `${RS.battery.toFixed(1)} %`;
  $("hGoal").textContent = MAP ? `${s.reached.length} / ${MAP.goals.length}` : "-";
  $("hTime").textContent = `${s.sim_time.toFixed(1)} s`;
}

function setRunning(r, mine = true) {
  const key = `${r}|${mine}`;
  if (key === runningKey) return;
  runningKey = key;
  running = r;
  const b = $("runBadge");
  b.textContent = r ? (mine ? "RUNNING" : "RUNNING · OTHER WINDOW") : "IDLE";
  b.title = r && !mine ? "A program started from another browser window is controlling the robot" : "";
  b.className = "badge " + (r ? "run" : "idle");
  $("btnRun").disabled = r || online === false;
}

// ============================================================================
// Robot cameras (robot.video). While a program has a camera open, this page renders what the
// front / rear RGB camera sees and uploads JPEG frames; the simulator streams them to the
// program (cv2.VideoCapture) the way the real robot streams RTSP.
// ============================================================================
const CAM_W = 640, CAM_H = 360, CAM_FPS = 12;
const CAM_VFOV = 2 * Math.atan(Math.tan((87 / 2) * Math.PI / 180) * CAM_H / CAM_W) * 180 / Math.PI;  // 87° wide
const robotCams = [0, 1].map((id) => {                 // index = CameraId value (0 front, 1 rear)
  const c = new THREE.PerspectiveCamera(CAM_VFOV, CAM_W / CAM_H, 0.03, 60);
  if (id === 0) { c.position.set(0.40, -0.02, 0); c.rotation.y = -Math.PI / 2; }   // looks forward (+x)
  else { c.position.set(-0.32, -0.02, 0); c.rotation.y = Math.PI / 2; }           // looks backward
  torso.add(c);
  return c;
});
let wantedCams = [];
let camRenderer = null, camBusy = false, camLast = 0;

function camTick(now) {
  if (!wantedCams.length || camBusy || now - camLast < 1000 / CAM_FPS) return;
  camLast = now;
  if (!camRenderer) {
    camRenderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
    camRenderer.setPixelRatio(1);
    camRenderer.setSize(CAM_W, CAM_H, false);
    camRenderer.shadowMap.enabled = true;
    camRenderer.shadowMap.type = THREE.PCFSoftShadowMap;
    camRenderer.outputColorSpace = THREE.SRGBColorSpace;
  }
  camBusy = true;
  (async () => {
    try {
      for (const id of wantedCams) {
        if (!robotCams[id]) continue;
        camRenderer.render(scene, robotCams[id]);      // layer 0 only: no editor helpers
        const blob = await new Promise((r) => camRenderer.domElement.toBlob(r, "image/jpeg", 0.8));
        if (blob) await fetch(`/api/frame?cam=${id}`, { method: "POST", headers: { "Content-Type": "image/jpeg" }, body: blob });
      }
    } catch (e) {
      // the simulator may be restarting - the next tick tries again
    }
    camBusy = false;
  })();
}

function showCams(list) {
  wantedCams = list || [];
  const b = $("camBadge");
  const names = wantedCams.map((i) => (i === 0 ? "FRONT" : "REAR")).join("+");
  b.textContent = `📷 ${names}`;
  b.classList.toggle("hidden", !wantedCams.length);
}

// ============================================================================
// Render loop
// ============================================================================
let follow = false, showTrail = true;
let lastT = performance.now();
const tmpV = new THREE.Vector3();
function frame(now) {
  const dt = Math.min(0.05, (now - lastT) / 1000);
  lastT = now;
  updateRobot(dt, now);
  if (follow && RS) {
    tmpV.copy(robot.position);
    tmpV.y += 0.2;
    const delta = tmpV.clone().sub(controls.target).multiplyScalar(1 - Math.exp(-dt * 4));
    controls.target.add(delta);
    camera.position.add(delta);
  }
  goalMeshes.forEach((gm) => { gm.userData.flag.rotation.y = Math.sin(now / 400 + gm.userData.idx) * 0.25; });
  trailLine.visible = showTrail;


  controls.update();
  renderer.render(scene, camera);
  camTick(now);
  requestAnimationFrame(frame);
}

// ============================================================================
// Map editor
// ============================================================================
const TOOLS = {
  none: { label: "View only" },
  wall: { label: "Wall 0.6 m", h: 0.6, color: "#8d99ae" },
  tall: { label: "Tall wall 1.2 m", h: 1.2, color: "#6c7a8f" },
  block: { label: "Box 0.3 m", h: 0.3, color: "#e07a5f" },
  step: { label: "Step 0.1 m", h: 0.1, color: "#f2cc8f" },
  step2: { label: "Step 0.14 m", h: 0.14, color: "#e9b872" },
  goal: { label: "Goal flag" },
  start: { label: "Start point" },
  erase: { label: "Eraser" },
};
let tool = "none";
const ghost = new THREE.Mesh(new THREE.BoxGeometry(CELL, 1, CELL),
  new THREE.MeshBasicMaterial({ color: 0xff8a00, transparent: true, opacity: 0.35, depthWrite: false }));
ghost.visible = false;
scene.add(editorOnly(ghost));
const raycaster = new THREE.Raycaster();
const mouse = new THREE.Vector2();

function setTool(t) {
  tool = t;
  document.querySelectorAll(".tool").forEach((b) => b.classList.toggle("active", b.dataset.tool === t));
  $("editBar").classList.toggle("hidden", t === "none");
  $("editToolName").textContent = TOOLS[t].label;
  ghost.visible = false;
}

function cellAt(ev) {
  if (!groundMesh) return null;
  const rect = renderer.domElement.getBoundingClientRect();
  mouse.set(((ev.clientX - rect.left) / rect.width) * 2 - 1, -((ev.clientY - rect.top) / rect.height) * 2 + 1);
  raycaster.setFromCamera(mouse, camera);
  const hit = raycaster.intersectObjects([groundMesh, ...boxMeshes], false)[0];
  if (!hit) return null;
  const cellOf = hit.object.userData.boxCell;        // a box under the mouse: use its own cell
  const sx = cellOf ? cellOf.x : hit.point.x, sy = cellOf ? cellOf.y : -hit.point.z;
  const [W, D] = MAP.size;
  if (sx < 0 || sy < 0 || sx >= W || sy >= D) return null;
  const i = Math.floor(sx / CELL), j = Math.floor(sy / CELL);
  return { i, j, x: i * CELL + CELL / 2, y: j * CELL + CELL / 2 };
}

const near = (a, b) => Math.abs(a - b) < 0.01;
function applyTool(c) {
  const m = structuredClone(MAP);
  const t = TOOLS[tool];
  const bi = m.boxes.findIndex((b) => near(b.x, c.x) && near(b.y, c.y));
  const gi = m.goals.findIndex((g) => Math.hypot(g.x - c.x, g.y - c.y) < 0.3);
  const onStart = near(m.start.x, c.x) && near(m.start.y, c.y);
  if (t.h > STEP_LIMIT && onStart) { logLine("⚠ The start point is on this cell - move the start first", "warn"); return false; }
  if (tool === "start" && bi >= 0 && m.boxes[bi].h > STEP_LIMIT && !onStart) {
    logLine("⚠ The start point cannot be on an obstacle (the robot could not stand there)", "warn");
    return false;
  }
  if (t.h) {
    const nb = { x: c.x, y: c.y, w: CELL, d: CELL, h: t.h, color: t.color };
    if (bi >= 0) { if (m.boxes[bi].h === t.h) return false; m.boxes[bi] = nb; } else m.boxes.push(nb);
  } else if (tool === "goal") {
    if (gi >= 0) m.goals.splice(gi, 1); else m.goals.push({ x: c.x, y: c.y });
  } else if (tool === "start") {
    if (near(m.start.x, c.x) && near(m.start.y, c.y)) m.start.yaw = ((m.start.yaw || 0) + 90) % 360;
    else m.start = { x: c.x, y: c.y, yaw: m.start.yaw || 0 };
  } else if (tool === "erase") {
    if (bi < 0 && gi < 0) return false;
    if (bi >= 0) m.boxes.splice(bi, 1);
    if (gi >= 0) m.goals.splice(gi, 1);
  } else return false;
  sendMap(m, tool === "start");
  return true;
}

// Map saves go out one at a time and only the newest one matters, so fast painting with
// Shift+drag can never be overwritten by an older map arriving late.
let mapBusy = false, mapPending = null;
function sendMap(m, reset = false) {
  buildMap(m);                                         // show the change at once
  mapPending = { map: m, reset: reset || (mapPending ? mapPending.reset : false) };
  if (!mapBusy) flushMap();
}
async function flushMap() {
  mapBusy = true;
  try {
    while (mapPending) {
      const job = mapPending;
      mapPending = null;
      const r = await api("/api/map", { map: job.map, reset: job.reset || !running });
      if (!r.ok) { logLine("✖ Map not saved: " + (r.error || "error"), "error"); continue; }
      mapVersion = r.version;
      if (!mapPending) buildMap(r.map);                // draw exactly the map the simulator uses
    }
  } catch (e) {
    logLine("✖ Map not saved - is the simulator running?", "error");
  } finally {
    mapBusy = false;
  }
}

let downPos = null, lastCellKey = "";
renderer.domElement.addEventListener("pointerdown", (ev) => {
  downPos = { x: ev.clientX, y: ev.clientY };
  if (ev.shiftKey && tool !== "none") {
    controls.enabled = false;
    const c = cellAt(ev);
    if (c) { applyTool(c); lastCellKey = `${c.i},${c.j}`; }
  }
});
renderer.domElement.addEventListener("pointermove", (ev) => {
  if (tool === "none") return;
  const c = cellAt(ev);
  if (!c) { ghost.visible = false; return; }
  const t = TOOLS[tool];
  const h = t.h || 0.05;
  ghost.scale.y = h;
  ghost.position.copy(S2T(c.x, c.y, h / 2));
  ghost.material.color.set(tool === "erase" ? 0xff4d4d : tool === "goal" ? 0xff3b3b : tool === "start" ? 0x00d0ff : 0xff8a00);
  ghost.visible = true;
  if (ev.shiftKey && ev.buttons === 1 && downPos) {
    const key = `${c.i},${c.j}`;
    if (key !== lastCellKey) { lastCellKey = key; applyTool(c); }
  }
});
renderer.domElement.addEventListener("pointerup", (ev) => {
  controls.enabled = true;
  if (!downPos || tool === "none") { downPos = null; return; }
  const moved = Math.hypot(ev.clientX - downPos.x, ev.clientY - downPos.y);
  if (moved < 5 && !ev.shiftKey) { const c = cellAt(ev); if (c) applyTool(c); }
  downPos = null; lastCellKey = "";
});
renderer.domElement.addEventListener("pointerleave", () => { ghost.visible = false; });
window.addEventListener("pointerup", () => { controls.enabled = true; downPos = null; lastCellKey = ""; });
window.addEventListener("pointercancel", () => { controls.enabled = true; downPos = null; });

document.querySelectorAll(".tool").forEach((b) => b.addEventListener("click", () => setTool(b.dataset.tool)));
for (const [name, fn] of Object.entries(window.PRESETS)) {
  const b = document.createElement("button");
  b.textContent = name;
  b.onclick = () => { sendMap(fn(), true); setTimeout(resetCamera, 50); };
  $("presets").appendChild(b);
}
$("btnResize").onclick = () => {
  const W = Math.max(4, Math.min(40, Math.round(+$("mapW").value))), D = Math.max(4, Math.min(40, Math.round(+$("mapD").value)));
  const m = structuredClone(MAP);
  m.size = [W, D];
  m.boxes = m.boxes.filter((b) => b.x < W && b.y < D);
  m.goals = m.goals.filter((g) => g.x < W && g.y < D);
  if (m.start.x > W - 0.3 || m.start.y > D - 0.3) m.start = { x: 1.25, y: 1.25, yaw: 0 };
  sendMap(m, true);
  setTimeout(resetCamera, 50);
};
$("btnClearMap").onclick = () => {
  if (!confirm("Remove all obstacles and flags?")) return;
  sendMap({ ...structuredClone(MAP), name: "New map", boxes: [], goals: [] }, true);
};
$("btnSaveMap").onclick = () => {
  const blob = new Blob([JSON.stringify(MAP, null, 1)], { type: "application/json" });
  const a = document.createElement("a");
  a.href = URL.createObjectURL(blob);
  a.download = (MAP.name || "rover_map").replace(/\s+/g, "_") + ".json";
  a.click();
  URL.revokeObjectURL(a.href);
};
$("fileMap").onchange = async (ev) => {
  const f = ev.target.files[0];
  if (!f) return;
  try {
    const m = JSON.parse(await f.text());
    if (!m.size || !m.boxes) throw new Error("not a map file");
    m.name = m.name || f.name.replace(/\.json$/, "");
    sendMap(m, true);
    setTimeout(resetCamera, 50);
  } catch (e) { alert("Cannot open the file: " + e.message); }
  ev.target.value = "";
};

// ============================================================================
// Code editor
// ============================================================================
let SDK_DOCS = [], CONSTS = {};
const ROBOT_RE = /\b([A-Za-z_]\w*)\.(\w*)$/;
function hintSource(varName) {
  if (CONSTS[varName]) return CONSTS[varName].map((n) => ({ name: n, sig: "", plain: true }));
  const code = cm.getValue();
  const sdkVars = new Set(["robot"]);
  for (const m of code.matchAll(/^\s*(\w+)\s*=\s*(?:\w+\.)?RobotClient\s*\(/gm)) sdkVars.add(m[1]);
  for (const m of code.matchAll(/RobotClient\s*\(.*\)\s*as\s+(\w+)/g)) sdkVars.add(m[1]);
  if (sdkVars.has(varName)) return SDK_DOCS;
  return null;
}
const cm = CodeMirror.fromTextArea($("code"), {
  mode: "python", theme: "material-darker", lineNumbers: true, indentUnit: 4, tabSize: 4,
  matchBrackets: true, autoCloseBrackets: true, styleActiveLine: true,
  extraKeys: {
    "Ctrl-Enter": () => run(), "Cmd-Enter": () => run(),
    "Ctrl-Space": (c) => c.showHint({ hint: roverHint, completeSingle: false }),
    Tab: (c) => (c.somethingSelected() ? c.indentSelection("add") : c.replaceSelection("    ", "end")),
    "Shift-Tab": (c) => c.indentSelection("subtract"),
  },
});
// start with the first example (code is remembered per browser)
const CODE_KEY = "rover_code_sdk";
cm.setValue(store.get(CODE_KEY, window.EXAMPLES[Object.keys(window.EXAMPLES)[0]]));

function roverHint(c) {
  const cur = c.getCursor();
  const line = c.getLine(cur.line).slice(0, cur.ch);
  const m = line.match(ROBOT_RE);
  const src = m ? hintSource(m[1]) : null;
  let word, list;
  if (m && src) {
    word = m[2];
    const w = word.toLowerCase();
    list = src.filter((d) => d.name.toLowerCase().startsWith(w));
    if (!list.length) list = src.filter((d) => d.name.toLowerCase().includes(w));
  } else {
    word = (line.match(/\w*$/) || [""])[0];
    const extra = ["robot", "RobotClient", "Color", "Leg", "from dobot_quad import RobotClient", "print", "range", "for", "while", "if", "else", "def", "True", "False", "math", "import time", "time.sleep()"];
    const pool = extra.map((n) => ({ name: n, sig: "", plain: true }));
    list = pool.filter((d) => word && d.name.startsWith(word));
  }
  return {
    list: list.map((d) => ({
      text: d.name,
      render(el) {
        el.textContent = d.name;
        if (d.sig) { const s = document.createElement("span"); s.className = "hint-sig"; s.textContent = d.sig; el.appendChild(s); }
        if (d.doc) el.title = d.doc;
      },
      hint(cmi, data) {
        const insert = d.plain ? d.name : d.name + "()";
        cmi.replaceRange(insert, data.from, data.to);
        if (!d.plain && d.sig !== "()") {
          const p = cmi.getCursor();
          cmi.setCursor({ line: p.line, ch: p.ch - 1 });
        }
      },
    })),
    from: CodeMirror.Pos(cur.line, cur.ch - word.length),
    to: cur,
  };
}
cm.on("inputRead", (c, change) => {
  const ch = change.text[0];
  if (ch === "." || /\w/.test(ch)) {
    const cur = c.getCursor();
    const line = c.getLine(cur.line).slice(0, cur.ch);
    const m = line.match(ROBOT_RE);
    if (m && hintSource(m[1])) c.showHint({ hint: roverHint, completeSingle: false });
  }
});

// ---------- live check (Auto-correct preview) ----------
let checkTimer = null, markedLines = [];
cm.on("change", () => {
  store.set(CODE_KEY, cm.getValue());
  clearTimeout(checkTimer);
  checkTimer = setTimeout(liveCheck, 900);
});
function clearMarks() {
  markedLines.forEach((h) => ["cm-fixed-line", "cm-error-line"].forEach((k) => cm.removeLineClass(h, "background", k)));
  markedLines = [];
}
function markLine(n, cls) {
  const h = cm.getLineHandle(n - 1);
  if (h) { cm.addLineClass(h, "background", cls); markedLines.push(h); }
}
async function liveCheck() {
  const code = cm.getValue();
  let r;
  try { r = await api("/api/check", { code }); } catch (e) { return; }
  if (cm.getValue() !== code) return;
  r.source = code;
  showFixes(r, false);
}
function esc(s) { return String(s).replace(/[&<>]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;" }[c])); }
function showFixes(r, applied) {
  clearMarks();
  const box = $("fixBox");
  const changes = r.changes || [];
  if (!changes.length && !r.error) { box.classList.add("hidden"); return; }
  const fixes = changes.filter((c) => !c.warning);
  const title = applied ? `🔧 Auto-correct fixed ${fixes.length} issue(s)` : `🔧 Found ${fixes.length} likely typo(s)`;
  let html = `<div class="head"><span>${title}</span>`;
  if (!applied && fixes.length) html += `<button class="tiny" id="btnApplyFix">Fix all</button>`;
  html += `</div>`;
  for (const c of changes) {
    markLine(c.line, c.warning ? "cm-error-line" : "cm-fixed-line");
    html += c.warning
      ? `<div class="fx warnline"><span class="ln">L${c.line}</span><span>⚠ <code>${esc(c.before)}</code> ${esc(c.reason)}</span></div>`
      : `<div class="fx"><span class="ln">L${c.line}</span><span>${esc(c.reason)}</span></div>`;
  }
  if (r.error) {
    markLine(r.error.line, "cm-error-line");
    html += `<div class="fx warnline"><span class="ln">L${r.error.line}</span><span>✖ SyntaxError: ${esc(r.error.msg)}</span></div>`;
  }
  box.innerHTML = html;
  box.classList.remove("hidden");
  const btn = $("btnApplyFix");
  if (btn) btn.onclick = () => {
    if (r.source !== undefined && cm.getValue() !== r.source) { liveCheck(); return; }   // code changed since
    replaceCode(r.code);
    showFixes({ ...r, changes: fixes }, true);
  };
}
function replaceCode(code) {
  if (code === cm.getValue()) return;
  const cur = cm.getCursor(), sc = cm.getScrollInfo();
  cm.setValue(code);
  cm.setCursor(cur);
  cm.scrollTo(sc.left, sc.top);
}

let activeLineHandle = null;
function setCurLine(n) {
  if (n === curLine) return;
  if (activeLineHandle) cm.removeLineClass(activeLineHandle, "background", "cm-active-line");
  activeLineHandle = null;
  curLine = n;
  if (n) { activeLineHandle = cm.getLineHandle(n - 1); if (activeLineHandle) cm.addLineClass(activeLineHandle, "background", "cm-active-line"); }
}

// ---------- console ----------
const con = $("console");
function logLine(text, kind) {
  const d = document.createElement("div");
  d.className = "l " + kind;
  d.textContent = text;
  const atBottom = con.scrollHeight - con.scrollTop - con.clientHeight < 40;
  con.appendChild(d);
  while (con.childNodes.length > 600) con.removeChild(con.firstChild);
  if (atBottom) con.scrollTop = con.scrollHeight;
}

// ---------- buttons ----------
async function run() {
  if (running) return;
  clearTimeout(checkTimer);
  let r;
  try {
    r = await api("/api/run", { code: cm.getValue(), autocorrect: $("optAuto").checked, reset: $("optReset").checked });
  } catch (e) {
    logLine("✖ Cannot reach the simulator - start RoverX1Sim.exe (or python server.py) and try again", "error");
    return;
  }
  if (!r.ok) { logLine(r.error || "Could not run", "error"); return; }
  myRunId = r.run_id;
  if ($("optAuto").checked) {
    replaceCode(r.code);
    showFixes({ ...r, changes: r.changes.filter((c) => !c.warning).concat(r.changes.filter((c) => c.warning)) }, true);
  }
  setRunning(true);
}
$("btnRun").onclick = run;
const quiet = (p) => p.catch(() => setOnline(false));
$("btnStop").onclick = () => quiet(api("/api/stop", {}));
$("btnReset").onclick = () => quiet(api("/api/reset", {}));
$("btnCheck").onclick = async () => {
  let r;
  try { r = await api("/api/check", { code: cm.getValue() }); } catch (e) { setOnline(false); return; }
  replaceCode(r.code);
  showFixes(r, true);
};
$("btnClear").onclick = () => { con.innerHTML = ""; quiet(api("/api/clearlog", {})); };
$("simSpeed").onchange = (e) => quiet(api("/api/speed", { speed: +e.target.value }));

const exSel = $("examples");
for (const name of Object.keys(window.EXAMPLES)) {
  const o = document.createElement("option");
  o.value = name; o.textContent = name;
  exSel.appendChild(o);
}
exSel.onchange = () => {
  if (!exSel.value) return;
  cm.setValue(window.EXAMPLES[exSel.value]);
  if (exSel.value.includes("Stairs")) sendMap(window.PRESETS["Stairs"](), true);
  if (exSel.value.includes("flags")) sendMap(window.PRESETS["Starter field"](), true);
  exSel.value = "";
};

// ---------- tabs ----------
document.querySelectorAll(".tab").forEach((t) => t.addEventListener("click", () => {
  document.querySelectorAll(".tab").forEach((x) => x.classList.toggle("active", x === t));
  document.querySelectorAll(".tabpane").forEach((p) => p.classList.toggle("active", p.id === "tab-" + t.dataset.tab));
  if (t.dataset.tab === "code") setTimeout(() => cm.refresh(), 0);
  if (t.dataset.tab !== "map") setTool("none");
}));

// ---------- camera buttons ----------
$("camFollow").onclick = (e) => { follow = !follow; e.currentTarget.classList.toggle("on", follow); };
$("camTop").onclick = () => { follow = false; $("camFollow").classList.remove("on"); topCamera(); };
$("camReset").onclick = () => { follow = false; $("camFollow").classList.remove("on"); resetCamera(); };
$("toggleTrail").onclick = (e) => { showTrail = !showTrail; e.currentTarget.classList.toggle("on", !showTrail); };

// message from the manual page: "Try this code"
window.addEventListener("message", (ev) => {
  if (ev.origin !== location.origin || !ev.data || ev.data.type !== "try-code") return;
  cm.setValue(ev.data.code);
  document.querySelector('.tab[data-tab="code"]').click();
});

// ============================================================================
// start
// ============================================================================
api("/api/docs").then((d) => { SDK_DOCS = d.sdk.concat([{ name: "video", sig: "", plain: true }]); CONSTS = d.consts; }).catch(() => {});
resize();
requestAnimationFrame(frame);
poll();
liveCheck();
