// The live 3D Go2.
//
// Builds a three.js scene from the two files make_model.py produced --
// go2_model.json (the link tree) and go2_meshes.bin (the geometry) -- and then
// drives it from the numbers in /data.
//
// HOW THE MODEL IS PUT TOGETHER
//
// The robot is a tree of rigid links. Each link sits at a fixed offset from its
// parent, and twelve of those offsets have a motor that rotates the link about
// a single axis. So each body becomes three nested three.js objects:
//
//     group   fixed offset+rotation from the parent (body_pos, body_quat)
//       pivot at the joint anchor; THIS is what we rotate by the motor angle
//         content   holds the meshes, and the child bodies hang off it
//
// The pivot/content pair exists because a joint can be anchored away from the
// body origin. content is offset by -anchor so the geometry stays put while
// the pivot spins around the right point.
//
// COORDINATES: MuJoCo and ROS are both Z-up; three.js defaults to Y-up. Rather
// than rotate the whole robot, we tell the camera that Z is up. Nothing in the
// model data is transformed, so a position printed here matches one printed by
// the robot.
//
// ANGLES: motor_state[i].q is radians about the joint's own axis, right-handed.
// It is applied directly, with no sign flips -- see the joint-order comment in
// make_model.py.

import * as THREE from "/static/vendor/three.module.js";
import { OrbitControls } from "/static/vendor/OrbitControls.js";

const MOTOR_COUNT = 12;

// Colour ramps for the joint heat-map. Blue is cold/unloaded, red is hot/loaded.
const RAMP = [
  [0.0, [0x3b, 0x82, 0xf6]],
  [0.5, [0x4a, 0xde, 0x80]],
  [0.75, [0xfb, 0xbf, 0x24]],
  [1.0, [0xf8, 0x71, 0x71]],
];

function ramp(t) {
  t = Math.max(0, Math.min(1, t));
  for (let i = 1; i < RAMP.length; i++) {
    if (t <= RAMP[i][0]) {
      const [a, ca] = RAMP[i - 1], [b, cb] = RAMP[i];
      const f = (t - a) / (b - a);
      return new THREE.Color(
        (ca[0] + (cb[0] - ca[0]) * f) / 255,
        (ca[1] + (cb[1] - ca[1]) * f) / 255,
        (ca[2] + (cb[2] - ca[2]) * f) / 255
      );
    }
  }
  return new THREE.Color(1, 0, 0);
}

// The scales the heat-map is normalised against. Motor temperature above 80 C
// is genuinely hot; tau_est on a Go2 leg peaks around 45 N*m at the calf.
const TEMP_RANGE = [20, 80];
const TAU_RANGE = [0, 25];

export class Robot3D {
  constructor(canvas, tooltip) {
    this.canvas = canvas;
    this.tooltip = tooltip;
    this.bodies = [];        // one entry per body in go2_model.json
    this.byMotor = {};       // motor index -> the body it drives
    this.footPads = [];
    this.colorMode = "temperature";
    this.ready = false;

    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));

    this.scene = new THREE.Scene();

    this.camera = new THREE.PerspectiveCamera(45, 1, 0.05, 50);
    this.camera.up.set(0, 0, 1);              // Z-up, see the note above
    this.camera.position.set(0.72, -0.72, 0.42);

    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.enableDamping = true;
    this.controls.target.set(0, 0, 0.22);

    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x334155, 2.2));
    const key = new THREE.DirectionalLight(0xffffff, 1.6);
    key.position.set(1.2, -1.0, 2.0);
    this.scene.add(key);
    const fill = new THREE.DirectionalLight(0xffffff, 0.5);
    fill.position.set(-1.5, 1.0, 0.6);
    this.scene.add(fill);

    const grid = new THREE.GridHelper(4, 20, 0x334155, 0x1f2733);
    grid.rotation.x = Math.PI / 2;            // GridHelper is XZ; we want XY
    this.scene.add(grid);

    // Everything below `root` is the robot. Rotating root is how the IMU
    // attitude gets applied, leaving the grid as the world horizon.
    this.root = new THREE.Group();
    this.scene.add(this.root);

    this.raycaster = new THREE.Raycaster();
    this.pointer = new THREE.Vector2();
    this.hover = null;
    canvas.addEventListener("pointermove", (e) => this._onPointerMove(e));
    canvas.addEventListener("pointerleave", () => {
      this.tooltip.style.display = "none";
      this.hover = null;
    });

    this._resize();
    new ResizeObserver(() => this._resize()).observe(canvas.parentElement);
    this._animate();
  }

  async load() {
    const [model, blob] = await Promise.all([
      fetch("/static/go2_model.json").then((r) => r.json()),
      fetch("/static/go2_meshes.bin").then((r) => r.arrayBuffer()),
    ]);
    this.model = model;

    // Slice the one binary blob into the three typed arrays it holds.
    const s = model.sections;
    const positions = new Float32Array(blob, s.positions.byteOffset,
      s.positions.byteLength / 4);
    const normals = new Int8Array(blob, s.normals.byteOffset, s.normals.byteLength);
    const indices = new Uint16Array(blob, s.indices.byteOffset,
      s.indices.byteLength / 2);

    // One BufferGeometry per unique mesh. Legs share geometry: the same hip
    // mesh is drawn four times, so it is uploaded to the GPU once.
    const geoms = {};
    for (const [name, m] of Object.entries(model.meshes)) {
      const g = new THREE.BufferGeometry();
      g.setAttribute("position", new THREE.BufferAttribute(
        positions.subarray(m.voff * 3, (m.voff + m.vcount) * 3), 3));
      // normalized=true turns the stored bytes back into unit vectors.
      g.setAttribute("normal", new THREE.BufferAttribute(
        normals.subarray(m.voff * 3, (m.voff + m.vcount) * 3), 3, true));
      g.setIndex(new THREE.BufferAttribute(
        indices.subarray(m.ioff, m.ioff + m.icount), 1));
      geoms[name] = g;
    }

    // Walk the body list. make_model.py emits parents before children, so a
    // parent's `content` group always exists by the time a child needs it.
    for (let i = 0; i < model.bodies.length; i++) {
      const b = model.bodies[i];

      const group = new THREE.Group();
      group.position.fromArray(b.pos);
      group.quaternion.set(b.quat[1], b.quat[2], b.quat[3], b.quat[0]); // wxyz -> xyzw

      const pivot = new THREE.Group();
      const content = new THREE.Group();
      pivot.add(content);
      group.add(pivot);

      const entry = {
        name: b.name, group, pivot, content,
        axis: null, meshes: [], baseColors: [], joint: b.joint,
      };

      if (b.joint) {
        const a = b.joint.pos;
        pivot.position.set(a[0], a[1], a[2]);
        content.position.set(-a[0], -a[1], -a[2]);
        entry.axis = new THREE.Vector3().fromArray(b.joint.axis).normalize();
        this.byMotor[b.joint.motor] = entry;
      }

      for (const g of b.geoms) {
        const color = new THREE.Color(g.color[0], g.color[1], g.color[2]);
        const mat = new THREE.MeshStandardMaterial({
          color: color.clone(), roughness: 0.55, metalness: 0.25,
        });
        const mesh = new THREE.Mesh(geoms[g.mesh], mat);
        mesh.position.fromArray(g.pos);
        mesh.quaternion.set(g.quat[1], g.quat[2], g.quat[3], g.quat[0]);
        mesh.userData.body = i;
        content.add(mesh);
        entry.meshes.push(mesh);
        entry.baseColors.push(color);
      }

      // parent 0 is the MuJoCo world body; those links hang off our root.
      const parent = b.parent === 0 || b.parent === i
        ? this.root
        : this.bodies[b.parent].content;

      // The trunk is a floating base. Its body_pos in go2.xml is only the
      // simulator's SPAWN height (0.445 m), not a measurement -- leaving it in
      // parks the robot in mid-air with its feet well above the grid. Zero it
      // and drive the height from the live body_height instead, in update().
      if (parent === this.root && i > 0) {
        group.position.set(0, 0, 0);
        this.base = entry;
      }

      parent.add(group);
      this.bodies.push(entry);
    }

    // Foot-load pads: a translucent sphere at each foot's contact point that
    // grows and reddens with load. Parented to the foot body so it tracks the
    // leg automatically.
    for (const f of model.feet) {
      const pad = new THREE.Mesh(
        new THREE.SphereGeometry(1, 20, 14),
        new THREE.MeshBasicMaterial({ color: 0x4ade80, transparent: true, opacity: 0.4 })
      );
      pad.scale.setScalar(0.02);
      this.bodies[f.body].content.add(pad);
      this.footPads.push(pad);
    }

    this.ready = true;
    this.resetView();
    return this;
  }

  // -- live data ---------------------------------------------------------

  update(d) {
    if (!this.ready) return;

    if (d.joints) {
      for (let i = 0; i < MOTOR_COUNT; i++) {
        const body = this.byMotor[i];
        if (!body) continue;
        body.pivot.quaternion.setFromAxisAngle(body.axis, d.joints[i].q);
        this._colorBody(body, d.joints[i]);
      }
    }

    // Body attitude. The IMU reports roll/pitch/yaw of the trunk in the world
    // frame; applying it to root tips the whole robot against the grid.
    if (d.imu) {
      this.root.rotation.set(d.imu.roll, d.imu.pitch, d.imu.yaw, "ZYX");
    }

    // Ride height, so the feet meet the grid rather than hovering over it.
    // SportModeState_.body_height is the trunk height above the ground; when
    // there is no sportmodestate yet, hold a nominal standing height.
    const h = d.motion && d.motion.body_height > 0.05 ? d.motion.body_height : 0.32;
    this.root.position.z = h;

    if (d.feet) {
      for (let i = 0; i < this.footPads.length; i++) {
        // "load" is newtons from the leg motors (dashboard.py leg_load); older
        // dashboards only send the raw sensor "force".
        const ft = d.feet[i] || {};
        const f = Math.max(0, ft.load != null ? ft.load : ft.force || 0);
        const t = Math.min(1, f / 100);           // 100 N ~ one foot taking half the robot
        this.footPads[i].scale.setScalar(0.018 + 0.055 * t);
        this.footPads[i].material.color = ramp(t);
        this.footPads[i].material.opacity = 0.2 + 0.5 * t;
      }
    }

    this.latest = d;
  }

  setColorMode(mode) {
    this.colorMode = mode;
    if (this.latest) this.update(this.latest);
  }

  _colorBody(body, joint) {
    let t;
    if (this.colorMode === "torque") {
      t = (Math.abs(joint.tau) - TAU_RANGE[0]) / (TAU_RANGE[1] - TAU_RANGE[0]);
    } else if (this.colorMode === "temperature") {
      t = (joint.temp - TEMP_RANGE[0]) / (TEMP_RANGE[1] - TEMP_RANGE[0]);
    } else {
      // "off" -- put the original material colours back.
      for (let i = 0; i < body.meshes.length; i++) {
        body.meshes[i].material.color.copy(body.baseColors[i]);
      }
      return;
    }
    const c = ramp(t);
    for (const m of body.meshes) m.material.color.copy(c);
  }

  // -- view presets ------------------------------------------------------

  resetView() { this._moveTo(0.72, -0.72, 0.42); }
  topView() { this._moveTo(0.001, 0, 1.4); }
  sideView() { this._moveTo(0, -1.3, 0.25); }

  _moveTo(x, y, z) {
    this.camera.position.set(x, y, z);
    this.controls.target.set(0, 0, 0.22);
    this.controls.update();
  }

  // -- hover -------------------------------------------------------------

  _onPointerMove(e) {
    const r = this.canvas.getBoundingClientRect();
    this.pointer.x = ((e.clientX - r.left) / r.width) * 2 - 1;
    this.pointer.y = -((e.clientY - r.top) / r.height) * 2 + 1;
    this._tooltipAt = [e.clientX - r.left, e.clientY - r.top];
  }

  _updateHover() {
    if (!this.ready || !this._tooltipAt || !this.latest) return;
    this.raycaster.setFromCamera(this.pointer, this.camera);
    const hits = this.raycaster.intersectObject(this.root, true);
    const hit = hits.find((h) => h.object.userData.body !== undefined);

    if (!hit) {
      this.tooltip.style.display = "none";
      return;
    }
    const body = this.bodies[hit.object.userData.body];
    const j = body.joint ? this.latest.joints[body.joint.motor] : null;
    const deg = (v) => (v * 180 / Math.PI).toFixed(1);

    this.tooltip.style.display = "block";
    this.tooltip.style.left = this._tooltipAt[0] + 14 + "px";
    this.tooltip.style.top = this._tooltipAt[1] + 14 + "px";
    this.tooltip.innerHTML = j
      ? `<b>${j.name}</b><br>angle ${deg(j.q)}&deg; (${j.q.toFixed(3)} rad)` +
        `<br>velocity ${j.dq.toFixed(2)} rad/s<br>torque ${j.tau.toFixed(2)} N&middot;m` +
        `<br>temperature ${j.temp}&deg;C`
      : `<b>${body.name}</b><br>no motor on this link`;
  }

  // -- loop --------------------------------------------------------------

  _resize() {
    const el = this.canvas.parentElement;
    const w = el.clientWidth, h = el.clientHeight;
    if (!w || !h) return;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  _animate() {
    requestAnimationFrame(() => this._animate());
    this.controls.update();
    this._updateHover();
    this.renderer.render(this.scene, this.camera);
  }
}
