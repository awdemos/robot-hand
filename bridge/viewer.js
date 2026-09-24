/* Biomechanical Tendon Hand viewer.
 * Data: GET /arm, SSE /events. Manual sliders + goals + Replicanta volition.
 * Realistic rendering: metallic bone segments, visible pulleys, multi-strand
 * flexor/extensor cables that tighten and loosen with live tendon force.
 */
(() => {
  "use strict";

  // Guard and error surfacing must exist before any THREE use: if three.js
  // failed to load, the code below would abort with no message and no fallback.
  if (typeof THREE === "undefined" || !THREE.WebGLRenderer) { showErr("three.js failed to load"); return; }
  if (!THREE.CatmullRomCurve3 || !THREE.TubeGeometry) { showErr("three.js missing geometry constructors"); return; }
  window.addEventListener("error", (e) => showErr("js error: " + e.message));
  window.addEventListener("unhandledrejection", (e) => showErr("promise: " + (e.reason || "")));

  const FINGERS = ["thumb", "index", "middle", "ring", "pinky"];
  const JOINT_NAMES = { thumb: ["cmc", "mcp", "ip"], index: ["mcp", "pip", "dip"], middle: ["mcp", "pip", "dip"], ring: ["mcp", "pip", "dip"], pinky: ["mcp", "pip", "dip"] };
  const D2R = Math.PI / 180;

  // Right hand. base = palm attachment (cm). angles in degrees.
  const ANATOMY = {
    thumb:  { base: [1.1, -1.35, -0.2], curlAxis: "z", yaw: 55, roll: 120,
              segs: [["cmc", 1.8, 0.46], ["mcp", 2.8, 0.48], ["ip", 2.4, 0.40]] },
    index:  { base: [2.0, 1.55, -3.2], curlAxis: "z", yaw: -3, roll: 5,
              segs: [["mcp", 4.2, 0.46], ["pip", 2.6, 0.40], ["dip", 1.9, 0.34]] },
    middle: { base: [1.28, 1.72, -3.65], curlAxis: "z", yaw: 0, roll: 0,
              segs: [["mcp", 4.5, 0.48], ["pip", 3.0, 0.42], ["dip", 2.0, 0.35]] },
    ring:   { base: [0.5, 1.62, -3.45], curlAxis: "z", yaw: 4, roll: -6,
              segs: [["mcp", 4.1, 0.45], ["pip", 2.8, 0.39], ["dip", 1.95, 0.33]] },
    pinky:  { base: [-0.35, 1.35, -2.85], curlAxis: "z", yaw: 9, roll: -12,
              segs: [["mcp", 3.4, 0.40], ["pip", 2.2, 0.34], ["dip", 1.7, 0.29]] },
  };

  const canvas = document.getElementById("scene");
  let renderer, ctx2d, webglFailed = false;
  try {
    renderer = new THREE.WebGLRenderer({ canvas, antialias: true, powerPreference: "high-performance" });
  } catch (e1) {
    try { renderer = new THREE.WebGLRenderer({ canvas, antialias: false, powerPreference: "low-power" }); } catch (e2) { webglFailed = true; }
  }
  const force2D = location.search.includes("fallback=1");
  if (!webglFailed && (!renderer || !renderer.getContext())) webglFailed = true;
  if (force2D) webglFailed = true;
  if (webglFailed) {
    canvas.style.background = "var(--bg)";
    ctx2d = canvas.getContext("2d");
    if (!ctx2d) { canvas.outerHTML = `<div id="scene" style="display:flex;align-items:center;justify-content:center;color:#c96b4a;padding:40px;">WebGL unavailable and 2D canvas unsupported. Open this page in a browser with hardware acceleration enabled.</div>`; return; }
  } else {
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.0;
  }

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0b0d10);
  scene.fog = new THREE.FogExp2(0x0b0d10, 0.018);

  const camera = new THREE.PerspectiveCamera(40, 1, 0.1, 120);
  const cam = { theta: -0.55, phi: 0.35, dist: 32, target: new THREE.Vector3(0.6, 1.0, -5.0) };

  scene.add(new THREE.HemisphereLight(0x8fa3b0, 0x1a1512, 0.45));
  const key = new THREE.DirectionalLight(0xfff1dd, 2.2);
  key.position.set(10, 16, 6);
  key.castShadow = true;
  key.shadow.mapSize.set(2048, 2048);
  key.shadow.camera.left = -18; key.shadow.camera.right = 18;
  key.shadow.camera.top = 18; key.shadow.camera.bottom = -18;
  key.shadow.bias = -0.0006;
  scene.add(key);
  const rim = new THREE.DirectionalLight(0x87a9ff, 0.8);
  rim.position.set(-10, 6, -12);
  scene.add(rim);
  const fill = new THREE.PointLight(0xd8a24a, 10, 36, 2);
  fill.position.set(-3, 4, 8);
  scene.add(fill);

  const groundMat = new THREE.MeshStandardMaterial({ color: 0x15171c, roughness: 0.92, metalness: 0.06 });
  const ground = new THREE.Mesh(new THREE.PlaneGeometry(120, 120), groundMat);
  ground.rotation.x = -Math.PI / 2;
  ground.position.y = -3.6;
  ground.receiveShadow = true;
  scene.add(ground);
  const grid = new THREE.GridHelper(80, 80, 0x2a2e35, 0x1a1d22);
  grid.position.y = -3.58;
  scene.add(grid);

  // materials: mechanical bone, steel joint caps, tendon sheaths
  const boneMat = new THREE.MeshStandardMaterial({ color: 0xbfaea1, roughness: 0.45, metalness: 0.25 });
  const jointMat = new THREE.MeshStandardMaterial({ color: 0x8c8c94, roughness: 0.35, metalness: 0.72 });
  const pulleyMat = new THREE.MeshStandardMaterial({ color: 0x707078, roughness: 0.4, metalness: 0.6 });
  const nailMat = new THREE.MeshPhysicalMaterial({ color: 0xe8ddd1, roughness: 0.25, metalness: 0.1, clearcoat: 0.3, clearcoatRoughness: 0.2 });
  const flexorMat = new THREE.MeshStandardMaterial({ color: 0xc24a32, roughness: 0.55, metalness: 0.0, emissive: 0x4a120a, emissiveIntensity: 0.12 });
  const extensorMat = new THREE.MeshStandardMaterial({ color: 0xd8d0c4, roughness: 0.55, metalness: 0.0 });
  const sheathMat = new THREE.MeshStandardMaterial({ color: 0x4a4a52, roughness: 0.7, metalness: 0.2, transparent: true, opacity: 0.55 });

  const armRoot = new THREE.Group();
  scene.add(armRoot);

  // Forearm
  const forearm = new THREE.Mesh(new THREE.CapsuleGeometry(2.6, 11, 16, 24), boneMat);
  forearm.rotation.x = -Math.PI / 2;
  forearm.position.set(0.5, -1.0, 7.0);
  forearm.castShadow = true;
  armRoot.add(forearm);

  // Palm: low-poly block with knuckle ridge
  const palmGeo = new THREE.BoxGeometry(5.0, 1.4, 6.2);
  palmGeo.translate(0.8, 0.7, -3.1);
  const palm = new THREE.Mesh(palmGeo, boneMat);
  palm.castShadow = true; palm.receiveShadow = true;
  armRoot.add(palm);

  // Knuckle bar across metacarpal heads
  const knuckleBar = new THREE.Mesh(new THREE.CylinderGeometry(0.28, 0.28, 5.4, 16), jointMat);
  knuckleBar.rotation.z = Math.PI / 2;
  knuckleBar.position.set(0.9, 1.35, -5.4);
  knuckleBar.castShadow = true;
  armRoot.add(knuckleBar);

  const digits = {};
  const tendons = { flexor: [], extensor: [] };

  function createPhalanx(radius, length) {
    // flattened capsule for phalanx: radius slightly oval
    const g = new THREE.CapsuleGeometry(radius, length, 12, 20);
    const m = new THREE.Mesh(g, boneMat);
    m.castShadow = true; m.receiveShadow = true;
    return m;
  }

  function createJointDisc(radius) {
    const g = new THREE.CylinderGeometry(radius * 1.25, radius * 1.25, radius * 0.7, 20);
    const m = new THREE.Mesh(g, jointMat);
    m.rotation.x = Math.PI / 2;
    m.castShadow = true;
    return m;
  }

  function createPulley(radius) {
    const g = new THREE.TorusGeometry(radius, radius * 0.35, 10, 24);
    const m = new THREE.Mesh(g, pulleyMat);
    m.castShadow = true;
    return m;
  }

  for (const name of FINGERS) {
    const A = ANATOMY[name];
    const anchor = new THREE.Group();
    anchor.position.set(...A.base);
    anchor.rotation.y = A.yaw * D2R;
    anchor.rotation.z = A.roll * D2R;
    armRoot.add(anchor);

    const joints = [];
    let parent = anchor;
    for (let i = 0; i < A.segs.length; i++) {
      const [jname, len, rad] = A.segs[i];
      const pivot = new THREE.Group();
      if (i === 0) {
        // base joint at knuckle: initial orientation aligns segment along -Z
        pivot.position.set(0, 0, 0);
      } else {
        pivot.position.set(0, 0, -joints[i - 1].len);
      }
      parent.add(pivot);

      const disc = createJointDisc(rad);
      pivot.add(disc);

      // tendon pulleys on each side of the joint
      const flexPulley = createPulley(rad * 0.8);
      flexPulley.position.set(rad * 0.6, 0, 0);
      pivot.add(flexPulley);
      const extPulley = createPulley(rad * 0.8);
      extPulley.position.set(-rad * 0.6, 0, 0);
      pivot.add(extPulley);

      const phalanx = createPhalanx(rad, len);
      phalanx.position.set(0, 0, -len / 2);
      pivot.add(phalanx);

      // nail on distal phalanx
      if (i === A.segs.length - 1) {
        const nail = new THREE.Mesh(new THREE.SphereGeometry(rad * 0.78, 14, 10), nailMat);
        nail.scale.set(0.8, 0.35, 1.25);
        nail.position.set(0, rad * 0.55, -len * 0.6);
        pivot.add(nail);
      }

      joints.push({ name: jname, pivot, len, rad, flexPulley, extPulley, phalanx });
      parent = pivot;
    }
    digits[name] = { anchor, joints };
  }

  // Tendon strands. Build once, update geometry in animate.
  const TENDON_SEGMENTS = 32;
  function makeStrandMat(kind, n) {
    const mat = kind === "flexor" ? flexorMat.clone() : extensorMat.clone();
    mat.opacity = 0.7 + n * 0.08;
    mat.transparent = true;
    return mat;
  }

  const strandMeshes = [];
  for (const name of FINGERS) {
    const A = ANATOMY[name];
    for (let strand = 0; strand < 3; strand++) {
      const flexGeo = new THREE.TubeGeometry(new THREE.CatmullRomCurve3([new THREE.Vector3(), new THREE.Vector3()]), 1, 0.018 + strand * 0.006, 6, false);
      const flexMesh = new THREE.Mesh(flexGeo, makeStrandMat("flexor", strand));
      flexMesh.castShadow = true;
      armRoot.add(flexMesh);
      strandMeshes.push({ finger: name, kind: "flexor", strand, mesh: flexMesh, sheath: false });

      const extGeo = new THREE.TubeGeometry(new THREE.CatmullRomCurve3([new THREE.Vector3(), new THREE.Vector3()]), 1, 0.018 + strand * 0.006, 6, false);
      const extMesh = new THREE.Mesh(extGeo, makeStrandMat("extensor", strand));
      extMesh.castShadow = true;
      armRoot.add(extMesh);
      strandMeshes.push({ finger: name, kind: "extensor", strand, mesh: extMesh, sheath: false });
    }
  }

  // Sheath tubes (stationary guides) from forearm to knuckles
  const sheaths = [];
  for (const name of FINGERS) {
    const A = ANATOMY[name];
    const start = new THREE.Vector3(...A.base).add(new THREE.Vector3(0, 1.2, 2.5));
    const end = new THREE.Vector3(...A.base);
    const path = new THREE.LineCurve3(start, end);
    const tube = new THREE.Mesh(new THREE.TubeGeometry(path, 4, 0.12, 8, false), sheathMat);
    tube.castShadow = true;
    armRoot.add(tube);
    sheaths.push(tube);
  }

  // State variables
  let bridgeLive = false;
  let lastState = null;
  let appliedState = null;
  const cur = { thumb: 0.05, index: 0.05, middle: 0.05, ring: 0.05, pinky: 0.05 };
  let curSpread = 0.12, curOpp = 0.1, curPitch = 0, curYaw = 0;
  const curAngles = { thumb: { cmc: 0, mcp: 0, ip: 0 }, index: { mcp: 0, pip: 0, dip: 0 }, middle: { mcp: 0, pip: 0, dip: 0 }, ring: { mcp: 0, pip: 0, dip: 0 }, pinky: { mcp: 0, pip: 0, dip: 0 } };
  const curForces = {};
  for (const f of FINGERS) { curForces[f] = {}; for (const j of JOINT_NAMES[f]) curForces[f][j] = { flex: 0, ext: 0 }; }

  function synthesizeState() {
    const fingers = {};
    for (const f of FINGERS) {
      const joints = {};
      for (const j of JOINT_NAMES[f]) {
        const a = curAngles[f][j];
        const ft = curForces[f][j].flex || 0;
        const et = curForces[f][j].ext || 0;
        joints[j] = {
          angle: a, flex_force: ft, ext_force: et, flex_activation: ft / 90, ext_activation: et / 90,
          strain: Math.max(0, ft - 70) / 25, heat: 0, velocity: 0, contact: 0,
        };
      }
      fingers[f] = { curl_target: cur[f], segments: ANATOMY[f].segs.map(s => ({ name: s[0], len: s[1] })), joints };
    }
    return { fingers, spread: curSpread, thumb_opposition: curOpp, wrist: { pitch: curPitch, yaw: curYaw }, wrist_target: { pitch: curPitch, yaw: curYaw }, goal: localSim.goal, load_contact: false, emotion: { stress: 0.2, arousal: 0.3, mood: "offline" }, time: 0 };
  }

  function applyState(s) {
    if (!s || !s.fingers) return;
    for (const f of FINGERS) {
      const fd = s.fingers[f];
      if (!fd) continue;
      cur[f] = fd.curl_target || 0;
      for (const j of JOINT_NAMES[f]) {
        const jd = fd.joints[j];
        if (!jd) continue;
        curAngles[f][j] = jd.angle || 0;
        curForces[f][j].flex = jd.flex_force || 0;
        curForces[f][j].ext = jd.ext_force || 0;
      }
    }
    curSpread = s.spread || 0;
    curOpp = s.thumb_opposition || 0;
    curPitch = (s.wrist || {}).pitch || 0;
    curYaw = (s.wrist || {}).yaw || 0;

    // Update 3D skeleton
    for (const f of FINGERS) {
      const digit = digits[f];
      const spreadBase = f === "thumb" ? curOpp * 0.9 : (curSpread - 0.12) * (f === "index" ? 0.8 : f === "pinky" ? -0.8 : 0);
      digit.anchor.rotation.y = (ANATOMY[f].yaw * D2R) + spreadBase;
      for (let i = 0; i < digit.joints.length; i++) {
        const j = digit.joints[i];
        const a = curAngles[f][j.name] * D2R;
        j.pivot.rotation.x = a;
        // pulleys stay perpendicular to tendon pull
        j.flexPulley.rotation.y = Math.sin(a) * 0.3;
        j.extPulley.rotation.y = -Math.sin(a) * 0.3;
      }
    }
    armRoot.rotation.x = curPitch * 0.5;
    armRoot.rotation.y = curYaw * 0.6;

    updateTendons();
    updateHUD(s);
  }

  function updateTendons() {
    // SSE applyState mutates pivots between animation frames; matrixWorld is
    // only refreshed during render, so force it before sampling joint poses.
    scene.updateMatrixWorld(true);
    const now = performance.now();
    for (const item of strandMeshes) {
      const f = item.finger;
      const A = ANATOMY[f];
      const digit = digits[f];
      const kind = item.kind;
      const side = kind === "flexor" ? 1 : -1;
      const strandOffset = (item.strand - 1) * 0.14;
      const sheathStart = new THREE.Vector3(...A.base).add(new THREE.Vector3(side * (0.35 + strandOffset), 1.2, 2.5));

      // Collect points along the active tendon path: sheath -> each joint pulley -> tip
      const points = [sheathStart.clone()];
      let tip = null;
      for (let i = 0; i < digit.joints.length; i++) {
        const j = digit.joints[i];
        // pulley position, converted from world into armRoot-local (meshes parent to armRoot)
        const pulleyLocal = new THREE.Vector3(side * j.rad * 0.6, 0, -j.len * (i === 0 ? 0.0 : 1.0));
        const p = armRoot.worldToLocal(pulleyLocal.applyMatrix4(j.pivot.matrixWorld));
        points.push(p);
        if (i === digit.joints.length - 1) tip = p;
      }
      if (tip) {
        const end = tip.clone().add(new THREE.Vector3(side * 0.12, 0.05, -0.4));
        points.push(end);
      }

      // Sag based on slack: low force = more sag
      const lastJoint = JOINT_NAMES[f][JOINT_NAMES[f].length - 1];
      const force = curForces[f][lastJoint][kind === "flexor" ? "flex" : "ext"] || 0;
      const slack = Math.max(0, 1 - force / 35);
      for (let i = 1; i < points.length - 1; i++) {
        points[i].y -= slack * (0.2 + 0.2 * Math.sin(i + now * 0.001));
      }

      item.mesh.geometry.dispose();
      item.mesh.geometry = new THREE.TubeGeometry(new THREE.CatmullRomCurve3(points), 24, 0.018 + item.strand * 0.006, 6, false);
      // Color intensity from force
      const intensity = Math.min(1, force / 70);
      if (kind === "flexor") item.mesh.material.emissiveIntensity = 0.1 + intensity * 0.55;
      else item.mesh.material.opacity = 0.55 + intensity * 0.35;
    }
  }

  function updateHUD(s) {
    for (const f of FINGERS) {
      const js = s.fingers[f].joints;
      const keys = Object.keys(js);
      const t = keys.reduce((a, k) => a + (js[k].flex_force || 0), 0) / (keys.length * 90);
      const strain = keys.reduce((a, k) => Math.max(a, js[k].strain || 0), 0);
      const heat = keys.reduce((a, k) => Math.max(a, js[k].heat || 0), 0);
      const bar = document.getElementById("t-" + f);
      if (bar) {
        bar.style.insetInlineEnd = `${(1 - t) * 100}%`;
        bar.classList.toggle("hot", strain > 0.55 || heat > 0.5);
      }
      const v = document.getElementById("v-" + f);
      if (v) v.textContent = `${Math.round(t * 100)}%`;
    }
    const m = document.getElementById("mind");
    if (s.emotion) {
      m.innerHTML = `state <b>${escapeHtml(s.emotion.mood || "—")}</b> · stress <b>${(s.emotion.stress * 100) | 0}%</b> · arousal <b>${(s.emotion.arousal * 100) | 0}%</b>` +
        (s.goal ? `<br>volition → <b id="goalTag">${escapeHtml(s.goal)}</b>` : "<br>volition idle");
    }
  }
  function escapeHtml(t) {
    return String(t).replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
  }

  // telemetry rows for the TENDON TELEMETRY panel (ids t-<f> / v-<f> read by updateHUD)
  const trows = document.getElementById("trows");
  for (const f of FINGERS) {
    const row = document.createElement("div");
    row.className = "frow";
    row.innerHTML = `<span>${f}</span><span class="tbar"><i id="t-${f}"></i></span><span id="v-${f}">0%</span>`;
    trows.appendChild(row);
  }

  // manual UI
  const srows = document.getElementById("srows");
  const manual = { fingers: {} };
  for (const f of FINGERS) manual.fingers[f] = 0;
  manual.spread = 0.12; manual.opp = 0.1; manual.pitch = 0; manual.yaw = 0;
  for (const f of FINGERS) {
    const row = document.createElement("div");
    row.className = "srow";
    row.innerHTML = `<label>${f}</label><input data-f="${f}" type="range" min="0" max="1" step=".01" value="0">`;
    srows.appendChild(row);
  }
  [["spread", 0, 1, 0.12], ["opposition", 0, 1, 0.1], ["pitch", -1, 1, 0], ["yaw", -1, 1, 0]].forEach(([id, mn, mx, defv]) => {
    const row = document.createElement("div");
    row.className = "srow";
    row.innerHTML = `<label>${id}</label><input data-global="${id}" type="range" min="${mn}" max="${mx}" step=".01" value="${defv}">`;
    srows.appendChild(row);
  });
  srows.addEventListener("input", (e) => {
    const el = e.target;
    if (el.dataset.f) manual.fingers[el.dataset.f] = parseFloat(el.value);
    else manual[el.dataset.global] = parseFloat(el.value);
    if (!bridgeLive) {
      cur[el.dataset.f || "thumb"] = manual.fingers[el.dataset.f || "thumb"] || 0;
      curSpread = manual.spread; curOpp = manual.opp; curPitch = manual.pitch; curYaw = manual.yaw;
    }
  });
  srows.addEventListener("change", () => {
    if (bridgeLive) api("/pose", { finger_targets: manual.fingers, spread: manual.spread, thumb_opposition: manual.opp, wrist: { pitch: manual.pitch, yaw: manual.yaw }, duration_s: 2 }).catch(() => setConn(false));
  });

  document.querySelectorAll(".goals button").forEach((b) => {
    b.addEventListener("click", () => {
      if (!bridgeLive) { localSim.goal = b.dataset.g; localSim.goalUntil = performance.now() + 4000; }
      else api("/goal", { kind: b.dataset.g, duration_s: 4 }).catch(() => {});
    });
  });

  let localSim = { goal: null, goalUntil: 0, wavePhase: 0 };
  // joint maxima (degrees) for mapping 0..1 curl onto the offline skeleton
  const MAXA = { cmc: 55, mcp: 90, pip: 100, dip: 75, ip: 55 };
  function offlineTick(dt) {
    if (localSim.goal === "wave") { localSim.wavePhase += dt * 9; curYaw = Math.sin(localSim.wavePhase) * 0.55; }
    if (localSim.goal && performance.now() > localSim.goalUntil) {
      for (const f of FINGERS) cur[f] = Math.min(cur[f], 0.1);
      curSpread = 0.15; curOpp = 0.05; curPitch = 0; curYaw = 0; localSim.goal = null;
    }
    const k = Math.min(1, dt * 4.5);
    for (const f of FINGERS) cur[f] += ((manual.fingers[f] || 0) - cur[f]) * k;
    curSpread += (manual.spread - curSpread) * k;
    curOpp += (manual.opp - curOpp) * k;
    curPitch += (manual.pitch - curPitch) * k;
    curYaw += (manual.yaw - curYaw) * k;
    for (const f of FINGERS) {
      const target = cur[f];
      for (let i = 0; i < JOINT_NAMES[f].length; i++) {
        const j = JOINT_NAMES[f][i];
        const mix = i === 0 ? 0.45 : i === 1 ? 0.38 : 0.22;
        curAngles[f][j] += (target * mix * MAXA[j] - curAngles[f][j]) * k;
      }
    }
    applyState(synthesizeState());
  }

  async function api(path, body) {
    const r = await fetch(path, {
      method: body ? "POST" : "GET",
      headers: body ? { "Content-Type": "application/json" } : undefined,
      body: body ? JSON.stringify(body) : undefined,
    });
    return r.json();
  }

  function connectSSE() {
    const es = new EventSource("/events");
    es.onopen = () => { bridgeLive = true; setConn(true); };
    es.onmessage = (ev) => {
      try {
        const msg = JSON.parse(ev.data);
        if (msg.type === "arm") { lastState = msg.state; applyState(msg.state); }
      } catch (_) {}
    };
    es.onerror = () => { setConn(false); bridgeLive = false; es.close(); setTimeout(connectSSE, 3000); };
  }
  function setConn(on) {
    const el = document.getElementById("conn");
    el.textContent = on ? "● bridge live — replicanta wired" : "● offline — direct drive";
    el.classList.toggle("on", on);
    // goals only act on the live bridge; offline they would be no-ops
    document.querySelectorAll("#goals button").forEach((b) => { b.disabled = !on; });
  }
  function showErr(msg) {
    const el = document.getElementById("conn");
    el.textContent = "● " + msg;
    el.classList.remove("on");
  }

  function resize() {
    if (webglFailed) {
      canvas.width = innerWidth * devicePixelRatio;
      canvas.height = innerHeight * devicePixelRatio;
      canvas.style.width = "100%";
      canvas.style.height = "100%";
      return;
    }
    renderer.setSize(innerWidth, innerHeight, false);
    camera.aspect = innerWidth / innerHeight;
    camera.updateProjectionMatrix();
  }
  addEventListener("resize", resize);
  resize();

  // camera orbit
  let dragging = false, px = 0, py = 0;
  addEventListener("pointerdown", (e) => { if (e.target.closest(".panel")) return; dragging = true; px = e.clientX; py = e.clientY; });
  addEventListener("pointerup", () => { dragging = false; });
  addEventListener("pointermove", (e) => {
    if (!dragging) return;
    cam.theta -= (e.clientX - px) * 0.005;
    cam.phi = Math.max(0.05, Math.min(1.35, cam.phi + (e.clientY - py) * 0.004));
    px = e.clientX; py = e.clientY;
  });
  addEventListener("wheel", (e) => { cam.dist = Math.max(12, Math.min(46, cam.dist + e.deltaY * 0.02)); }, { passive: true });

  setConn(false);
  scene.updateMatrixWorld(true);
  connectSSE();
  api("/arm").then((s) => { bridgeLive = true; setConn(true); lastState = s; applyState(s); updateHUD(s); }).catch(() => setConn(false));

  let last = performance.now();
  function frame(now) {
    const dt = Math.min(0.05, (now - last) / 1000);
    last = now;
    if (!bridgeLive) offlineTick(dt);
    else if (lastState && lastState !== appliedState) { applyState(lastState); appliedState = lastState; }
    if (webglFailed) { draw2D(); requestAnimationFrame(frame); return; }
    const cp = cam.phi, ct = cam.theta;
    camera.position.set(
      cam.target.x + cam.dist * Math.cos(cp) * Math.sin(ct),
      cam.target.y + cam.dist * Math.sin(cp),
      cam.target.z + cam.dist * Math.cos(cp) * Math.cos(ct)
    );
    camera.lookAt(cam.target);
    armRoot.position.y = Math.sin(now * 0.0006) * 0.06;
    try { renderer.render(scene, camera); } catch (e) { showErr("render failed: " + e.message); requestAnimationFrame(frame); return; }
    requestAnimationFrame(frame);
  }

  function draw2D() {
    if (!ctx2d) return;
    const W = canvas.width / (devicePixelRatio || 1);
    const H = canvas.height / (devicePixelRatio || 1);
    const g = ctx2d;
    g.save();
    g.scale(devicePixelRatio || 1, devicePixelRatio || 1);
    g.fillStyle = "#0b0d10"; g.fillRect(0, 0, W, H);

    const cx = W * 0.5, cy = H * 0.42;
    g.save(); g.translate(cx, cy); g.rotate(curYaw * 0.4 + Math.PI * 0.04);

    g.fillStyle = "#6e6056"; g.strokeStyle = "#a38f7e"; g.lineWidth = 2;
    g.beginPath(); g.ellipse(0, 0, 72, 54, 0, 0, Math.PI * 2); g.fill(); g.stroke();

    const layout = [
      { name: "pinky", base: [-58, -28], len: [28, 24, 18], angle: -0.55 },
      { name: "ring", base: [-28, -42], len: [34, 30, 22], angle: -0.22 },
      { name: "middle", base: [4, -46], len: [38, 34, 24], angle: 0.05 },
      { name: "index", base: [38, -38], len: [34, 30, 22], angle: 0.35 },
      { name: "thumb", base: [66, 18], len: [30, 26], angle: 1.15 },
    ];
    for (const f of layout) {
      const curl = cur[f.name] || 0;
      let a = f.angle;
      if (f.name === "thumb") a += curOpp * 0.6;
      else {
        const d = curSpread - 0.12;
        if (f.name === "index") a += d * 1.2;
        else if (f.name === "pinky") a -= d * 1.2;
      }
      let x = f.base[0], y = f.base[1];
      const segs = [];
      for (let i = 0; i < f.len.length; i++) {
        const bend = curl * (i === 0 ? 0.35 : 0.55);
        a += (i === 0 ? 0 : bend);
        const nx = x + Math.cos(a) * f.len[i];
        const ny = y + Math.sin(a) * f.len[i];
        segs.push({ x, y, nx, ny, a, len: f.len[i] });
        x = nx; y = ny;
      }
      g.strokeStyle = "#c9b8a8"; g.lineWidth = 2; g.beginPath(); g.moveTo(segs[0].x, segs[0].y); for (const s of segs) g.lineTo(s.nx, s.ny); g.stroke();
      g.strokeStyle = `rgba(201, 107, 74, ${0.4 + curl * 0.6})`; g.lineWidth = 3; g.beginPath(); g.moveTo(segs[0].x, segs[0].y + 5); for (const s of segs) g.lineTo(s.nx, s.ny + 5 + curl * 6); g.stroke();
      for (const s of segs) {
        g.fillStyle = "#7d6d61"; g.beginPath(); g.arc(s.x, s.y, 6, 0, Math.PI * 2); g.fill();
        g.strokeStyle = "#a38f7e"; g.lineWidth = 2; g.beginPath(); g.moveTo(s.x, s.y); g.lineTo(s.nx, s.ny); g.stroke();
      }
      const tip = segs[segs.length - 1];
      g.fillStyle = "#e6d8cc"; g.beginPath(); g.ellipse(tip.nx, tip.ny, 7, 4, tip.a, 0, Math.PI * 2); g.fill();
    }
    g.fillStyle = "#5c4f47"; g.beginPath(); g.moveTo(-40, 45); g.lineTo(40, 45); g.lineTo(60, 220); g.lineTo(-60, 220); g.closePath(); g.fill(); g.stroke();
    g.restore(); g.restore();
  }

  requestAnimationFrame(frame);
})();
