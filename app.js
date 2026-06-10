import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

const tableBody = document.querySelector("#jointTable");
const sliderList = document.querySelector("#sliderList");
const matrixOutput = document.querySelector("#matrixOutput");
const reportOutput = document.querySelector("#reportOutput");
const jointMatrices = document.querySelector("#jointMatrices");
const dhMode = document.querySelector("#dhMode");
const viewer = document.querySelector("#viewer");
const poseX = document.querySelector("#poseX");
const poseY = document.querySelector("#poseY");
const poseZ = document.querySelector("#poseZ");

const degToRad = Math.PI / 180;
const storageKey = "dh-transform-app-state-v1";
let playing = false;
let lastTick = 0;
let currentViewScale = 1;

function exampleJoints() {
  return [
    { type: "revolute", theta: 0, d: 0.8, a: 1.2, alpha: 0, value: 15, min: -120, max: 120 },
    { type: "revolute", theta: 30, d: 0, a: 1.0, alpha: 0, value: 25, min: -140, max: 140 },
    { type: "prismatic", theta: 0, d: 0.4, a: 0.7, alpha: 90, value: 0.35, min: 0, max: 1.2 }
  ];
}

function normalizeJoint(joint) {
  const type = joint?.type === "prismatic" ? "prismatic" : "revolute";
  return {
    type,
    theta: Number.isFinite(Number(joint?.theta)) ? Number(joint.theta) : 0,
    d: Number.isFinite(Number(joint?.d)) ? Number(joint.d) : 0,
    a: Number.isFinite(Number(joint?.a)) ? Number(joint.a) : 0,
    alpha: Number.isFinite(Number(joint?.alpha)) ? Number(joint.alpha) : 0,
    value: Number.isFinite(Number(joint?.value)) ? Number(joint.value) : 0,
    min: Number.isFinite(Number(joint?.min)) ? Number(joint.min) : type === "revolute" ? -180 : 0,
    max: Number.isFinite(Number(joint?.max)) ? Number(joint.max) : type === "revolute" ? 180 : 1.5
  };
}

function loadState() {
  try {
    const raw = localStorage.getItem(storageKey);
    if (!raw) return { joints: exampleJoints(), dhMode: "classic" };
    const state = JSON.parse(raw);
    const savedJoints = Array.isArray(state.joints) && state.joints.length ? state.joints.map(normalizeJoint) : exampleJoints();
    return {
      joints: savedJoints,
      dhMode: state.dhMode === "modified" ? "modified" : "classic"
    };
  } catch {
    return { joints: exampleJoints(), dhMode: "classic" };
  }
}

function saveState() {
  try {
    localStorage.setItem(storageKey, JSON.stringify({ joints, dhMode: dhMode.value }));
  } catch {
    // Some embedded browsers restrict localStorage. Calculation still works without persistence.
  }
}

const loadedState = loadState();
let joints = loadedState.joints;
dhMode.value = loadedState.dhMode;

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x000000);

const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 1000);
camera.position.set(4.6, 4.0, 3.2);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
viewer.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.target.set(1.2, 0, 0.6);

scene.add(new THREE.HemisphereLight(0xffffff, 0x242424, 2.6));
const keyLight = new THREE.DirectionalLight(0xffffff, 2.2);
keyLight.position.set(4, 6, 5);
scene.add(keyLight);

const grid = new THREE.GridHelper(8, 16, 0x6f6f6f, 0x2a2a2a);
scene.add(grid);

const robotGroup = new THREE.Group();
scene.add(robotGroup);

function multiply(A, B) {
  const result = Array.from({ length: 4 }, () => Array(4).fill(0));
  for (let r = 0; r < 4; r += 1) {
    for (let c = 0; c < 4; c += 1) {
      for (let k = 0; k < 4; k += 1) result[r][c] += A[r][k] * B[k][c];
    }
  }
  return result;
}

function identity() {
  return [
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
  ];
}

function transformMatrix(joint) {
  const theta = (joint.theta + (joint.type === "revolute" ? joint.value : 0)) * degToRad;
  const d = joint.d + (joint.type === "prismatic" ? joint.value : 0);
  const a = joint.a;
  const alpha = joint.alpha * degToRad;
  const ct = Math.cos(theta);
  const st = Math.sin(theta);
  const ca = Math.cos(alpha);
  const sa = Math.sin(alpha);

  if (dhMode.value === "modified") {
    return [
      [ct, -st, 0, a],
      [st * ca, ct * ca, -sa, -d * sa],
      [st * sa, ct * sa, ca, d * ca],
      [0, 0, 0, 1]
    ];
  }

  return [
    [ct, -st * ca, st * sa, a * ct],
    [st, ct * ca, -ct * sa, a * st],
    [0, sa, ca, d],
    [0, 0, 0, 1]
  ];
}

function toThreeMatrix(m) {
  const xAxis = robotVectorToThree(m[0][0], m[1][0], m[2][0], 1);
  const yAxis = robotVectorToThree(m[0][1], m[1][1], m[2][1], 1);
  const zAxis = robotVectorToThree(m[0][2], m[1][2], m[2][2], 1);
  const position = robotVectorToThree(m[0][3], m[1][3], m[2][3], currentViewScale);

  return new THREE.Matrix4().set(
    xAxis.x, yAxis.x, zAxis.x, position.x,
    xAxis.y, yAxis.y, zAxis.y, position.y,
    xAxis.z, yAxis.z, zAxis.z, position.z,
    0, 0, 0, 1
  );
}

function computeChain() {
  let total = identity();
  const frames = [identity()];
  const localMatrices = [];
  const cumulativeMatrices = [];
  joints.forEach((joint) => {
    const local = transformMatrix(joint);
    localMatrices.push(local);
    total = multiply(total, local);
    cumulativeMatrices.push(total);
    frames.push(total);
  });
  return { total, frames, localMatrices, cumulativeMatrices };
}

function formatNumber(value) {
  if (Math.abs(value) < 1e-10) return "0.000";
  return value.toFixed(3);
}

function formatMatrix(matrix) {
  return matrix.map((row) => `[ ${row.map((value) => formatNumber(value).padStart(8, " ")).join("  ")} ]`).join("\n");
}

function robotVectorToThree(x, y, z, scale = currentViewScale) {
  return new THREE.Vector3(x * scale, z * scale, -y * scale);
}

function framePosition(frame) {
  return robotVectorToThree(frame[0][3], frame[1][3], frame[2][3]);
}

function calculateViewScale(frames) {
  const rawPoints = frames.map((frame) => new THREE.Vector3(frame[0][3], frame[1][3], frame[2][3]));
  const box = new THREE.Box3().setFromPoints(rawPoints);
  const size = Math.max(box.getSize(new THREE.Vector3()).length(), 0.001);
  return THREE.MathUtils.clamp(4 / size, 0.001, 4);
}

function activeJointValues(joint) {
  return {
    theta: joint.theta + (joint.type === "revolute" ? joint.value : 0),
    d: joint.d + (joint.type === "prismatic" ? joint.value : 0),
    a: joint.a,
    alpha: joint.alpha
  };
}

function buildReport(total, localMatrices, cumulativeMatrices) {
  const lines = ["Einzeltransformationen", ""];

  joints.forEach((joint, index) => {
    const values = activeJointValues(joint);
    const jointType = joint.type === "revolute" ? "R" : "P";
    const label = `G${index + 1}`;
    lines.push(
      `${index + 1}. ${label} (${jointType})  theta=${formatNumber(values.theta)} deg, d=${formatNumber(values.d)}, a=${formatNumber(values.a)}, alpha=${formatNumber(values.alpha)} deg`
    );
    lines.push(`Transformation von G${index + 1} zu G${index} = T${index + 1}${index}`);
    lines.push(formatMatrix(localMatrices[index]));
    lines.push("");
  });

  lines.push("Gesamttransformationen", "");
  cumulativeMatrices.forEach((matrix, index) => {
    lines.push(`Gesamttransformation von G${index + 1} zu G0 = Tn0 (n = ${index + 1})`);
    lines.push(formatMatrix(matrix));
    lines.push("");
  });

  lines.push(`Endeffektor: x=${formatNumber(total[0][3])}, y=${formatNumber(total[1][3])}, z=${formatNumber(total[2][3])}`);
  lines.push(`Endeffektor-Transformation von Gn zu G0 = Tn0 (n = ${joints.length})`);
  lines.push(formatMatrix(total));
  return lines.join("\n");
}

function renderTable() {
  tableBody.innerHTML = "";
  joints.forEach((joint, index) => {
    const row = document.createElement("tr");
    row.innerHTML = `
      <td>${index + 1}</td>
      <td>
        <select data-field="type" data-index="${index}">
          <option value="revolute"${joint.type === "revolute" ? " selected" : ""}>R</option>
          <option value="prismatic"${joint.type === "prismatic" ? " selected" : ""}>P</option>
        </select>
      </td>
      <td><input data-field="theta" data-index="${index}" type="number" step="1" value="${joint.theta}"></td>
      <td><input data-field="d" data-index="${index}" type="number" step="0.1" value="${joint.d}"></td>
      <td><input data-field="a" data-index="${index}" type="number" step="0.1" value="${joint.a}"></td>
      <td><input data-field="alpha" data-index="${index}" type="number" step="1" value="${joint.alpha}"></td>
    `;
    tableBody.appendChild(row);
  });
}

function renderSliders() {
  sliderList.innerHTML = "";
  joints.forEach((joint, index) => {
    const unit = joint.type === "revolute" ? "deg" : "m";
    const step = joint.type === "revolute" ? "1" : "0.01";
    const row = document.createElement("label");
    row.className = "slider-row";
    row.innerHTML = `
      <span>q${index + 1}</span>
      <input data-slider="${index}" type="range" min="${joint.min}" max="${joint.max}" step="${step}" value="${joint.value}">
      <strong>${formatNumber(joint.value)} ${unit}</strong>
      <button class="zero-button secondary" data-zero="${index}" type="button" title="q${index + 1} auf 0 setzen">0</button>
    `;
    sliderList.appendChild(row);
  });
}

function makeAxisLabel(text, color) {
  const canvas = document.createElement("canvas");
  canvas.width = 96;
  canvas.height = 40;
  const ctx = canvas.getContext("2d");
  ctx.fillStyle = color;
  ctx.font = "700 24px sans-serif";
  ctx.fillText(text, 8, 28);
  const texture = new THREE.CanvasTexture(canvas);
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, transparent: true }));
  sprite.scale.set(0.28, 0.12, 1);
  return sprite;
}

function addFrame(matrix, index) {
  const frame = new THREE.Group();
  frame.applyMatrix4(toThreeMatrix(matrix));
  frame.add(new THREE.AxesHelper(0.42));
  const label = makeAxisLabel(`F${index}`, "#ffffff");
  label.position.set(0.12, 0.12, 0.42);
  frame.add(label);
  robotGroup.add(frame);
}

function addJointSphere(position, index) {
  const geometry = new THREE.SphereGeometry(index === 0 ? 0.08 : 0.07, 24, 16);
  const material = new THREE.MeshStandardMaterial({ color: index === 0 ? 0xffffff : 0xc12872, roughness: 0.45 });
  const sphere = new THREE.Mesh(geometry, material);
  sphere.position.copy(position);
  robotGroup.add(sphere);
}

function addLink(start, end) {
  const direction = new THREE.Vector3().subVectors(end, start);
  const length = direction.length();
  if (length < 0.001) return;
  const geometry = new THREE.CylinderGeometry(0.035, 0.035, length, 18);
  const material = new THREE.MeshStandardMaterial({ color: 0xffffff, metalness: 0.15, roughness: 0.35 });
  const cylinder = new THREE.Mesh(geometry, material);
  cylinder.position.copy(start).addScaledVector(direction, 0.5);
  cylinder.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), direction.clone().normalize());
  robotGroup.add(cylinder);
}

function drawRobot(frames) {
  robotGroup.clear();
  currentViewScale = calculateViewScale(frames);
  const positions = frames.map(framePosition);
  positions.forEach((position, index) => {
    addJointSphere(position, index);
    addFrame(frames[index], index);
    if (index > 0) addLink(positions[index - 1], position);
  });
}

function updateOutput() {
  const { total, frames, localMatrices, cumulativeMatrices } = computeChain();
  matrixOutput.textContent = formatMatrix(total);
  reportOutput.textContent = buildReport(total, localMatrices, cumulativeMatrices);
  poseX.textContent = formatNumber(total[0][3]);
  poseY.textContent = formatNumber(total[1][3]);
  poseZ.textContent = formatNumber(total[2][3]);

  jointMatrices.innerHTML = "";
  localMatrices.forEach((matrix, index) => {
    const title = document.createElement("p");
    title.className = "matrix-title";
    title.textContent = `T${index},${index + 1}`;
    const pre = document.createElement("pre");
    pre.textContent = formatMatrix(matrix);
    jointMatrices.append(title, pre);
  });

  drawRobot(frames);
  saveState();
}

function updateAll() {
  renderTable();
  renderSliders();
  updateOutput();
}

function fitCamera() {
  const { frames } = computeChain();
  currentViewScale = calculateViewScale(frames);
  const points = frames.map(framePosition);
  const box = new THREE.Box3().setFromPoints(points);
  const size = box.getSize(new THREE.Vector3()).length() || 3;
  const center = box.getCenter(new THREE.Vector3());
  controls.target.copy(center);
  camera.position.copy(center).add(new THREE.Vector3(size * 0.9 + 1.4, size * 0.75 + 1.2, size * 0.85 + 1.4));
  camera.near = Math.max(0.01, size / 200);
  camera.far = Math.max(100, size * 20);
  camera.updateProjectionMatrix();
  controls.update();
}

function resizeRenderer() {
  const rect = viewer.getBoundingClientRect();
  renderer.setSize(rect.width, rect.height, false);
  camera.aspect = rect.width / Math.max(rect.height, 1);
  camera.updateProjectionMatrix();
}

tableBody.addEventListener("input", (event) => {
  const field = event.target.dataset.field;
  if (!field) return;
  const index = Number(event.target.dataset.index);
  joints[index][field] = field === "type" ? event.target.value : Number(event.target.value);
  if (field === "type") {
    joints[index].min = event.target.value === "revolute" ? -180 : 0;
    joints[index].max = event.target.value === "revolute" ? 180 : 1.5;
    joints[index].value = 0;
  }
  renderSliders();
  updateOutput();
});

sliderList.addEventListener("input", (event) => {
  const index = event.target.dataset.slider;
  if (index === undefined) return;
  joints[Number(index)].value = Number(event.target.value);
  renderTable();
  renderSliders();
  updateOutput();
});

sliderList.addEventListener("click", (event) => {
  const index = event.target.dataset.zero;
  if (index === undefined) return;
  joints[Number(index)].value = 0;
  renderTable();
  renderSliders();
  updateOutput();
});

document.querySelector("#addJointBtn").addEventListener("click", () => {
  joints.push({ type: "revolute", theta: 0, d: 0, a: 0.8, alpha: 0, value: 0, min: -180, max: 180 });
  updateAll();
  fitCamera();
});

document.querySelector("#removeJointBtn").addEventListener("click", () => {
  if (joints.length <= 1) return;
  joints.pop();
  updateAll();
  fitCamera();
});

document.querySelector("#resetExampleBtn").addEventListener("click", () => {
  joints = exampleJoints();
  updateAll();
  fitCamera();
});

document.querySelector("#playBtn").addEventListener("click", (event) => {
  playing = !playing;
  event.target.textContent = playing ? "Pause" : "Play";
});

document.querySelector("#copyBtn").addEventListener("click", async () => {
  await navigator.clipboard.writeText(reportOutput.textContent);
});

document.querySelector("#fitBtn").addEventListener("click", fitCamera);

document.querySelectorAll("[data-view]").forEach((button) => {
  button.addEventListener("click", () => {
    const { frames } = computeChain();
    currentViewScale = calculateViewScale(frames);
    const points = frames.map(framePosition);
    const center = new THREE.Box3().setFromPoints(points).getCenter(new THREE.Vector3());
    controls.target.copy(center);
    const distance = 6;
    if (button.dataset.view === "front") camera.position.set(center.x, center.y + 0.2, center.z + distance);
    if (button.dataset.view === "top") camera.position.set(center.x, center.y + distance, center.z + 0.01);
    if (button.dataset.view === "iso") camera.position.set(center.x + distance, center.y + distance * 0.72, center.z + distance);
    controls.update();
  });
});

dhMode.addEventListener("change", updateOutput);
window.addEventListener("resize", resizeRenderer);

function animate(time) {
  requestAnimationFrame(animate);
  const delta = Math.min((time - lastTick) / 1000, 0.05);
  lastTick = time;

  if (playing) {
    joints.forEach((joint, index) => {
      const speed = joint.type === "revolute" ? 28 + index * 9 : 0.18 + index * 0.03;
      const next = joint.value + speed * delta;
      joint.value = next > joint.max ? joint.min : next;
    });
    renderTable();
    renderSliders();
    updateOutput();
  }

  controls.update();
  renderer.render(scene, camera);
}

updateAll();
resizeRenderer();
fitCamera();
requestAnimationFrame(animate);
