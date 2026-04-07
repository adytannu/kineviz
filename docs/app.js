const canvas = document.getElementById("alignment-canvas");
const context = canvas.getContext("2d");

const rollInput = document.getElementById("roll-input");
const pitchInput = document.getElementById("pitch-input");
const yawInput = document.getElementById("yaw-input");
const resetButton = document.getElementById("reset-button");
const matrixReadout = document.getElementById("matrix-readout");
const quaternionReadout = document.getElementById("quaternion-readout");

const rootStyles = getComputedStyle(document.documentElement);
const INS_COLOR = rootStyles.getPropertyValue("--ins").trim();
const HEAD_COLOR = rootStyles.getPropertyValue("--head").trim();
const GRID_COLOR = rootStyles.getPropertyValue("--grid").trim();

const worldRotation = rotationMatrixFromEulerXYZ(
  Math.PI / 2,
  0,
  Math.PI / 2
);

const state = {
  rollDeg: 0,
  pitchDeg: 0,
  yawDeg: 0,
  orbitYaw: -0.16,
  orbitPitch: 0.42,
  orbitDistance: 11.5,
  dragging: false,
  pointerId: null,
  lastPointerX: 0,
  lastPointerY: 0,
};

const meshState = {
  echoShield: {
    status: "loading",
    triangles: [],
    error: null,
  },
};

const deviceDefinitions = [
  {
    kind: "box",
    label: "INS",
    color: INS_COLOR,
    position: [0, 0, 0],
    size: [0.4, 0.4, 0.4],
    rotation: identityMatrix(),
  },
  {
    kind: "mesh",
    meshKey: "echoShield",
    label: "EchoShield",
    color: HEAD_COLOR,
    position: [2, 0, 0],
    fallbackSize: [0.4, 0.4, 0.2],
    rotation: identityMatrix(),
    meshRotation: [
      [1, 0, 0],
      [0, 0, -1],
      [0, 1, 0],
    ],
  },
];

function degToRad(value) {
  return (value * Math.PI) / 180;
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function identityMatrix() {
  return [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
  ];
}

function multiplyMatrices(a, b) {
  const result = identityMatrix();
  for (let row = 0; row < 3; row += 1) {
    for (let column = 0; column < 3; column += 1) {
      result[row][column] = 0;
      for (let index = 0; index < 3; index += 1) {
        result[row][column] += a[row][index] * b[index][column];
      }
    }
  }
  return result;
}

function multiplyMatrixVector(matrix, vector) {
  return [
    matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
    matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
    matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
  ];
}

function addVectors(a, b) {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

function subtractVectors(a, b) {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

function scaleVector(vector, scalar) {
  return [vector[0] * scalar, vector[1] * scalar, vector[2] * scalar];
}

function dot(a, b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function cross(a, b) {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

function vectorLength(vector) {
  return Math.hypot(...vector);
}

function normalize(vector) {
  const length = vectorLength(vector) || 1;
  return vector.map((value) => value / length);
}

function rotationMatrixFromEulerXYZ(x, y, z) {
  const a = Math.cos(x);
  const b = Math.sin(x);
  const c = Math.cos(y);
  const d = Math.sin(y);
  const e = Math.cos(z);
  const f = Math.sin(z);

  return [
    [c * e, -c * f, d],
    [b * d * e + a * f, a * e - b * d * f, -b * c],
    [b * f - a * d * e, b * e + a * d * f, a * c],
  ];
}

function alignmentMatrix(roll, pitch, yaw) {
  const cosRoll = Math.cos(roll);
  const sinRoll = Math.sin(roll);
  const cosPitch = Math.cos(pitch);
  const sinPitch = Math.sin(pitch);
  const cosYaw = Math.cos(yaw);
  const sinYaw = Math.sin(yaw);

  return [
    [
      cosPitch * cosYaw,
      sinYaw * cosPitch,
      -sinPitch,
    ],
    [
      sinPitch * sinRoll * cosYaw - sinYaw * cosRoll,
      sinPitch * sinRoll * sinYaw + cosRoll * cosYaw,
      sinRoll * cosPitch,
    ],
    [
      sinPitch * cosRoll * cosYaw + sinRoll * sinYaw,
      sinPitch * sinYaw * cosRoll - sinRoll * cosYaw,
      cosPitch * cosRoll,
    ],
  ];
}

function quaternionFromMatrix(matrix) {
  const trace = matrix[0][0] + matrix[1][1] + matrix[2][2];
  let w;
  let x;
  let y;
  let z;

  if (trace > 0) {
    const s = Math.sqrt(trace + 1) * 2;
    w = 0.25 * s;
    x = (matrix[2][1] - matrix[1][2]) / s;
    y = (matrix[0][2] - matrix[2][0]) / s;
    z = (matrix[1][0] - matrix[0][1]) / s;
  } else if (matrix[0][0] > matrix[1][1] && matrix[0][0] > matrix[2][2]) {
    const s = Math.sqrt(1 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2;
    w = (matrix[2][1] - matrix[1][2]) / s;
    x = 0.25 * s;
    y = (matrix[0][1] + matrix[1][0]) / s;
    z = (matrix[0][2] + matrix[2][0]) / s;
  } else if (matrix[1][1] > matrix[2][2]) {
    const s = Math.sqrt(1 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2;
    w = (matrix[0][2] - matrix[2][0]) / s;
    x = (matrix[0][1] + matrix[1][0]) / s;
    y = 0.25 * s;
    z = (matrix[1][2] + matrix[2][1]) / s;
  } else {
    const s = Math.sqrt(1 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2;
    w = (matrix[1][0] - matrix[0][1]) / s;
    x = (matrix[0][2] + matrix[2][0]) / s;
    y = (matrix[1][2] + matrix[2][1]) / s;
    z = 0.25 * s;
  }

  return normalize([w, x, y, z]);
}

function blendColor(color, brightness) {
  const hex = color.replace("#", "");
  const fullHex = hex.length === 3
    ? hex.split("").map((chunk) => chunk + chunk).join("")
    : hex;

  const red = parseInt(fullHex.slice(0, 2), 16);
  const green = parseInt(fullHex.slice(2, 4), 16);
  const blue = parseInt(fullHex.slice(4, 6), 16);

  const channel = (value) =>
    Math.round(clamp(value * brightness + 18, 0, 255));

  return `rgb(${channel(red)} ${channel(green)} ${channel(blue)})`;
}

function colorChannels(color) {
  const hex = color.replace("#", "");
  const fullHex = hex.length === 3
    ? hex.split("").map((chunk) => chunk + chunk).join("")
    : hex;

  return [
    parseInt(fullHex.slice(0, 2), 16),
    parseInt(fullHex.slice(2, 4), 16),
    parseInt(fullHex.slice(4, 6), 16),
  ];
}

function rgba(color, alpha) {
  const [red, green, blue] = colorChannels(color);
  return `rgba(${red}, ${green}, ${blue}, ${alpha})`;
}

function resizeCanvas() {
  const pixelRatio = window.devicePixelRatio || 1;
  const bounds = canvas.getBoundingClientRect();
  const width = Math.floor(bounds.width * pixelRatio);
  const height = Math.floor(bounds.height * pixelRatio);

  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }

  context.setTransform(pixelRatio, 0, 0, pixelRatio, 0, 0);
}

function getCamera() {
  const target = [0, -0.5, 1];
  const distance = state.orbitDistance;
  const yaw = state.orbitYaw;
  const pitch = state.orbitPitch;
  const position = [
    target[0] + Math.cos(pitch) * Math.sin(yaw) * distance,
    target[1] + Math.sin(pitch) * distance,
    target[2] + Math.cos(pitch) * Math.cos(yaw) * distance,
  ];

  const forward = normalize(subtractVectors(target, position));
  const right = normalize(cross(forward, [0, 1, 0]));
  const up = normalize(cross(right, forward));

  return {
    position,
    forward,
    right,
    up,
    target,
    fov: degToRad(19),
  };
}

function projectPoint(point, camera, width, height) {
  const relative = subtractVectors(point, camera.position);
  const depth = dot(relative, camera.forward);

  if (depth <= 0.1) {
    return null;
  }

  const x = dot(relative, camera.right);
  const y = dot(relative, camera.up);
  const focal = height / (2 * Math.tan(camera.fov / 2));

  return {
    x: width / 2 + (x * focal) / depth,
    y: height / 2 - (y * focal) / depth,
    depth,
  };
}

function getBoxVertices(size) {
  const halfSize = [size[0] / 2, size[1] / 2, size[2] / 2];
  return [
    [-halfSize[0], -halfSize[1], -halfSize[2]],
    [halfSize[0], -halfSize[1], -halfSize[2]],
    [halfSize[0], halfSize[1], -halfSize[2]],
    [-halfSize[0], halfSize[1], -halfSize[2]],
    [-halfSize[0], -halfSize[1], halfSize[2]],
    [halfSize[0], -halfSize[1], halfSize[2]],
    [halfSize[0], halfSize[1], halfSize[2]],
    [-halfSize[0], halfSize[1], halfSize[2]],
  ];
}

const boxFaces = [
  { indices: [0, 1, 2, 3], normal: [0, 0, -1] },
  { indices: [4, 5, 6, 7], normal: [0, 0, 1] },
  { indices: [0, 4, 7, 3], normal: [-1, 0, 0] },
  { indices: [1, 5, 6, 2], normal: [1, 0, 0] },
  { indices: [0, 1, 5, 4], normal: [0, -1, 0] },
  { indices: [3, 2, 6, 7], normal: [0, 1, 0] },
];

function parseBinaryStl(arrayBuffer) {
  const view = new DataView(arrayBuffer);
  const triangleCount = view.getUint32(80, true);
  const triangles = [];
  const min = [Infinity, Infinity, Infinity];
  const max = [-Infinity, -Infinity, -Infinity];

  for (let index = 0, offset = 84; index < triangleCount; index += 1, offset += 50) {
    const vertices = [];

    for (let vertexIndex = 0; vertexIndex < 3; vertexIndex += 1) {
      const base = offset + 12 + vertexIndex * 12;
      const vertex = [
        view.getFloat32(base, true),
        view.getFloat32(base + 4, true),
        view.getFloat32(base + 8, true),
      ];
      vertices.push(vertex);

      for (let axis = 0; axis < 3; axis += 1) {
        min[axis] = Math.min(min[axis], vertex[axis]);
        max[axis] = Math.max(max[axis], vertex[axis]);
      }
    }

    triangles.push({ vertices });
  }

  const center = min.map((value, axis) => (value + max[axis]) / 2);
  const size = max.map((value, axis) => value - min[axis]);
  const scale = 1.1 / Math.max(...size);

  return triangles.map((triangle) => {
    const vertices = triangle.vertices.map((vertex) =>
      scaleVector(subtractVectors(vertex, center), scale)
    );
    const edgeA = subtractVectors(vertices[1], vertices[0]);
    const edgeB = subtractVectors(vertices[2], vertices[0]);
    const normal = normalize(cross(edgeA, edgeB));
    return { vertices, normal };
  });
}

async function loadEchoShieldMesh() {
  try {
    const response = await fetch("./echoshield.stl");
    if (!response.ok) {
      throw new Error(`HTTP ${response.status}`);
    }

    const triangles = parseBinaryStl(await response.arrayBuffer());
    meshState.echoShield.status = "ready";
    meshState.echoShield.triangles = triangles;
    drawScene();
  } catch (error) {
    meshState.echoShield.status = "error";
    meshState.echoShield.error = error instanceof Error ? error.message : String(error);
    drawScene();
  }
}

function transformBox(vertex, rotation, position) {
  const rotated = multiplyMatrixVector(rotation, vertex);
  const translated = addVectors(rotated, position);
  return multiplyMatrixVector(worldRotation, translated);
}

function getWorldOrigin(position) {
  return multiplyMatrixVector(worldRotation, position);
}

function getMeshSceneRotation(device) {
  return multiplyMatrices(
    device.sceneRotation,
    device.meshRotation || identityMatrix()
  );
}

function getDeviceTransforms() {
  const roll = degToRad(state.rollDeg);
  const pitch = degToRad(state.pitchDeg);
  const yaw = degToRad(state.yawDeg);
  const headRotation = alignmentMatrix(roll, pitch, yaw);

  deviceDefinitions[0].rotation = identityMatrix();
  deviceDefinitions[1].rotation = headRotation;

  return deviceDefinitions.map((device) => {
    const sceneRotation = multiplyMatrices(worldRotation, device.rotation);
    return {
      ...device,
      sceneRotation,
      worldOrigin: getWorldOrigin(device.position),
      transformedVertices: device.kind === "box"
        ? getBoxVertices(device.size).map((vertex) =>
            transformBox(vertex, device.rotation, device.position)
          )
        : null,
    };
  });
}

function drawGrid(camera, width, height) {
  context.save();
  context.lineWidth = 1;

  for (let value = -10; value <= 10; value += 1) {
    const emphasis = value % 5 === 0;
    context.strokeStyle = emphasis ? "rgba(92, 168, 255, 0.38)" : GRID_COLOR;

    const lineA = projectPoint([-10, 0, value], camera, width, height);
    const lineB = projectPoint([10, 0, value], camera, width, height);
    if (lineA && lineB) {
      context.beginPath();
      context.moveTo(lineA.x, lineA.y);
      context.lineTo(lineB.x, lineB.y);
      context.stroke();
    }

    const lineC = projectPoint([value, 0, -10], camera, width, height);
    const lineD = projectPoint([value, 0, 10], camera, width, height);
    if (lineC && lineD) {
      context.beginPath();
      context.moveTo(lineC.x, lineC.y);
      context.lineTo(lineD.x, lineD.y);
      context.stroke();
    }
  }

  context.restore();
}

function drawArrow(start, end, color, camera, width, height, label) {
  const projectedStart = projectPoint(start, camera, width, height);
  const projectedEnd = projectPoint(end, camera, width, height);

  if (!projectedStart || !projectedEnd) {
    return;
  }

  context.strokeStyle = color;
  context.fillStyle = color;
  context.lineWidth = 2;

  context.beginPath();
  context.moveTo(projectedStart.x, projectedStart.y);
  context.lineTo(projectedEnd.x, projectedEnd.y);
  context.stroke();

  const angle = Math.atan2(projectedEnd.y - projectedStart.y, projectedEnd.x - projectedStart.x);
  const arrowSize = 8;

  context.beginPath();
  context.moveTo(projectedEnd.x, projectedEnd.y);
  context.lineTo(
    projectedEnd.x - Math.cos(angle - Math.PI / 6) * arrowSize,
    projectedEnd.y - Math.sin(angle - Math.PI / 6) * arrowSize
  );
  context.lineTo(
    projectedEnd.x - Math.cos(angle + Math.PI / 6) * arrowSize,
    projectedEnd.y - Math.sin(angle + Math.PI / 6) * arrowSize
  );
  context.closePath();
  context.fill();

  context.font = "600 12px IBM Plex Sans, sans-serif";
  context.fillText(label, projectedEnd.x + 6, projectedEnd.y - 6);
}

function drawAxes(device, camera, width, height) {
  const axisLength = 0.9;
  const origin = device.worldOrigin;
  const xAxis = addVectors(origin, multiplyMatrixVector(device.sceneRotation, [axisLength, 0, 0]));
  const yAxis = addVectors(origin, multiplyMatrixVector(device.sceneRotation, [0, axisLength, 0]));
  const zAxis = addVectors(origin, multiplyMatrixVector(device.sceneRotation, [0, 0, axisLength]));

  drawArrow(origin, xAxis, "orange", camera, width, height, "X");
  drawArrow(origin, yAxis, "#4fce7b", camera, width, height, "Y");
  drawArrow(origin, zAxis, "#4b86ff", camera, width, height, "Z");
}

function collectBoxFaces(device, camera, width, height, facesToDraw, light) {
  for (const face of boxFaces) {
    const points = face.indices.map((index) => device.transformedVertices[index]);
    const projected = points.map((point) => projectPoint(point, camera, width, height));
    if (projected.some((point) => point === null)) {
      continue;
    }

    const worldNormal = normalize(multiplyMatrixVector(device.sceneRotation, face.normal));
    const brightness = clamp(dot(worldNormal, light) * 0.45 + 0.7, 0.2, 1.2);
    const avgDepth = projected.reduce((sum, point) => sum + point.depth, 0) / projected.length;

    facesToDraw.push({
      projected,
      fill: blendColor(device.color, brightness),
      stroke: "rgba(222, 235, 246, 0.28)",
      depth: avgDepth,
    });
  }
}

function collectMeshFaces(device, camera, width, height, facesToDraw, light) {
  const mesh = meshState[device.meshKey];
  if (!mesh || mesh.status !== "ready") {
    return;
  }

  const meshSceneRotation = getMeshSceneRotation(device);

  for (const triangle of mesh.triangles) {
    const points = triangle.vertices.map((vertex) =>
      addVectors(
        device.worldOrigin,
        multiplyMatrixVector(meshSceneRotation, vertex)
      )
    );
    const faceCenter = scaleVector(
      addVectors(addVectors(points[0], points[1]), points[2]),
      1 / 3
    );
    const projected = points.map((point) => projectPoint(point, camera, width, height));
    if (projected.some((point) => point === null)) {
      continue;
    }

    const worldNormal = normalize(multiplyMatrixVector(meshSceneRotation, triangle.normal));
    const toCamera = normalize(subtractVectors(camera.position, faceCenter));
    if (dot(worldNormal, toCamera) <= 0) {
      continue;
    }
    const avgDepth = projected.reduce((sum, point) => sum + point.depth, 0) / projected.length;
    const diffuse = clamp(dot(worldNormal, light) * 0.18 + 0.9, 0.82, 1.03);
    const reflection = subtractVectors(
      scaleVector(worldNormal, 2 * dot(worldNormal, light)),
      light
    );
    const specular = Math.pow(clamp(dot(reflection, toCamera), 0, 1), 20) * 0.28;
    const rim = Math.pow(1 - clamp(dot(worldNormal, toCamera), 0, 1), 2) * 0.16;
    const solidFill = blendColor(device.color, diffuse);

    facesToDraw.push({
      projected,
      fill: solidFill,
      stroke: solidFill,
      lineWidth: 1.25,
      highlight: specular + rim,
      depth: avgDepth,
    });
  }
}

function drawGeometry(devices, camera, width, height) {
  const light = normalize([1, 2, 1.6]);
  const facesToDraw = [];

  for (const device of devices) {
    if (device.kind === "box") {
      collectBoxFaces(device, camera, width, height, facesToDraw, light);
      continue;
    }

    collectMeshFaces(device, camera, width, height, facesToDraw, light);

    const mesh = meshState[device.meshKey];
    if (!mesh || mesh.status !== "ready") {
      const fallbackDevice = {
        ...device,
        transformedVertices: getBoxVertices(device.fallbackSize).map((vertex) =>
          transformBox(vertex, device.rotation, device.position)
        ),
      };
      collectBoxFaces(fallbackDevice, camera, width, height, facesToDraw, light);
    }
  }

  facesToDraw.sort((a, b) => b.depth - a.depth);

  for (const face of facesToDraw) {
    context.beginPath();
    context.moveTo(face.projected[0].x, face.projected[0].y);
    for (let index = 1; index < face.projected.length; index += 1) {
      context.lineTo(face.projected[index].x, face.projected[index].y);
    }
    context.closePath();
    context.fillStyle = face.fill;
    context.fill();
    if (face.stroke) {
      context.strokeStyle = face.stroke;
      context.lineWidth = face.lineWidth || 1;
      context.lineJoin = "round";
      context.lineCap = "round";
      context.stroke();
    }
    if (face.highlight) {
      context.fillStyle = rgba("#ffffff", Math.min(face.highlight, 0.22));
      context.fill();
    }
  }
}

function drawScene() {
  resizeCanvas();

  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  const camera = getCamera();
  const devices = getDeviceTransforms();

  context.clearRect(0, 0, width, height);

  const sky = context.createLinearGradient(0, 0, 0, height);
  sky.addColorStop(0, "rgba(19, 41, 61, 0.25)");
  sky.addColorStop(1, "rgba(3, 8, 14, 0.05)");
  context.fillStyle = sky;
  context.fillRect(0, 0, width, height);

  drawGrid(camera, width, height);
  drawGeometry(devices, camera, width, height);
  for (const device of devices) {
    drawAxes(device, camera, width, height);
  }

  context.fillStyle = "rgba(227, 240, 252, 0.75)";
  context.font = "600 13px IBM Plex Sans, sans-serif";
  context.fillText("INS frame", 18, 28);
  context.fillText("EchoShield follows roll/pitch/yaw inputs", 18, 48);

  const mesh = meshState.echoShield;
  if (mesh.status === "loading") {
    context.fillStyle = "rgba(227, 240, 252, 0.58)";
    context.font = "500 12px IBM Plex Sans, sans-serif";
    context.fillText("Loading EchoShield STL...", 18, 68);
  } else if (mesh.status === "error") {
    context.fillStyle = "rgba(255, 180, 180, 0.92)";
    context.font = "500 12px IBM Plex Sans, sans-serif";
    context.fillText(`EchoShield STL failed to load: ${mesh.error}`, 18, 68);
  }
}

function updateReadouts() {
  const matrix = alignmentMatrix(
    degToRad(state.rollDeg),
    degToRad(state.pitchDeg),
    degToRad(state.yawDeg)
  );
  const quaternion = quaternionFromMatrix(matrix);

  matrixReadout.textContent = matrix
    .map((row) => row.map((value) => value.toFixed(3)).join(" "))
    .join("; ");
  quaternionReadout.textContent = `[${quaternion
    .map((value) => value.toFixed(3))
    .join(", ")}]`;
}

function syncInputs() {
  rollInput.value = String(state.rollDeg);
  pitchInput.value = String(state.pitchDeg);
  yawInput.value = String(state.yawDeg);
}

function updateStateFromInputs() {
  state.rollDeg = Number.parseFloat(rollInput.value) || 0;
  state.pitchDeg = Number.parseFloat(pitchInput.value) || 0;
  state.yawDeg = Number.parseFloat(yawInput.value) || 0;
  updateReadouts();
  drawScene();
}

function applyPreset(values) {
  const [roll, pitch, yaw] = values.split(",").map((value) => Number.parseFloat(value));
  state.rollDeg = roll;
  state.pitchDeg = pitch;
  state.yawDeg = yaw;
  syncInputs();
  updateReadouts();
  drawScene();
}

canvas.addEventListener("pointerdown", (event) => {
  state.dragging = true;
  state.pointerId = event.pointerId;
  state.lastPointerX = event.clientX;
  state.lastPointerY = event.clientY;
  canvas.setPointerCapture(event.pointerId);
});

canvas.addEventListener("pointermove", (event) => {
  if (!state.dragging || state.pointerId !== event.pointerId) {
    return;
  }

  const deltaX = event.clientX - state.lastPointerX;
  const deltaY = event.clientY - state.lastPointerY;
  state.lastPointerX = event.clientX;
  state.lastPointerY = event.clientY;

  state.orbitYaw -= deltaX * 0.008;
  state.orbitPitch = clamp(state.orbitPitch - deltaY * 0.008, -1.2, 1.2);
  drawScene();
});

function endDrag(event) {
  if (state.pointerId === event.pointerId) {
    state.dragging = false;
    state.pointerId = null;
    canvas.releasePointerCapture(event.pointerId);
  }
}

canvas.addEventListener("pointerup", endDrag);
canvas.addEventListener("pointercancel", endDrag);

canvas.addEventListener("wheel", (event) => {
  event.preventDefault();
  state.orbitDistance = clamp(state.orbitDistance + event.deltaY * 0.01, 6, 18);
  drawScene();
}, { passive: false });

rollInput.addEventListener("input", updateStateFromInputs);
pitchInput.addEventListener("input", updateStateFromInputs);
yawInput.addEventListener("input", updateStateFromInputs);

resetButton.addEventListener("click", () => {
  applyPreset("0,0,0");
  state.orbitYaw = -0.16;
  state.orbitPitch = 0.42;
  state.orbitDistance = 11.5;
  drawScene();
});

window.addEventListener("resize", drawScene);

updateReadouts();
drawScene();
loadEchoShieldMesh();
