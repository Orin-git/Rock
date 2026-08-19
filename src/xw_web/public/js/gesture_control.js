/**
 * HOLO PILOT — gesture teleop UI + /remote_cmd_vel bridge.
 * Visual: full-stage energy field + holo particle Earth; camera is sensor PIP only.
 */
import {
    FilesetResolver,
    GestureRecognizer,
    DrawingUtils
} from 'https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@0.10.18/+esm';
import * as THREE from 'https://cdn.jsdelivr.net/npm/three@0.160.1/build/three.module.js';

const MODEL_URL =
    'https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/1/gesture_recognizer.task';
const WASM_URL = 'https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@0.10.18/wasm';

const GESTURE_LOST_MS = 220;
/**
 * cmd_vel over HTTPS.
 * Arbiter stale_timeout is 0.4s — identical non-zero twists must refresh sooner,
 * or teleop drops and the chassis ticks stop/start (jerky wheels).
 * Keep rate modest to avoid HTTPS/CPU spam on the SBC.
 */
const PUBLISH_HZ_MS = 100;
/** Idle zero heartbeat; moving cmds use MOVING_HEARTBEAT_MS below. */
const CMD_HEARTBEAT_MS = 800;
/** Must stay under arbiter stale_timeout_sec (0.4). */
const MOVING_HEARTBEAT_MS = 250;
const API_FAIL_WARN_COUNT = 3;
const GESTURE_SCORE_MIN = 0.55;
const POINT_SCORE_MIN = 0.4;
/** Fist stop: time to ramp from full speed down to zero (no hard brake). */
const STOP_RAMP_MS = 800;

const CMD = {
    NONE: 'none',
    FORWARD: 'forward',
    BACKWARD: 'backward',
    TURN_LEFT: 'turn_left',
    TURN_RIGHT: 'turn_right',
    SPIN_CW: 'spin_cw',
    SPIN_CCW: 'spin_ccw',
    STOP: 'stop'
};

const CMD_LABEL = {
    none: '等待手势向量',
    forward: 'THRUST FORWARD',
    backward: 'REVERSE VECTOR',
    turn_left: 'YAW LEFT',
    turn_right: 'YAW RIGHT',
    spin_cw: 'SPIN CLOCKWISE',
    spin_ccw: 'SPIN COUNTER-CLOCKWISE',
    stop: 'SMOOTH BRAKE · 减速停车'
};

const CMD_GESTURE_NAME = {
    none: 'STANDBY',
    forward: '↑ 指上',
    backward: '↓ 指下',
    turn_left: '← 指左',
    turn_right: '→ 指右',
    spin_cw: '张手 CW',
    spin_ccw: 'SPIN CCW',
    stop: '握拳 STOP'
};

const els = {
    webcam: document.getElementById('webcam'),
    overlay: document.getElementById('overlay'),
    stageCanvas: document.getElementById('stageCanvas'),
    avatar3d: document.getElementById('avatar3d'),
    camIdle: document.getElementById('camIdle'),
    camHelp: document.getElementById('camHelp'),
    startBtn: document.getElementById('startBtn'),
    stopBtn: document.getElementById('stopBtn'),
    estopBtn: document.getElementById('estopBtn'),
    rosPill: document.getElementById('rosPill'),
    runPill: document.getElementById('runPill'),
    safetyPill: document.getElementById('safetyPill'),
    gestureName: document.getElementById('gestureName'),
    gestureCmd: document.getElementById('gestureCmd'),
    linVal: document.getElementById('linVal'),
    angVal: document.getElementById('angVal'),
    maxLin: document.getElementById('maxLin'),
    maxAng: document.getElementById('maxAng'),
    maxLinLabel: document.getElementById('maxLinLabel'),
    maxAngLabel: document.getElementById('maxAngLabel'),
    fpsLabel: document.getElementById('fpsLabel')
};

const overlayCtx = els.overlay.getContext('2d');
const stageCtx = els.stageCanvas.getContext('2d');

let gestureRecognizer = null;
let mediaStream = null;
let running = false;
let starting = false;
let animFrameId = null;
let lastVideoTime = -1;
let lastGestureAt = 0;
let currentCmd = CMD.NONE;
let prevCmd = CMD.NONE;
let currentLinear = 0;
let currentAngular = 0;
/** Commanded targets; published twist always ramps toward these (spin/fist stop included). */
let targetLinear = 0;
let targetAngular = 0;
let maxLinear = parseFloat(els.maxLin.value);
let maxAngular = parseFloat(els.maxAng.value);
let cmdVelInterval = null;
let statusPollTimer = null;
let apiReady = false;
let apiFailStreak = 0;
let lastPostedLin = null;
let lastPostedAng = null;
let lastPostAt = 0;
/** Avoid overlapping /api/cmd_vel fetches when the HTTPS bridge is slow. */
let cmdVelInFlight = false;
let pendingTwist = null;
let fpsFrames = 0;
let fpsLastTs = performance.now();

let ambient = [];
let jets = [];
let shocks = [];
let ringAngle = 0;

/** Holo particle planet pose / morph */
let planetPose = {
    yawSpeed: 0,
    glow: 0.4,
    morph: 0, // 0 = earth, 1 = arrow
    scatter: 0 // 0..1 burst amount
};
let planetTarget = { yawSpeed: 0, glow: 0.4, morph: 0, scatter: 0 };
let planetMode = 'earth'; // earth | scatter | arrow
let planetDir = CMD.NONE;
let planetScatterUntil = 0;
let lastFrameTs = performance.now();

// Three.js holo planet
let three = {
    ready: false,
    renderer: null,
    scene: null,
    camera: null,
    root: null,
    points: null,
    ringGlow: null,
    atmos: null,
    clock: null,
    // particle buffers
    home: null,
    arrowBase: null,
    scatter: null,
    target: null,
    current: null,
    colors: null,
    count: 0,
    earthCount: 0
};

function isHttpsGestureHost() {
    return window.location.protocol === 'https:' && /:9443$/.test(window.location.host);
}

function setPill(el, text, cls) {
    el.textContent = text;
    el.className = 'pill' + (cls ? ' ' + cls : '');
}

function updateSpeedLabels() {
    maxLinear = parseFloat(els.maxLin.value);
    maxAngular = parseFloat(els.maxAng.value);
    els.maxLinLabel.textContent = maxLinear.toFixed(2);
    els.maxAngLabel.textContent = maxAngular.toFixed(2);
}

function updateMetricsUi() {
    els.linVal.textContent = currentLinear.toFixed(2) + ' m/s';
    els.angVal.textContent = currentAngular.toFixed(2) + ' rad/s';
    els.gestureName.textContent = CMD_GESTURE_NAME[currentCmd] || 'STANDBY';
    els.gestureCmd.textContent = CMD_LABEL[currentCmd] || '等待手势向量';
}

function showCamHelp(message) {
    const host = window.location.hostname || '192.168.0.189';
    const httpsUrl = 'https://' + host + ':9443/gesture_control.html';
    els.camHelp.classList.add('visible');
    els.camHelp.innerHTML =
        '<strong>摄像头不可用</strong><br>' +
        message +
        '<br><br>请用 HTTPS 打开：<br><code>' +
        httpsUrl +
        '</code>';
}

function hideCamHelp() {
    els.camHelp.classList.remove('visible');
    els.camHelp.innerHTML = '';
}

function syncRunUi() {
    els.startBtn.disabled = running || starting;
    els.stopBtn.disabled = !running && !starting;
    if (running) {
        setPill(els.runPill, '状态 · 运行中', 'running');
        els.camIdle.classList.add('hidden');
    } else if (starting) {
        setPill(els.runPill, '状态 · 启动中', 'warn');
        els.camIdle.classList.add('hidden');
    } else {
        setPill(els.runPill, '状态 · 空闲', '');
        els.camIdle.classList.remove('hidden');
    }
}

function publishTwist(lin, ang) {
    const now = performance.now();
    const sameAsLast = lastPostedLin === lin && lastPostedAng === ang;
    const moving = Math.abs(lin) > 1e-4 || Math.abs(ang) > 1e-4;
    const heartbeat = moving ? MOVING_HEARTBEAT_MS : CMD_HEARTBEAT_MS;
    if (sameAsLast && now - lastPostAt < heartbeat) {
        return;
    }

    // Coalesce while a previous POST is still in flight (common under CPU pressure).
    if (cmdVelInFlight) {
        pendingTwist = { lin: lin, ang: ang };
        return;
    }

    lastPostedLin = lin;
    lastPostedAng = ang;
    lastPostAt = now;
    cmdVelInFlight = true;
    pendingTwist = null;

    fetch('/api/cmd_vel', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ linear_x: lin, angular_z: ang }),
        cache: 'no-store'
    })
        .then(function (res) {
            if (!res.ok) {
                throw new Error('HTTP ' + res.status);
            }
            return res.json().catch(function () { return {}; });
        })
        .then(function () {
            apiFailStreak = 0;
            apiReady = true;
            setPill(els.rosPill, 'ROS · 已连接', 'on');
        })
        .catch(function (err) {
            apiFailStreak += 1;
            console.warn('[gesture] cmd_vel post failed', err);
            if (apiFailStreak >= API_FAIL_WARN_COUNT) {
                apiReady = false;
                setPill(els.rosPill, 'ROS · 接口失败', 'danger');
            }
        })
        .finally(function () {
            cmdVelInFlight = false;
            if (pendingTwist) {
                const next = pendingTwist;
                pendingTwist = null;
                publishTwist(next.lin, next.ang);
            }
        });
}

function publishZero() {
    currentLinear = 0;
    currentAngular = 0;
    targetLinear = 0;
    targetAngular = 0;
    currentCmd = CMD.NONE;
    setAvatarTarget(CMD.NONE);
    publishTwist(0, 0);
    updateMetricsUi();
}

function approachValue(value, target, maxAbs, dtMs) {
    const span = Math.max(Math.abs(maxAbs), 1e-3);
    const step = (span / STOP_RAMP_MS) * dtMs;
    const diff = target - value;
    if (Math.abs(diff) <= step) {
        return target;
    }
    return value + Math.sign(diff) * step;
}

function tickVelocityRamp() {
    const nextLin = approachValue(currentLinear, targetLinear, maxLinear, PUBLISH_HZ_MS);
    const nextAng = approachValue(currentAngular, targetAngular, maxAngular, PUBLISH_HZ_MS);
    if (nextLin !== currentLinear || nextAng !== currentAngular) {
        currentLinear = nextLin;
        currentAngular = nextAng;
        updateMetricsUi();
    } else {
        currentLinear = nextLin;
        currentAngular = nextAng;
    }
    return (
        currentLinear === 0 &&
        currentAngular === 0 &&
        targetLinear === 0 &&
        targetAngular === 0
    );
}

function beginSmoothStop(reasonCmd) {
    targetLinear = 0;
    targetAngular = 0;
    if (currentCmd !== CMD.STOP && currentCmd !== CMD.NONE) {
        prevCmd = reasonCmd || currentCmd;
        currentCmd = CMD.STOP;
        setAvatarTarget(CMD.STOP);
        updateMetricsUi();
    } else if (currentCmd === CMD.NONE && (currentLinear !== 0 || currentAngular !== 0)) {
        currentCmd = CMD.STOP;
        setAvatarTarget(CMD.STOP);
        updateMetricsUi();
    }
}

function startPublisher() {
    if (cmdVelInterval !== null) {
        return;
    }
    cmdVelInterval = setInterval(function () {
        if (!running) {
            return;
        }
        const lost = performance.now() - lastGestureAt > GESTURE_LOST_MS;

        if (lost && currentCmd !== CMD.STOP && currentCmd !== CMD.NONE) {
            // Palm→fist gap or hand away while spinning/moving: ramp, never snap.
            beginSmoothStop(currentCmd);
        } else if (lost && currentCmd === CMD.NONE && (currentLinear !== 0 || currentAngular !== 0)) {
            beginSmoothStop(CMD.NONE);
        }

        const settled = tickVelocityRamp();
        if (currentCmd === CMD.STOP && settled && lost) {
            currentCmd = CMD.NONE;
            prevCmd = CMD.NONE;
            setAvatarTarget(CMD.NONE);
            updateMetricsUi();
        }
        publishTwist(currentLinear, currentAngular);
    }, PUBLISH_HZ_MS);
}

function stopPublisher(sendZero) {
    if (cmdVelInterval !== null) {
        clearInterval(cmdVelInterval);
        cmdVelInterval = null;
    }
    if (sendZero) {
        publishZero();
    }
}

function cmdFromGesture(results) {
    if (!results || !results.gestures || !results.gestures.length) {
        return CMD.NONE;
    }
    const gestures = results.gestures[0];
    if (!gestures || !gestures.length) {
        return CMD.NONE;
    }
    const top = gestures[0];
    const name = top.categoryName || '';
    const score = top.score || 0;

    if (name === 'Open_Palm' && score >= GESTURE_SCORE_MIN) {
        return CMD.SPIN_CW;
    }
    if (name === 'Closed_Fist' && score >= GESTURE_SCORE_MIN) {
        return CMD.STOP;
    }

    const landmarks = results.landmarks && results.landmarks[0];
    if (!landmarks || landmarks.length < 9) {
        return CMD.NONE;
    }

    const pointingLike =
        (name === 'Pointing_Up' && score >= POINT_SCORE_MIN) ||
        (name === 'Victory' && score >= 0.5) ||
        isIndexExtended(landmarks);

    if (!pointingLike) {
        return CMD.NONE;
    }
    return directionFromIndex(landmarks);
}

function isIndexExtended(lm) {
    const tip = lm[8];
    const pip = lm[6];
    const mcp = lm[5];
    const wrist = lm[0];
    const tipDist = dist2(tip, wrist);
    const pipDist = dist2(pip, wrist);
    const mcpDist = dist2(mcp, wrist);
    return tipDist > pipDist * 1.05 && tipDist > mcpDist * 1.15;
}

function dist2(a, b) {
    const dx = a.x - b.x;
    const dy = a.y - b.y;
    return Math.sqrt(dx * dx + dy * dy);
}

function directionFromIndex(lm) {
    const tip = lm[8];
    const mcp = lm[5];
    const dx = (1 - tip.x) - (1 - mcp.x);
    const dy = tip.y - mcp.y;
    const a = (Math.atan2(dy, dx) * 180) / Math.PI;
    if (a > -135 && a <= -45) {
        return CMD.FORWARD;
    }
    if (a > 45 && a <= 135) {
        return CMD.BACKWARD;
    }
    if (a > -45 && a <= 45) {
        return CMD.TURN_RIGHT;
    }
    return CMD.TURN_LEFT;
}

function setAvatarTarget(cmd) {
    const directional =
        cmd === CMD.FORWARD ||
        cmd === CMD.BACKWARD ||
        cmd === CMD.TURN_LEFT ||
        cmd === CMD.TURN_RIGHT;

    if (directional) {
        if (planetDir !== cmd || planetMode === 'earth') {
            planetMode = 'scatter';
            planetScatterUntil = performance.now() + 380;
            refillScatterTargets();
            assignArrowTargets(cmd);
        }
        planetDir = cmd;
        planetTarget = { yawSpeed: 0, glow: 1, morph: 1, scatter: 1 };
    } else if (cmd === CMD.SPIN_CW) {
        planetMode = 'earth';
        planetDir = cmd;
        planetTarget = { yawSpeed: 1.05, glow: 1, morph: 0, scatter: 0 };
    } else if (cmd === CMD.SPIN_CCW) {
        planetMode = 'earth';
        planetDir = cmd;
        planetTarget = { yawSpeed: -1.05, glow: 1, morph: 0, scatter: 0 };
    } else {
        planetMode = 'earth';
        planetDir = CMD.NONE;
        planetTarget = { yawSpeed: 0, glow: 0.4, morph: 0, scatter: 0 };
    }
}

function triggerShock() {
    const w = els.stageCanvas.width;
    const h = els.stageCanvas.height;
    shocks.push({
        x: w * 0.5,
        y: h * 0.5,
        r: 16,
        life: 1
    });
}

function applyCommand(cmd) {
    if (cmd !== prevCmd && cmd !== CMD.NONE) {
        triggerShock();
    }
    prevCmd = cmd;
    currentCmd = cmd;
    lastGestureAt = performance.now();
    setAvatarTarget(cmd);

    switch (cmd) {
        case CMD.FORWARD:
            targetLinear = maxLinear;
            targetAngular = 0;
            break;
        case CMD.BACKWARD:
            targetLinear = -maxLinear;
            targetAngular = 0;
            break;
        case CMD.TURN_LEFT:
            targetLinear = 0;
            targetAngular = maxAngular;
            break;
        case CMD.TURN_RIGHT:
            targetLinear = 0;
            targetAngular = -maxAngular;
            break;
        case CMD.SPIN_CW:
            targetLinear = 0;
            targetAngular = -maxAngular;
            break;
        case CMD.SPIN_CCW:
            targetLinear = 0;
            targetAngular = maxAngular;
            break;
        case CMD.STOP:
            // Keep published twist; ramp angular/linear (incl. spin) down to zero.
            targetLinear = 0;
            targetAngular = 0;
            break;
        default:
            targetLinear = 0;
            targetAngular = 0;
            break;
    }
    updateMetricsUi();
}

function resizeCanvases() {
    const stage = els.stageCanvas.parentElement;
    const sw = stage.clientWidth;
    const sh = stage.clientHeight;
    if (els.stageCanvas.width !== sw || els.stageCanvas.height !== sh) {
        els.stageCanvas.width = sw;
        els.stageCanvas.height = sh;
        seedAmbient();
    }

    const pip = els.overlay.parentElement;
    const pw = pip.clientWidth;
    const ph = pip.clientHeight;
    if (els.overlay.width !== pw || els.overlay.height !== ph) {
        els.overlay.width = pw;
        els.overlay.height = ph;
    }
    resizeBuddy3d();
}

function seedAmbient() {
    const w = els.stageCanvas.width;
    const h = els.stageCanvas.height;
    ambient = [];
    const n = Math.floor((w * h) / 7000);
    for (let i = 0; i < n; i++) {
        ambient.push({
            x: Math.random() * w,
            y: Math.random() * h,
            z: 0.25 + Math.random() * 0.85,
            vx: (Math.random() - 0.5) * 0.35,
            vy: -0.15 - Math.random() * 0.45,
            size: 0.9 + Math.random() * 2.4,
            tw: Math.random() * Math.PI * 2
        });
    }
}

function lerp(a, b, t) {
    return a + (b - a) * t;
}

const EARTH_N = 2400;
const RING_N = 1100;
const PARTICLE_N = EARTH_N + RING_N;

function fibSphere(i, n, r) {
    const y = 1 - (i / (n - 1)) * 2;
    const rad = Math.sqrt(Math.max(0, 1 - y * y));
    const theta = i * 2.399963229728653;
    return [Math.cos(theta) * rad * r, y * r, Math.sin(theta) * rad * r];
}

function sampleArrowLocal(i, n) {
    // unit arrow pointing +X in local space (shaft + head)
    const t = (i + 0.5) / n;
    let x;
    let y;
    let z;
    if (t < 0.62) {
        // shaft
        const u = t / 0.62;
        x = -0.95 + u * 1.15;
        y = (pseudo(i, 1) - 0.5) * 0.28;
        z = (pseudo(i, 2) - 0.5) * 0.22;
    } else {
        // head triangle
        const u = (t - 0.62) / 0.38;
        const width = (1 - u) * 0.72;
        x = 0.2 + u * 1.05;
        y = (pseudo(i, 3) - 0.5) * width;
        z = (pseudo(i, 4) - 0.5) * width * 0.55;
    }
    return [x * 1.15, y * 1.15, z * 1.15];
}

function pseudo(i, salt) {
    const x = Math.sin(i * 127.1 + salt * 311.7) * 43758.5453;
    return x - Math.floor(x);
}

function rotateArrowForCmd(x, y, z, cmd) {
    // map local +X arrow into world direction
    if (cmd === CMD.FORWARD) {
        return [-y, x, z]; // +Y up
    }
    if (cmd === CMD.BACKWARD) {
        return [y, -x, z]; // -Y
    }
    if (cmd === CMD.TURN_LEFT) {
        return [-x, y, z]; // -X
    }
    if (cmd === CMD.TURN_RIGHT) {
        return [x, y, z]; // +X
    }
    return [x, y, z];
}

function refillScatterTargets() {
    if (!three.scatter || !three.home) {
        return;
    }
    const s = three.scatter;
    const h = three.home;
    for (let i = 0; i < three.count; i++) {
        const i3 = i * 3;
        const hx = h[i3];
        const hy = h[i3 + 1];
        const hz = h[i3 + 2];
        const len = Math.sqrt(hx * hx + hy * hy + hz * hz) || 1;
        const blow = 1.8 + Math.random() * 2.4;
        s[i3] = hx / len * blow * (0.7 + Math.random());
        s[i3 + 1] = hy / len * blow * (0.7 + Math.random());
        s[i3 + 2] = hz / len * blow * (0.7 + Math.random());
        // add tangential swirl
        s[i3] += (Math.random() - 0.5) * 1.2;
        s[i3 + 1] += (Math.random() - 0.5) * 1.2;
        s[i3 + 2] += (Math.random() - 0.5) * 1.2;
    }
}

function assignArrowTargets(cmd) {
    if (!three.arrowBase || !three.target) {
        return;
    }
    const base = three.arrowBase;
    const tgt = three.target;
    for (let i = 0; i < three.count; i++) {
        const i3 = i * 3;
        const r = rotateArrowForCmd(base[i3], base[i3 + 1], base[i3 + 2], cmd);
        tgt[i3] = r[0];
        tgt[i3 + 1] = r[1];
        tgt[i3 + 2] = r[2];
    }
}

function buildHoloPlanet() {
    const root = new THREE.Group();
    const home = new Float32Array(PARTICLE_N * 3);
    const arrowBase = new Float32Array(PARTICLE_N * 3);
    const scatter = new Float32Array(PARTICLE_N * 3);
    const target = new Float32Array(PARTICLE_N * 3);
    const current = new Float32Array(PARTICLE_N * 3);
    const colors = new Float32Array(PARTICLE_N * 3);
    const sizes = new Float32Array(PARTICLE_N);

    const earthR = 1.05;
    for (let i = 0; i < EARTH_N; i++) {
        const i3 = i * 3;
        const p = fibSphere(i, EARTH_N, earthR);
        // slight radial noise for holographic depth
        const n = 0.96 + pseudo(i, 5) * 0.08;
        home[i3] = p[0] * n;
        home[i3 + 1] = p[1] * n;
        home[i3 + 2] = p[2] * n;
        current[i3] = home[i3];
        current[i3 + 1] = home[i3 + 1];
        current[i3 + 2] = home[i3 + 2];
        target[i3] = home[i3];
        target[i3 + 1] = home[i3 + 1];
        target[i3 + 2] = home[i3 + 2];

        const lat = Math.asin(Math.max(-1, Math.min(1, p[1] / earthR)));
        const lon = Math.atan2(p[2], p[0]);
        const land = Math.sin(lon * 3.2 + lat * 1.7) * Math.cos(lat * 2.4 + lon) > 0.22;
        if (land) {
            colors[i3] = 0.35 + pseudo(i, 6) * 0.25;
            colors[i3 + 1] = 0.85 + pseudo(i, 7) * 0.15;
            colors[i3 + 2] = 0.75 + pseudo(i, 8) * 0.2;
        } else {
            colors[i3] = 0.15 + pseudo(i, 6) * 0.15;
            colors[i3 + 1] = 0.55 + pseudo(i, 7) * 0.25;
            colors[i3 + 2] = 0.95;
        }
        sizes[i] = 2.2 + pseudo(i, 9) * 2.4;
        const a = sampleArrowLocal(i, EARTH_N);
        arrowBase[i3] = a[0];
        arrowBase[i3 + 1] = a[1];
        arrowBase[i3 + 2] = a[2];
    }

    // planetary rings (tilted particle torus)
    const ringTilt = 0.42;
    for (let i = 0; i < RING_N; i++) {
        const idx = EARTH_N + i;
        const i3 = idx * 3;
        const a = (i / RING_N) * Math.PI * 2 + pseudo(i, 10) * 0.04;
        const rr = 1.55 + pseudo(i, 11) * 0.55;
        let x = Math.cos(a) * rr;
        let y = (pseudo(i, 12) - 0.5) * 0.08;
        let z = Math.sin(a) * rr;
        // tilt around X
        const yt = y * Math.cos(ringTilt) - z * Math.sin(ringTilt);
        const zt = y * Math.sin(ringTilt) + z * Math.cos(ringTilt);
        home[i3] = x;
        home[i3 + 1] = yt;
        home[i3 + 2] = zt;
        current[i3] = x;
        current[i3 + 1] = yt;
        current[i3 + 2] = zt;
        target[i3] = x;
        target[i3 + 1] = yt;
        target[i3 + 2] = zt;
        colors[i3] = 0.55 + pseudo(i, 13) * 0.35;
        colors[i3 + 1] = 0.9;
        colors[i3 + 2] = 1.0;
        sizes[idx] = 1.6 + pseudo(i, 14) * 1.8;
        const ar = sampleArrowLocal(idx, PARTICLE_N);
        arrowBase[i3] = ar[0];
        arrowBase[i3 + 1] = ar[1];
        arrowBase[i3 + 2] = ar[2];
    }

    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(current, 3));
    geo.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    geo.setAttribute('size', new THREE.BufferAttribute(sizes, 1));

    const mat = new THREE.PointsMaterial({
        size: 0.045,
        vertexColors: true,
        transparent: true,
        opacity: 0.92,
        depthWrite: false,
        blending: THREE.AdditiveBlending,
        sizeAttenuation: true
    });
    const points = new THREE.Points(geo, mat);
    root.add(points);

    // soft atmosphere shell
    const atmos = new THREE.Mesh(
        new THREE.SphereGeometry(1.18, 32, 32),
        new THREE.MeshBasicMaterial({
            color: 0x4de8ff,
            transparent: true,
            opacity: 0.07,
            side: THREE.BackSide,
            depthWrite: false
        })
    );
    root.add(atmos);

    // faint ring glow disc
    const ringGlow = new THREE.Mesh(
        new THREE.RingGeometry(1.5, 2.15, 64),
        new THREE.MeshBasicMaterial({
            color: 0x7af0ff,
            transparent: true,
            opacity: 0.12,
            side: THREE.DoubleSide,
            depthWrite: false,
            blending: THREE.AdditiveBlending
        })
    );
    ringGlow.rotation.x = Math.PI / 2 - 0.42;
    root.add(ringGlow);

    // inner core glow
    const core = new THREE.Mesh(
        new THREE.SphereGeometry(0.35, 16, 16),
        new THREE.MeshBasicMaterial({
            color: 0x9df6ff,
            transparent: true,
            opacity: 0.15,
            depthWrite: false,
            blending: THREE.AdditiveBlending
        })
    );
    root.add(core);

    root.position.y = 0.05;
    root.scale.setScalar(0.95);

    three.home = home;
    three.arrowBase = arrowBase;
    three.scatter = scatter;
    three.target = target;
    three.current = current;
    three.colors = colors;
    three.count = PARTICLE_N;
    three.earthCount = EARTH_N;
    refillScatterTargets();

    return { root: root, points: points, ringGlow: ringGlow, atmos: atmos, core: core };
}

function initBuddy3d() {
    if (!els.avatar3d || three.ready) {
        return;
    }
    const canvas = els.avatar3d;
    const renderer = new THREE.WebGLRenderer({
        canvas: canvas,
        alpha: true,
        antialias: true,
        powerPreference: 'high-performance'
    });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.15;
    renderer.setClearColor(0x000000, 0);

    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(36, 1, 0.1, 50);
    camera.position.set(0, 0.35, 5.2);
    camera.lookAt(0, 0.05, 0);

    const hemi = new THREE.HemisphereLight(0xb8e8ff, 0x061018, 0.85);
    scene.add(hemi);
    const key = new THREE.DirectionalLight(0xffffff, 0.55);
    key.position.set(2.2, 3.2, 2.4);
    scene.add(key);
    const fill = new THREE.PointLight(0x4de8ff, 1.6, 12);
    fill.position.set(0, 0.4, 3.2);
    scene.add(fill);

    const planet = buildHoloPlanet();
    scene.add(planet.root);

    // holographic ground bloom
    const ground = new THREE.Mesh(
        new THREE.CircleGeometry(1.6, 64),
        new THREE.MeshBasicMaterial({
            color: 0x4de8ff,
            transparent: true,
            opacity: 0.08,
            depthWrite: false,
            blending: THREE.AdditiveBlending
        })
    );
    ground.rotation.x = -Math.PI / 2;
    ground.position.y = -1.35;
    scene.add(ground);

    three.renderer = renderer;
    three.scene = scene;
    three.camera = camera;
    three.root = planet.root;
    three.points = planet.points;
    three.ringGlow = planet.ringGlow;
    three.atmos = planet.atmos;
    three.clock = new THREE.Clock();
    three.ready = true;
    resizeBuddy3d();
}

function resizeBuddy3d() {
    if (!three.ready) {
        return;
    }
    const parent = els.avatar3d.parentElement;
    const w = parent.clientWidth || 1;
    const h = parent.clientHeight || 1;
    three.renderer.setSize(w, h, false);
    three.camera.aspect = w / h;
    three.camera.updateProjectionMatrix();
}

function updateBuddy3d(dt) {
    if (!three.ready) {
        return;
    }
    const t = 0.12;
    planetPose.yawSpeed = lerp(planetPose.yawSpeed, planetTarget.yawSpeed, t);
    planetPose.glow = lerp(planetPose.glow, planetTarget.glow, t);
    planetPose.morph = lerp(planetPose.morph, planetTarget.morph, 0.08);
    planetPose.scatter = lerp(planetPose.scatter, planetTarget.scatter, 0.14);

    const now = performance.now();
    if (planetMode === 'scatter' && now >= planetScatterUntil) {
        planetMode = 'arrow';
        planetTarget.scatter = 0;
        planetTarget.morph = 1;
    }

    const root = three.root;
    const idleSpin = planetMode === 'earth' && planetDir === CMD.NONE ? 0.18 : 0;
    if (planetMode === 'arrow' || planetMode === 'scatter') {
        // keep arrow screen-aligned
        root.rotation.y = lerp(root.rotation.y, 0, 0.12);
    } else {
        root.rotation.y += (planetPose.yawSpeed + idleSpin) * dt;
    }
    // subtle bob
    root.position.y = 0.05 + Math.sin(now / 1000 * 1.4) * 0.04;

    // show / hide atmosphere & ring disc while morphing to arrow
    if (three.atmos) {
        three.atmos.material.opacity = 0.07 * (1 - planetPose.morph) * (0.7 + planetPose.glow * 0.4);
        three.atmos.visible = planetPose.morph < 0.85;
    }
    if (three.ringGlow) {
        three.ringGlow.material.opacity = 0.12 * (1 - planetPose.morph);
        three.ringGlow.rotation.z += dt * 0.25;
        three.ringGlow.visible = planetPose.morph < 0.85;
    }

    if (three.points && three.points.material) {
        three.points.material.size = 0.04 + planetPose.glow * 0.02;
        three.points.material.opacity = 0.75 + planetPose.glow * 0.22;
    }

    // integrate particle positions toward mode targets
    const cur = three.current;
    const home = three.home;
    const sc = three.scatter;
    const arr = three.target;
    const posAttr = three.points.geometry.attributes.position;
    const speed = planetMode === 'scatter' ? 6.5 : planetMode === 'arrow' ? 4.2 : 3.2;
    const k = 1 - Math.exp(-speed * dt);

    for (let i = 0; i < three.count; i++) {
        const i3 = i * 3;
        let tx;
        let ty;
        let tz;
        if (planetMode === 'scatter') {
            tx = sc[i3];
            ty = sc[i3 + 1];
            tz = sc[i3 + 2];
        } else if (planetMode === 'arrow' || planetPose.morph > 0.05) {
            const m = planetPose.morph;
            tx = home[i3] * (1 - m) + arr[i3] * m;
            ty = home[i3 + 1] * (1 - m) + arr[i3 + 1] * m;
            tz = home[i3 + 2] * (1 - m) + arr[i3 + 2] * m;
            // during early arrow form after scatter, bias toward arrow
            if (planetMode === 'arrow') {
                tx = arr[i3];
                ty = arr[i3 + 1];
                tz = arr[i3 + 2];
            }
        } else {
            tx = home[i3];
            ty = home[i3 + 1];
            tz = home[i3 + 2];
        }
        cur[i3] = cur[i3] + (tx - cur[i3]) * k;
        cur[i3 + 1] = cur[i3 + 1] + (ty - cur[i3 + 1]) * k;
        cur[i3 + 2] = cur[i3 + 2] + (tz - cur[i3 + 2]) * k;
    }
    posAttr.needsUpdate = true;

    three.renderer.render(three.scene, three.camera);
}

function drawPerspectiveFloor(ctx, w, h, t) {
    const horizon = h * 0.58;
    ctx.save();
    const floorGrad = ctx.createLinearGradient(0, horizon, 0, h);
    floorGrad.addColorStop(0, 'rgba(8, 30, 55, 0)');
    floorGrad.addColorStop(0.2, 'rgba(8, 40, 70, 0.35)');
    floorGrad.addColorStop(1, 'rgba(4, 18, 36, 0.85)');
    ctx.fillStyle = floorGrad;
    ctx.fillRect(0, horizon, w, h - horizon);

    ctx.strokeStyle = 'rgba(77, 232, 255, 0.18)';
    ctx.lineWidth = 1;
    const vanishingX = w * 0.5;
    const vanishingY = horizon - 10;
    const scroll = (t * 0.04) % 40;
    for (let i = 0; i < 18; i++) {
        const y = horizon + 12 + i * 22 + scroll * 0.4;
        if (y > h) {
            continue;
        }
        const spread = ((y - horizon) / (h - horizon)) * w * 0.55;
        ctx.globalAlpha = 0.15 + (i / 18) * 0.35;
        ctx.beginPath();
        ctx.moveTo(vanishingX - spread, y);
        ctx.lineTo(vanishingX + spread, y);
        ctx.stroke();
    }
    for (let i = -10; i <= 10; i++) {
        const xEdge = vanishingX + i * (w * 0.055);
        ctx.globalAlpha = 0.12 + Math.abs(i) * 0.01;
        ctx.beginPath();
        ctx.moveTo(vanishingX, vanishingY);
        ctx.lineTo(xEdge, h);
        ctx.stroke();
    }
    ctx.restore();
}

function drawSidePillars(ctx, w, h, t) {
    ctx.save();
    for (let side = 0; side < 2; side++) {
        const x = side === 0 ? w * 0.08 : w * 0.92;
        const pulse = 0.45 + Math.sin(t / 500 + side) * 0.2;
        const grad = ctx.createLinearGradient(x, h * 0.15, x, h * 0.85);
        grad.addColorStop(0, 'rgba(77, 232, 255, 0)');
        grad.addColorStop(0.5, 'rgba(77, 232, 255,' + (0.22 * pulse).toFixed(2) + ')');
        grad.addColorStop(1, 'rgba(77, 232, 255, 0)');
        ctx.fillStyle = grad;
        ctx.fillRect(x - 3, h * 0.12, 6, h * 0.72);

        ctx.strokeStyle = 'rgba(93, 255, 200,' + (0.35 * pulse).toFixed(2) + ')';
        ctx.lineWidth = 1;
        for (let i = 0; i < 6; i++) {
            const y = h * 0.2 + i * h * 0.1 + ((t * 0.03) % (h * 0.1));
            ctx.beginPath();
            ctx.moveTo(x - 18, y);
            ctx.lineTo(x + 18, y);
            ctx.stroke();
        }
    }

    // corner brackets
    ctx.strokeStyle = 'rgba(77, 232, 255, 0.35)';
    ctx.lineWidth = 2;
    const b = 28;
    const inset = 24;
    [
        [inset, inset, 1, 1],
        [w - inset, inset, -1, 1],
        [inset, h - inset, 1, -1],
        [w - inset, h - inset, -1, -1]
    ].forEach(function (c) {
        ctx.beginPath();
        ctx.moveTo(c[0], c[1] + c[3] * b);
        ctx.lineTo(c[0], c[1]);
        ctx.lineTo(c[0] + c[2] * b, c[1]);
        ctx.stroke();
    });
    ctx.restore();
}

function drawHexGrid(ctx, w, h, t) {
    ctx.save();
    ctx.strokeStyle = 'rgba(77, 232, 255, 0.09)';
    ctx.lineWidth = 1;
    const size = 36;
    const offset = (t * 0.025) % size;
    for (let y = -size; y < h * 0.62; y += size * 0.75) {
        for (let x = -size; x < w + size; x += size) {
            const ox = x + ((Math.floor(y / (size * 0.75)) % 2) * size) / 2 + offset;
            const oy = y;
            ctx.globalAlpha = 0.35 + (oy / h) * 0.4;
            ctx.beginPath();
            for (let i = 0; i < 6; i++) {
                const a = (Math.PI / 3) * i;
                const px = ox + Math.cos(a) * size * 0.42;
                const py = oy + Math.sin(a) * size * 0.42;
                if (i === 0) {
                    ctx.moveTo(px, py);
                } else {
                    ctx.lineTo(px, py);
                }
            }
            ctx.closePath();
            ctx.stroke();
        }
    }
    ctx.restore();
}

function drawOrbitRings(ctx, cx, cy, t) {
    ctx.save();
    ctx.translate(cx, cy);
    for (let i = 0; i < 4; i++) {
        const r = 70 + i * 28;
        ctx.strokeStyle = 'rgba(77, 232, 255,' + (0.22 - i * 0.04) + ')';
        ctx.lineWidth = i === 0 ? 2 : 1.2;
        ctx.beginPath();
        ctx.ellipse(0, 10, r * 1.35, r * 0.4, 0, 0, Math.PI * 2);
        ctx.stroke();
        const a = t * (0.5 + i * 0.18) + i * 1.2;
        ctx.fillStyle = i % 2 ? 'rgba(93, 255, 200, 0.9)' : 'rgba(157, 246, 255, 0.9)';
        ctx.beginPath();
        ctx.arc(Math.cos(a) * r * 1.35, 10 + Math.sin(a) * r * 0.4, 2.5, 0, Math.PI * 2);
        ctx.fill();
    }
    ctx.restore();
}

function spawnJets() {
    if (currentCmd === CMD.NONE || currentCmd === CMD.STOP) {
        return;
    }
    const w = els.stageCanvas.width;
    const h = els.stageCanvas.height;
    const cx = w * 0.5;
    const cy = h * 0.5;
    const burst = 10;

    for (let i = 0; i < burst; i++) {
        if (currentCmd === CMD.FORWARD) {
            jets.push({
                x: cx + (Math.random() - 0.5) * 90,
                y: cy + 30 + Math.random() * 40,
                vx: (Math.random() - 0.5) * 1.2,
                vy: -4 - Math.random() * 5,
                life: 1,
                size: 1.8 + Math.random() * 2.8,
                kind: 'jet'
            });
        } else if (currentCmd === CMD.BACKWARD) {
            jets.push({
                x: cx + (Math.random() - 0.5) * 90,
                y: cy - 20,
                vx: (Math.random() - 0.5) * 1.2,
                vy: 3 + Math.random() * 4,
                life: 1,
                size: 1.8 + Math.random() * 2.8,
                kind: 'jet'
            });
        } else if (currentCmd === CMD.TURN_LEFT || currentCmd === CMD.TURN_RIGHT) {
            const dir = currentCmd === CMD.TURN_LEFT ? -1 : 1;
            jets.push({
                x: cx - dir * 20,
                y: cy + (Math.random() - 0.5) * 90,
                vx: dir * (4 + Math.random() * 5),
                vy: (Math.random() - 0.5) * 1.5,
                life: 1,
                size: 1.8 + Math.random() * 3,
                kind: 'jet'
            });
        } else {
            const a = Math.random() * Math.PI * 2;
            const spin = currentCmd === CMD.SPIN_CW ? 1 : -1;
            jets.push({
                x: cx + Math.cos(a) * 80,
                y: cy + Math.sin(a) * 50,
                angle: a,
                radius: 80 + Math.random() * 40,
                spin: spin,
                life: 1,
                size: 1.8 + Math.random() * 2.4,
                kind: 'orbit'
            });
        }
    }
}

function drawEnergyArrow() {
    // directional / spin cues are particle-morph in Three.js now
}

function drawStage() {
    resizeCanvases();
    const w = els.stageCanvas.width;
    const h = els.stageCanvas.height;
    const cx = w * 0.5;
    const cy = h * 0.52;
    const t = performance.now();

    stageCtx.clearRect(0, 0, w, h);

    stageCtx.fillStyle = '#030814';
    stageCtx.fillRect(0, 0, w, h);

    const bg = stageCtx.createRadialGradient(cx, cy, 30, cx, cy, Math.max(w, h) * 0.75);
    bg.addColorStop(0, 'rgba(28, 70, 110, 0.7)');
    bg.addColorStop(0.4, 'rgba(10, 28, 52, 0.55)');
    bg.addColorStop(1, 'rgba(2, 6, 14, 0.2)');
    stageCtx.fillStyle = bg;
    stageCtx.fillRect(0, 0, w, h);

    drawHexGrid(stageCtx, w, h, t);
    drawPerspectiveFloor(stageCtx, w, h, t);
    drawSidePillars(stageCtx, w, h, t);
    drawOrbitRings(stageCtx, cx, cy, t / 1000);

    for (let i = 0; i < ambient.length; i++) {
        const p = ambient[i];
        p.x += p.vx * p.z;
        p.y += p.vy * p.z;
        p.tw += 0.04;
        if (p.x < 0) p.x = w;
        if (p.x > w) p.x = 0;
        if (p.y < 0) p.y = h;
        if (p.y > h) p.y = 0;
        const a = 0.2 + p.z * 0.45 + Math.sin(p.tw) * 0.1;
        stageCtx.fillStyle = 'rgba(150, 230, 255,' + a.toFixed(2) + ')';
        stageCtx.beginPath();
        stageCtx.arc(p.x, p.y, p.size * p.z, 0, Math.PI * 2);
        stageCtx.fill();
    }

    if (running) {
        spawnJets();
    }

    for (let i = shocks.length - 1; i >= 0; i--) {
        const s = shocks[i];
        s.r += 14;
        s.life -= 0.035;
        if (s.life <= 0) {
            shocks.splice(i, 1);
            continue;
        }
        stageCtx.strokeStyle = 'rgba(157, 246, 255,' + (s.life * 0.75).toFixed(2) + ')';
        stageCtx.lineWidth = 4 * s.life;
        stageCtx.beginPath();
        stageCtx.arc(s.x, s.y, s.r, 0, Math.PI * 2);
        stageCtx.stroke();
    }

    for (let i = jets.length - 1; i >= 0; i--) {
        const p = jets[i];
        p.life -= 0.018;
        if (p.kind === 'orbit') {
            p.angle += p.spin * 0.12;
            p.radius += 1.4;
            p.x = cx + Math.cos(p.angle) * p.radius;
            p.y = cy + Math.sin(p.angle) * p.radius * 0.45;
        } else {
            p.x += p.vx;
            p.y += p.vy;
        }
        if (p.life <= 0) {
            jets.splice(i, 1);
            continue;
        }
        stageCtx.fillStyle = 'rgba(77, 232, 255,' + (p.life * 0.95).toFixed(2) + ')';
        stageCtx.beginPath();
        stageCtx.arc(p.x, p.y, p.size * (0.7 + p.life), 0, Math.PI * 2);
        stageCtx.fill();
    }

    ringAngle += 0.005;
    stageCtx.save();
    stageCtx.translate(cx, cy);
    stageCtx.rotate(ringAngle);
    stageCtx.strokeStyle = 'rgba(77, 232, 255, 0.14)';
    for (let i = 0; i < 36; i++) {
        const a = (i / 36) * Math.PI * 2;
        const r0 = 150;
        const r1 = i % 6 === 0 ? 175 : 162;
        stageCtx.lineWidth = i % 6 === 0 ? 2 : 1;
        stageCtx.beginPath();
        stageCtx.moveTo(Math.cos(a) * r0, Math.sin(a) * r0 * 0.42);
        stageCtx.lineTo(Math.cos(a) * r1, Math.sin(a) * r1 * 0.42);
        stageCtx.stroke();
    }
    stageCtx.restore();
}

function drawLandmarks(results) {
    const w = els.overlay.width;
    const h = els.overlay.height;
    overlayCtx.clearRect(0, 0, w, h);

    // darken non-hand area slightly in canvas too
    overlayCtx.fillStyle = 'rgba(2, 8, 16, 0.25)';
    overlayCtx.fillRect(0, 0, w, h);

    if (!results || !results.landmarks || !results.landmarks.length) {
        return;
    }

    // punch clear-ish window around hand then draw bright skeleton
    const lm = results.landmarks[0];
    let minX = 1;
    let minY = 1;
    let maxX = 0;
    let maxY = 0;
    for (let i = 0; i < lm.length; i++) {
        minX = Math.min(minX, lm[i].x);
        minY = Math.min(minY, lm[i].y);
        maxX = Math.max(maxX, lm[i].x);
        maxY = Math.max(maxY, lm[i].y);
    }
    const pad = 0.08;
    overlayCtx.save();
    overlayCtx.globalCompositeOperation = 'destination-out';
    overlayCtx.fillStyle = 'rgba(0,0,0,0.85)';
    overlayCtx.fillRect(
        (minX - pad) * w,
        (minY - pad) * h,
        (maxX - minX + pad * 2) * w,
        (maxY - minY + pad * 2) * h
    );
    overlayCtx.restore();

    const drawingUtils = new DrawingUtils(overlayCtx);
    drawingUtils.drawConnectors(lm, GestureRecognizer.HAND_CONNECTIONS, {
        color: '#5dffc8',
        lineWidth: 3
    });
    drawingUtils.drawLandmarks(lm, {
        color: '#9af6ff',
        fillColor: 'rgba(77, 232, 255, 0.55)',
        lineWidth: 1,
        radius: 3.5
    });
}

function tickFps() {
    fpsFrames += 1;
    const now = performance.now();
    if (now - fpsLastTs >= 1000) {
        els.fpsLabel.textContent = fpsFrames + ' FPS';
        fpsFrames = 0;
        fpsLastTs = now;
    }
}

function recognitionLoop() {
    animFrameId = requestAnimationFrame(recognitionLoop);
    const now = performance.now();
    const dt = Math.min(0.05, (now - lastFrameTs) / 1000);
    lastFrameTs = now;

    drawStage();
    updateBuddy3d(dt);

    if (!running) {
        return;
    }

    const video = els.webcam;
    if (!gestureRecognizer || video.readyState < 2) {
        return;
    }

    if (video.currentTime !== lastVideoTime) {
        lastVideoTime = video.currentTime;
        let results = null;
        try {
            results = gestureRecognizer.recognizeForVideo(video, now);
        } catch (err) {
            console.warn('[gesture] recognize error', err);
        }
        drawLandmarks(results);
        const cmd = cmdFromGesture(results);
        if (cmd !== CMD.NONE) {
            applyCommand(cmd);
        }
        tickFps();
    }
}

async function ensureRecognizer() {
    if (gestureRecognizer) {
        return;
    }
    const vision = await FilesetResolver.forVisionTasks(WASM_URL);
    const options = { runningMode: 'VIDEO', numHands: 1 };
    try {
        gestureRecognizer = await GestureRecognizer.createFromOptions(vision, {
            ...options,
            baseOptions: { modelAssetPath: MODEL_URL, delegate: 'GPU' }
        });
    } catch (gpuErr) {
        console.warn('[gesture] GPU delegate failed, falling back to CPU', gpuErr);
        gestureRecognizer = await GestureRecognizer.createFromOptions(vision, {
            ...options,
            baseOptions: { modelAssetPath: MODEL_URL, delegate: 'CPU' }
        });
    }
}

async function openCamera() {
    if (!window.isSecureContext) {
        throw new Error(
            '当前不是安全上下文（HTTP）。请改用 https://' +
                (window.location.hostname || '机器人IP') +
                ':9443/gesture_control.html'
        );
    }
    if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
        throw new Error('当前浏览器不支持摄像头 API，请用 Chrome / Edge 打开 HTTPS 页面。');
    }
    mediaStream = await navigator.mediaDevices.getUserMedia({
        audio: false,
        video: {
            facingMode: 'user',
            width: { ideal: 960 },
            height: { ideal: 540 }
        }
    });
    els.webcam.srcObject = mediaStream;
    await els.webcam.play();
}

function closeCamera() {
    if (mediaStream) {
        mediaStream.getTracks().forEach(function (t) {
            t.stop();
        });
        mediaStream = null;
    }
    els.webcam.srcObject = null;
    overlayCtx.clearRect(0, 0, els.overlay.width, els.overlay.height);
    jets = [];
    shocks = [];
    lastVideoTime = -1;
    prevCmd = CMD.NONE;
}

async function startGestureControl() {
    if (running || starting) {
        return;
    }
    starting = true;
    syncRunUi();
    hideCamHelp();
    try {
        await ensureRecognizer();
        await openCamera();
        running = true;
        starting = false;
        lastGestureAt = 0;
        applyCommand(CMD.NONE);
        startPublisher();
        syncRunUi();
    } catch (err) {
        console.error('[gesture] start failed', err);
        starting = false;
        running = false;
        closeCamera();
        stopPublisher(true);
        syncRunUi();
        showCamHelp(String(err && err.message ? err.message : err));
    }
}

function stopGestureControl(opts) {
    const sendZero = !opts || opts.sendZero !== false;
    running = false;
    starting = false;
    closeCamera();
    stopPublisher(sendZero);
    lastPostedLin = null;
    lastPostedAng = null;
    lastPostAt = 0;
    pendingTwist = null;
    cmdVelInFlight = false;
    setAvatarTarget(CMD.NONE);
    syncRunUi();
    updateMetricsUi();
    els.fpsLabel.textContent = '0 FPS';
}

function emergencyStop() {
    stopGestureControl({ sendZero: true });
    publishZero();
    setPill(els.runPill, '状态 · 急停', 'danger');
}

function initApiBridge() {
    if (window.location.protocol !== 'https:') {
        setPill(els.rosPill, 'ROS · 请用 HTTPS:9443', 'warn');
        showCamHelp('本页需通过 HTTPS 打开才能使用摄像头与控制接口。');
        return;
    }
    const poll = function () {
        fetch('/api/status', { cache: 'no-store' })
            .then(function (res) {
                if (!res.ok) {
                    throw new Error('status ' + res.status);
                }
                return res.json();
            })
            .then(function (data) {
                apiReady = true;
                setPill(els.rosPill, 'ROS · 已连接', 'on');
                if (data && data.safety_ok === false) {
                    setPill(els.safetyPill, '避障 · 受限', 'warn');
                } else {
                    setPill(els.safetyPill, '避障 · 安全', 'on');
                }
            })
            .catch(function () {
                apiReady = false;
                setPill(els.rosPill, 'ROS · 等待 :9443 服务', 'warn');
                setPill(els.safetyPill, '避障 · —', '');
            });
    };
    poll();
    statusPollTimer = setInterval(poll, 3000);
}

function bindUi() {
    els.startBtn.addEventListener('click', startGestureControl);
    els.stopBtn.addEventListener('click', function () {
        stopGestureControl({ sendZero: true });
    });
    els.estopBtn.addEventListener('click', emergencyStop);
    els.maxLin.addEventListener('input', updateSpeedLabels);
    els.maxAng.addEventListener('input', updateSpeedLabels);

    document.addEventListener('visibilitychange', function () {
        if (document.hidden && running) {
            stopGestureControl({ sendZero: true });
        }
    });
    window.addEventListener('blur', function () {
        if (running) {
            stopGestureControl({ sendZero: true });
        }
    });
    window.addEventListener('beforeunload', function () {
        stopGestureControl({ sendZero: true });
    });
    window.addEventListener('resize', resizeCanvases);
}

bindUi();
updateSpeedLabels();
updateMetricsUi();
syncRunUi();
initApiBridge();
initBuddy3d();
resizeCanvases();
seedAmbient();
recognitionLoop(); // always render holostage + particle Earth

if (!isHttpsGestureHost() && window.location.protocol !== 'https:') {
    showCamHelp('检测到 HTTP 访问。请从遥控页「空挡 · 手势控制」打开（https://IP:9443）。');
}
