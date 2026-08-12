/**
 * Gen2 Q-panda desktop pet (xw_web).
 * Low-CPU eco by default. Driven by /js/api.js from Gen2:
 *   /xw/robot_state (mode + run_mode), /xw/task/* via HTTP bridge.
 * No Gen1 mapping_service topics.
 */
(function (global) {
    'use strict';

    if (global.__DesktopPet) {
        return;
    }

    var PET_ASSET_VER = '20260812-pet9';
    var TASK_LABELS = { 0: '空闲', 1: '导航', 2: '建图', 3: '跟随' };
    var STORAGE_X = 'desktop_pet_shift_x';
    var BUBBLE_MS = 4200;
    var DEDUPE_MS = 1200;
    var WALK_RANGE = 40;
    /* Eco defaults: rare idle motion to keep CPU low on Radxa / tablet */
    var ACTION_MIN_MS = 22000;
    var ACTION_MAX_MS = 42000;
    var RECEIPT_OBSERVER_DEBOUNCE_MS = 220;
    var EXPR_CLASSES = ['expr-idle', 'expr-happy', 'expr-wow', 'expr-sad', 'expr-smirk', 'expr-focus'];
    var ACTION_CLASSES = ['anim-jump', 'anim-head', 'anim-roll', 'anim-wave', 'anim-bounce', 'anim-look', 'anim-speak'];

    var TASK_CATCHPHRASES = {
        0: ['摸鱼中～', '随时待命！', '呼呼睡着了…不对，醒着！', '空闲也要可爱～'],
        1: ['出发咯～', '导航小能手上线！', '路在脚下～', '方向盘交给我啦！'],
        2: ['扫描扫描～', '建图中请勿打扰喵', '激光转起来！', '地图一点点长大～'],
        3: ['跟着你～', '寸步不离！', '跟随模式启动～', '你走我就走！']
    };

    var TAIL_BY_LEVEL = {
        info: ['～', '哦～', '呢', '呀', '喵'],
        success: ['！好耶～', '！搞定啦～', '！棒棒的～', '！赞～'],
        error: ['…先别急', '…我再想想办法', '…没关系', '…深呼吸一下']
    };

    var COMFORT_LINES = [
        '没事的，再试一次就好～',
        '别担心，我还在呢～',
        '小问题而已，加油～',
        '失败也不可怕，我们继续～',
        '抱抱，下次一定成～'
    ];

    var GREETING_LINES = [
        'Q熊猫报到～二代开发者模式，有任务回执我会告诉你哦！',
        '我来啦～点我可看当前能力与开发/量产形态～',
        '桌宠上线～省电模式，不抢CPU～'
    ];

    var reducedMotion =
        typeof matchMedia === 'function' &&
        matchMedia('(prefers-reduced-motion: reduce)').matches;

    /* Always prefer eco unless explicitly disabled: ?pet_eco=0 */
    var ecoMode = true;
    try {
        var q = global.location && global.location.search;
        if (q && /(?:\?|&)pet_eco=0(?:&|$)/.test(q)) ecoMode = false;
        if (q && /(?:\?|&)pet_eco=1(?:&|$)/.test(q)) ecoMode = true;
    } catch (_) { /* noop */ }
    if (reducedMotion) ecoMode = true;

    var state = {
        task: 0,
        production: false,
        expression: 'idle',
        exprTimer: null,
        shiftX: 0,
        facingLeft: false,
        busy: false,
        speakQueue: [],
        speaking: false,
        lastMsg: '',
        lastMsgAt: 0,
        actionTimer: null,
        bubbleTimer: null,
        receiptObserver: null,
        receiptDebounceTimer: null,
        lastReceiptText: '',
        ros: null,
        topics: [],
        root: null,
        els: {},
        greeted: false
    };

    function pick(arr) {
        if (!arr || !arr.length) return '';
        return arr[Math.floor(Math.random() * arr.length)];
    }

    function pandaSvg() {
        return [
            '<svg viewBox="0 0 148 136" xmlns="http://www.w3.org/2000/svg" aria-hidden="true">',
            '  <defs>',
            '    <linearGradient id="petLidarGrad" x1="0" y1="0" x2="0" y2="1">',
            '      <stop offset="0%" stop-color="#38bdf8"/>',
            '      <stop offset="100%" stop-color="#0369a1"/>',
            '    </linearGradient>',
            '    <linearGradient id="petBeamGrad" x1="0" y1="0" x2="1" y2="0">',
            '      <stop offset="0%" stop-color="#22d3ee" stop-opacity="0.55"/>',
            '      <stop offset="100%" stop-color="#22d3ee" stop-opacity="0"/>',
            '    </linearGradient>',
            '    <linearGradient id="petCarGrad" x1="0" y1="0" x2="0" y2="1">',
            '      <stop offset="0%" stop-color="#60a5fa"/>',
            '      <stop offset="100%" stop-color="#2563eb"/>',
            '    </linearGradient>',
            '  </defs>',
            '  <g class="pet-scene">',
            '    <g class="pet-prop-car">',
            '      <ellipse cx="74" cy="126" rx="42" ry="5" fill="rgba(15,23,42,0.14)"/>',
            '      <g class="pet-car-body">',
            '        <path d="M34 108 C36 98 48 92 74 92 C100 92 112 98 114 108 L118 118 C118 122 114 124 110 124 L38 124 C34 124 30 122 30 118 Z" fill="url(#petCarGrad)"/>',
            '        <path d="M48 100 C54 94 64 90 74 90 C84 90 94 94 100 100 L98 108 L50 108 Z" fill="#93c5fd" opacity="0.9"/>',
            '        <rect x="52" y="96" width="14" height="8" rx="2" fill="#e0f2fe" opacity="0.85"/>',
            '        <rect x="82" y="96" width="14" height="8" rx="2" fill="#e0f2fe" opacity="0.85"/>',
            '        <circle cx="44" cy="120" r="3" fill="#fde68a"/>',
            '        <circle cx="104" cy="120" r="3" fill="#fca5a5"/>',
            '        <g class="pet-speed-lines" opacity="0.7">',
            '          <line x1="18" y1="104" x2="28" y2="104" stroke="#94a3b8" stroke-width="2" stroke-linecap="round"/>',
            '          <line x1="14" y1="112" x2="26" y2="112" stroke="#94a3b8" stroke-width="2" stroke-linecap="round"/>',
            '          <line x1="20" y1="120" x2="30" y2="120" stroke="#94a3b8" stroke-width="2" stroke-linecap="round"/>',
            '        </g>',
            '      </g>',
            '      <g class="pet-wheel pet-wheel-l" transform="translate(48 124)"><g class="pet-wheel-spin"><circle cx="0" cy="0" r="8" fill="#1e293b"/><circle cx="0" cy="0" r="3.5" fill="#94a3b8"/><line x1="0" y1="-7" x2="0" y2="7" stroke="#64748b" stroke-width="1.5"/><line x1="-7" y1="0" x2="7" y2="0" stroke="#64748b" stroke-width="1.5"/></g></g>',
            '      <g class="pet-wheel pet-wheel-r" transform="translate(100 124)"><g class="pet-wheel-spin"><circle cx="0" cy="0" r="8" fill="#1e293b"/><circle cx="0" cy="0" r="3.5" fill="#94a3b8"/><line x1="0" y1="-7" x2="0" y2="7" stroke="#64748b" stroke-width="1.5"/><line x1="-7" y1="0" x2="7" y2="0" stroke="#64748b" stroke-width="1.5"/></g></g>',
            '    </g>',
            '    <g class="pet-lidar-beams">',
            '      <g class="pet-beam-r">',
            '        <path d="M0 0 L46 -12 A48 48 0 0 1 46 12 Z" fill="url(#petBeamGrad)"/>',
            '        <path d="M0 0 L40 -24 A48 48 0 0 1 44 -8 Z" fill="#67e8f9" opacity="0.25"/>',
            '        <path d="M0 0 L40 8 A48 48 0 0 1 38 26 Z" fill="#67e8f9" opacity="0.18"/>',
            '      </g>',
            '      <g class="pet-beam-l">',
            '        <path d="M0 0 L-46 -12 A48 48 0 0 0 -46 12 Z" fill="url(#petBeamGrad)"/>',
            '        <path d="M0 0 L-40 -24 A48 48 0 0 0 -44 -8 Z" fill="#67e8f9" opacity="0.25"/>',
            '        <path d="M0 0 L-40 8 A48 48 0 0 0 -38 26 Z" fill="#67e8f9" opacity="0.18"/>',
            '      </g>',
            '    </g>',
            '    <g class="pet-body-group">',
            '      <ellipse class="pet-ground-shadow" cx="74" cy="128" rx="30" ry="4.5" fill="rgba(15,23,42,0.14)"/>',
            '      <ellipse class="pet-tail" cx="106" cy="96" rx="7" ry="5" fill="#1f2937"/>',
            '      <g class="pet-legs pet-legs-stand">',
            '        <g class="pet-leg-l">',
            '          <ellipse cx="58" cy="112" rx="13" ry="11" fill="#1f2937"/>',
            '          <ellipse cx="58" cy="115" rx="7.5" ry="5" fill="#f8fafc"/>',
            '        </g>',
            '        <g class="pet-leg-r">',
            '          <ellipse cx="90" cy="112" rx="13" ry="11" fill="#1f2937"/>',
            '          <ellipse cx="90" cy="115" rx="7.5" ry="5" fill="#f8fafc"/>',
            '        </g>',
            '      </g>',
            '      <g class="pet-walk-dust">',
            '        <circle class="pet-dust-1" cx="48" cy="122" r="2.2" fill="#cbd5e1"/>',
            '        <circle class="pet-dust-2" cx="42" cy="118" r="1.6" fill="#94a3b8"/>',
            '        <circle class="pet-dust-3" cx="100" cy="122" r="2" fill="#cbd5e1"/>',
            '      </g>',
            '      <ellipse cx="74" cy="90" rx="31" ry="27" fill="#f8fafc" stroke="#e2e8f0" stroke-width="1"/>',
            '      <ellipse cx="74" cy="95" rx="17" ry="13" fill="#1f2937"/>',
            '      <g class="pet-arms">',
            '        <ellipse class="pet-arm-l" cx="44" cy="86" rx="11" ry="15" fill="#1f2937" transform="rotate(-20 44 86)"/>',
            '        <ellipse class="pet-arm-r" cx="104" cy="86" rx="11" ry="15" fill="#1f2937" transform="rotate(20 104 86)"/>',
            '      </g>',
            '      <g class="pet-prop-lidar">',
            '        <g class="pet-lidar-hold pet-lidar-hold-r" transform="translate(112 72)">',
            '          <rect x="-3" y="8" width="6" height="16" rx="2" fill="#64748b"/>',
            '          <g class="pet-lidar-unit">',
            '            <rect x="-11" y="-14" width="22" height="26" rx="5" fill="#0f172a"/>',
            '            <rect x="-9" y="-11" width="18" height="4" rx="1.5" fill="url(#petLidarGrad)"/>',
            '            <rect x="-9" y="-5" width="18" height="4" rx="1.5" fill="url(#petLidarGrad)" opacity="0.85"/>',
            '            <rect x="-9" y="1" width="18" height="4" rx="1.5" fill="url(#petLidarGrad)" opacity="0.7"/>',
            '            <rect x="-9" y="7" width="18" height="4" rx="1.5" fill="url(#petLidarGrad)" opacity="0.55"/>',
            '            <circle cx="0" cy="-16" r="3.5" fill="#38bdf8"/>',
            '            <circle cx="0" cy="-16" r="1.6" fill="#e0f2fe"/>',
            '          </g>',
            '        </g>',
            '        <g class="pet-lidar-hold pet-lidar-hold-l" transform="translate(36 72)">',
            '          <rect x="-3" y="8" width="6" height="16" rx="2" fill="#64748b"/>',
            '          <g class="pet-lidar-unit">',
            '            <rect x="-11" y="-14" width="22" height="26" rx="5" fill="#0f172a"/>',
            '            <rect x="-9" y="-11" width="18" height="4" rx="1.5" fill="url(#petLidarGrad)"/>',
            '            <rect x="-9" y="-5" width="18" height="4" rx="1.5" fill="url(#petLidarGrad)" opacity="0.85"/>',
            '            <rect x="-9" y="1" width="18" height="4" rx="1.5" fill="url(#petLidarGrad)" opacity="0.7"/>',
            '            <rect x="-9" y="7" width="18" height="4" rx="1.5" fill="url(#petLidarGrad)" opacity="0.55"/>',
            '            <circle cx="0" cy="-16" r="3.5" fill="#38bdf8"/>',
            '            <circle cx="0" cy="-16" r="1.6" fill="#e0f2fe"/>',
            '          </g>',
            '        </g>',
            '      </g>',
            '      <g class="pet-head-group">',
            '        <ellipse class="pet-ear-l" cx="48" cy="48" rx="15" ry="13" fill="#1f2937"/>',
            '        <ellipse class="pet-ear-r" cx="100" cy="48" rx="15" ry="13" fill="#1f2937"/>',
            '        <ellipse cx="50" cy="50" rx="7" ry="6" fill="#374151"/>',
            '        <ellipse cx="98" cy="50" rx="7" ry="6" fill="#374151"/>',
            '        <ellipse cx="74" cy="63" rx="33" ry="29" fill="#f8fafc" stroke="#e2e8f0" stroke-width="1"/>',
            '        <ellipse class="pet-eye-patch" cx="58" cy="61" rx="12" ry="14" fill="#1f2937"/>',
            '        <ellipse class="pet-eye-patch" cx="90" cy="61" rx="12" ry="14" fill="#1f2937"/>',
            '        <g class="pet-pupils">',
            '          <circle cx="58" cy="62" r="5" fill="#0f172a"/>',
            '          <circle cx="90" cy="62" r="5" fill="#0f172a"/>',
            '          <circle cx="59.8" cy="60.2" r="1.8" fill="#fff"/>',
            '          <circle cx="91.8" cy="60.2" r="1.8" fill="#fff"/>',
            '          <circle cx="56.5" cy="64" r="0.9" fill="#fff" opacity="0.7"/>',
            '          <circle cx="88.5" cy="64" r="0.9" fill="#fff" opacity="0.7"/>',
            '        </g>',
            '        <ellipse class="pet-blush" cx="48" cy="74" rx="7" ry="4" fill="#fda4af" opacity="0.7"/>',
            '        <ellipse class="pet-blush" cx="100" cy="74" rx="7" ry="4" fill="#fda4af" opacity="0.7"/>',
            '        <ellipse cx="74" cy="71" rx="5.5" ry="4" fill="#1f2937"/>',
            '        <ellipse cx="74" cy="72.5" rx="2.2" ry="1.4" fill="#475569"/>',
            '        <path class="pet-mouth-idle" d="M67 81 Q74 86 81 81" fill="none" stroke="#1f2937" stroke-width="2.2" stroke-linecap="round"/>',
            '        <path class="pet-mouth-happy" d="M64 79 Q74 92 84 79" fill="none" stroke="#1f2937" stroke-width="2.4" stroke-linecap="round"/>',
            '        <ellipse class="pet-mouth-wow" cx="74" cy="84" rx="5" ry="6.5" fill="#1f2937"/>',
            '        <path class="pet-mouth-sad" d="M66 86 Q74 80 82 86" fill="none" stroke="#1f2937" stroke-width="2.2" stroke-linecap="round"/>',
            '        <path class="pet-mouth-smirk" d="M68 82 Q76 88 84 80" fill="none" stroke="#1f2937" stroke-width="2.2" stroke-linecap="round"/>',
            '        <g class="pet-heart">',
            '          <path d="M98 42 C98 38 104 38 104 42 C104 38 110 38 110 42 C110 48 104 53 104 53 C104 53 98 48 98 42Z" fill="#fb7185"/>',
            '        </g>',
            '      </g>',
            '    </g>',
            '  </g>',
            '</svg>'
        ].join('');
    }

    function buildDom() {
        var root = document.createElement('div');
        root.className = 'desktop-pet expr-idle task-mode-0';
        root.id = 'desktop-pet';
        root.setAttribute('aria-live', 'polite');
        root.setAttribute('title', 'Q熊猫桌宠（点击查看状态）');

        if (reducedMotion || ecoMode) {
            root.classList.add('no-motion');
        }
        if (ecoMode) {
            root.classList.add('pet-eco');
        }

        root.innerHTML = [
            '<div class="desktop-pet__hit">',
            '  <div class="desktop-pet__bubble level-info" role="status"></div>',
            '  <div class="desktop-pet__stage">',
            '    <div class="desktop-pet__sign mode-dev">开发者</div>',
            '    <div class="desktop-pet__flip">',
            '      <div class="desktop-pet__svg-wrap">' + pandaSvg() + '</div>',
            '    </div>',
            '    <div class="desktop-pet__badge task-0">空闲</div>',
            '  </div>',
            '</div>'
        ].join('');

        document.body.appendChild(root);

        state.root = root;
        state.els = {
            hit: root.querySelector('.desktop-pet__hit'),
            bubble: root.querySelector('.desktop-pet__bubble'),
            sign: root.querySelector('.desktop-pet__sign'),
            badge: root.querySelector('.desktop-pet__badge'),
            flip: root.querySelector('.desktop-pet__flip'),
            svgWrap: root.querySelector('.desktop-pet__svg-wrap')
        };

        var saved = parseFloat(localStorage.getItem(STORAGE_X) || '0');
        if (!isNaN(saved)) {
            setShift(Math.max(-WALK_RANGE, Math.min(WALK_RANGE, saved)), false);
        }

        try {
            sessionStorage.removeItem('desktop_pet_hidden');
        } catch (_) { /* noop */ }
    }

    function setShift(px, animate) {
        state.shiftX = px;
        if (!state.root) return;
        if (!animate || ecoMode) {
            state.root.style.transition = 'none';
            state.root.style.setProperty('--pet-shift', px + 'px');
            void state.root.offsetWidth;
            state.root.style.transition = '';
        } else {
            state.root.style.setProperty('--pet-shift', px + 'px');
        }
    }

    function setExpression(name, holdMs) {
        if (!state.root) return;
        var expr = name || 'idle';
        if (EXPR_CLASSES.indexOf('expr-' + expr) < 0) expr = 'idle';
        state.expression = expr;
        for (var i = 0; i < EXPR_CLASSES.length; i++) {
            state.root.classList.remove(EXPR_CLASSES[i]);
        }
        state.root.classList.add('expr-' + expr);

        if (state.exprTimer) {
            clearTimeout(state.exprTimer);
            state.exprTimer = null;
        }
        if (holdMs && expr !== taskDefaultExpr()) {
            state.exprTimer = setTimeout(function () {
                setExpression(taskDefaultExpr(), 0);
            }, holdMs);
        }
    }

    function taskDefaultExpr() {
        if (state.task === 1) return 'focus';
        if (state.task === 2) return 'smirk';
        if (state.task === 3) return 'happy';
        return 'idle';
    }

    function setRunMode(productionOrCode) {
        /* Gen2: run_mode 0=production 1=developer(default). Also accepts boolean production. */
        var production;
        if (typeof productionOrCode === 'number') {
            production = productionOrCode === 0;
        } else {
            production = !!productionOrCode;
        }
        state.production = production;
        if (!state.els.sign) return;
        state.els.sign.textContent = state.production ? '量产' : '开发者';
        state.els.sign.classList.toggle('mode-prod', state.production);
        state.els.sign.classList.toggle('mode-dev', !state.production);
    }

    function setGen2RunMode(runMode) {
        var code = Number(runMode);
        if (isNaN(code)) code = 1; /* Gen2 default developer */
        setRunMode(code);
    }

    function setTaskMode(code) {
        var n = Number(code);
        if (isNaN(n) || n < 0 || n > 3) n = 0;
        var changed = n !== state.task;
        state.task = n;
        if (!state.root || !state.els.badge) return;

        state.root.classList.remove('task-mode-0', 'task-mode-1', 'task-mode-2', 'task-mode-3');
        state.root.classList.add('task-mode-' + n);

        state.els.badge.textContent = TASK_LABELS[n] || '空闲';
        state.els.badge.className = 'desktop-pet__badge task-' + n;
        if (changed) {
            state.els.badge.classList.remove('is-pulse');
            void state.els.badge.offsetWidth;
            state.els.badge.classList.add('is-pulse');
            setTimeout(function () {
                if (state.els.badge) state.els.badge.classList.remove('is-pulse');
            }, 500);
            setExpression(taskDefaultExpr(), 0);
            showBubble(pick(TASK_CATCHPHRASES[n] || TASK_CATCHPHRASES[0]), 'info');
            if (!reducedMotion) {
                if (n === 1) runAction('bounce', 700, true);
                else if (n === 2) runAction('look', 900, true);
                else if (n === 3) runAction('bounce', 700, true);
                else runAction('wave', 900, true);
            }
        }
    }

    /* Gen2 RobotState.mode: 0 IDLE 1 MAPPING 2 NAVIGATING 3 FOLLOWING 4 FALL */
    function setGen2Mode(mode, modeName) {
        var m = Number(mode);
        var name = String(modeName || '').toUpperCase();
        var mapped = 0;
        if (m === 1 || name.indexOf('MAP') >= 0) mapped = 2;
        else if (m === 2 || name.indexOf('NAV') >= 0) mapped = 1;
        else if (m === 3 || name.indexOf('FOLLOW') >= 0) mapped = 3;
        else mapped = 0;
        setTaskMode(mapped);
    }

    function cuteify(message, level) {
        var msg = String(message || '').trim();
        if (!msg) return '';

        /* strip prior decoration if re-piped */
        msg = msg.replace(/^任务回执[：:]\s*/, '');
        msg = msg.replace(/^结果[：:]\s*/, '');

        var lv = level || 'info';
        var out = msg;

        if (lv === 'error') {
            out = pick(COMFORT_LINES) + '\n' + out + pick(TAIL_BY_LEVEL.error);
        } else if (lv === 'success') {
            var catchOk = pick(TASK_CATCHPHRASES[state.task] || TASK_CATCHPHRASES[0]);
            if (Math.random() < 0.55) {
                out = catchOk + ' ' + out + pick(TAIL_BY_LEVEL.success);
            } else {
                out = out + pick(TAIL_BY_LEVEL.success);
            }
        } else {
            if (Math.random() < 0.35) {
                out = pick(TASK_CATCHPHRASES[state.task] || TASK_CATCHPHRASES[0]) + ' ' + out;
            }
            out = out + pick(TAIL_BY_LEVEL.info);
        }
        return out;
    }

    function showBubble(message, level) {
        if (!message || !state.els.bubble) return;

        var lv = level || 'info';
        var cute = cuteify(message, lv);

        var now = Date.now();
        if (cute === state.lastMsg && now - state.lastMsgAt < DEDUPE_MS) {
            return;
        }
        state.lastMsg = cute;
        state.lastMsgAt = now;

        if (state.speaking) {
            if (state.speakQueue.length < 3) {
                state.speakQueue.push({ message: cute, level: lv, raw: true });
            }
            return;
        }

        speakNow(cute, lv);
    }

    function inferReceiptLevel(text) {
        var msg = String(text || '');
        if (!msg) return 'info';
        /* "停止/关闭" often means intentional success, not failure */
        if (/(失败|错误|异常|超时|未就绪|未连接|断开|拒绝|无法|invalid|error|failed)/i.test(msg)) {
            return 'error';
        }
        if (/(成功|完成|已启动|已停止|已关闭|已刷新|ok|ready|done|success)/i.test(msg)) {
            return 'success';
        }
        return 'info';
    }

    function relayServiceReceipt(rawText) {
        var text = String(rawText || '').replace(/\s+/g, ' ').trim();
        if (!text || text === '等待操作...') return;
        /* ignore noisy loc-health alone unless paired with result change */
        if (/^定位状态/.test(text) && text.indexOf('；') < 0) return;
        if (text === state.lastReceiptText) return;
        state.lastReceiptText = text;
        showBubble(text, inferReceiptLevel(text));
    }

    function speakNow(message, level) {
        state.speaking = true;
        var bubble = state.els.bubble;
        bubble.textContent = message;
        bubble.className = 'desktop-pet__bubble is-visible level-' + (level || 'info');

        if (level === 'success') {
            setExpression('happy', 2800);
        } else if (level === 'error') {
            setExpression('sad', 3200);
        } else {
            setExpression('wow', 2200);
        }

        if (!reducedMotion && state.root) {
            state.root.classList.add('anim-speak');
            setTimeout(function () {
                if (state.root) state.root.classList.remove('anim-speak');
            }, 900);
        }

        if (state.bubbleTimer) clearTimeout(state.bubbleTimer);
        state.bubbleTimer = setTimeout(function () {
            bubble.classList.remove('is-visible');
            state.speaking = false;
            if (state.speakQueue.length) {
                var next = state.speakQueue.shift();
                setTimeout(function () {
                    speakNow(next.message, next.level);
                }, 180);
            }
        }, BUBBLE_MS);
    }

    function announceSelf() {
        var run = state.production ? '量产模式' : '开发者模式';
        var task = TASK_LABELS[state.task] || '空闲';
        var slogan = pick(TASK_CATCHPHRASES[state.task] || TASK_CATCHPHRASES[0]);
        showBubble(slogan + ' 当前：' + task + ' · ' + run, 'info');
    }

    function clearActionClasses() {
        if (!state.root) return;
        for (var i = 0; i < ACTION_CLASSES.length; i++) {
            if (ACTION_CLASSES[i] !== 'anim-speak') {
                state.root.classList.remove(ACTION_CLASSES[i]);
            }
        }
        state.root.classList.remove('is-busy');
        state.busy = false;
    }

    function runAction(name, durationMs, force) {
        if (reducedMotion || document.hidden || !state.root) return;
        if (!force && (state.busy || state.speaking)) return;
        /* eco: only allow short light actions */
        if (ecoMode && !force && name !== 'wave' && name !== 'head' && name !== 'look') {
            name = 'wave';
            durationMs = 800;
        }

        state.busy = true;
        state.root.classList.add('is-busy', 'anim-' + name);

        if (name === 'wave' || name === 'bounce') {
            setExpression('happy', durationMs + 400);
        } else if (name === 'look') {
            setExpression('wow', durationMs + 200);
        } else if (name === 'roll') {
            setExpression('smirk', durationMs + 300);
        }

        setTimeout(function () {
            clearActionClasses();
        }, durationMs);
    }

    function doWalk() {
        if (ecoMode || reducedMotion || document.hidden || !state.root) return;
        if (state.busy || state.speaking) return;

        var target = (Math.random() * 2 - 1) * WALK_RANGE;
        if (Math.abs(target - state.shiftX) < 16) {
            target = state.shiftX >= 0 ? -WALK_RANGE * 0.7 : WALK_RANGE * 0.7;
        }

        state.facingLeft = target > state.shiftX;
        if (state.els.flip) {
            state.els.flip.classList.toggle('is-flipped', state.facingLeft);
        }
        state.busy = true;
        state.root.classList.add('is-busy');
        setExpression('smirk', 2600);
        setShift(target, true);
        localStorage.setItem(STORAGE_X, String(Math.round(target)));

        setTimeout(function () {
            clearActionClasses();
        }, 2300);
    }

    function pickIdleAction() {
        if (reducedMotion || document.hidden) return;
        if (state.busy || state.speaking) return;

        if (ecoMode) {
            /* rare micro-action only */
            if (Math.random() < 0.55) runAction('head', 700);
            else runAction('wave', 800);
            return;
        }

        if (state.task === 1) {
            doWalk();
            return;
        }
        if (state.task === 2) {
            if (Math.random() < 0.45) runAction('look', 1100);
            else runAction('head', 900);
            return;
        }
        if (state.task === 3) {
            doWalk();
            return;
        }

        var r = Math.random();
        if (r < 0.28) {
            doWalk();
        } else if (r < 0.42) {
            runAction('jump', 700);
        } else if (r < 0.55) {
            runAction('wave', 950);
        } else if (r < 0.68) {
            runAction('bounce', 750);
        } else if (r < 0.80) {
            runAction('head', 900);
        } else if (r < 0.90) {
            runAction('look', 1100);
        } else {
            runAction('roll', 1200);
        }
    }

    function scheduleNextAction() {
        if (state.actionTimer) clearTimeout(state.actionTimer);
        if (reducedMotion) return;

        var minMs = ACTION_MIN_MS;
        var maxMs = ACTION_MAX_MS;
        if (!ecoMode) {
            if (state.task === 3) {
                minMs = 8000;
                maxMs = 16000;
            } else if (state.task === 1) {
                minMs = 10000;
                maxMs = 18000;
            } else if (state.task === 2) {
                minMs = 14000;
                maxMs = 24000;
            } else {
                minMs = 12000;
                maxMs = 22000;
            }
        }

        var delay = minMs + Math.random() * (maxMs - minMs);
        state.actionTimer = setTimeout(function () {
            if (!document.hidden) pickIdleAction();
            scheduleNextAction();
        }, delay);
    }

    function onVisibility() {
        if (document.hidden) {
            if (state.actionTimer) {
                clearTimeout(state.actionTimer);
                state.actionTimer = null;
            }
            if (state.root) state.root.classList.add('pet-paused');
        } else {
            if (state.root) state.root.classList.remove('pet-paused');
            scheduleNextAction();
        }
    }

    function bindInteractions() {
        var hit = state.els.hit;
        var dragging = false;
        var moved = false;
        var startX = 0;
        var originShift = 0;

        hit.addEventListener('pointerdown', function (e) {
            dragging = true;
            moved = false;
            startX = e.clientX;
            originShift = state.shiftX;
            hit.setPointerCapture(e.pointerId);
        });

        hit.addEventListener('pointermove', function (e) {
            if (!dragging) return;
            var next = originShift + (e.clientX - startX);
            next = Math.max(-WALK_RANGE * 1.5, Math.min(WALK_RANGE * 1.5, next));
            if (Math.abs(e.clientX - startX) > 4) moved = true;
            setShift(next, false);
        });

        function endDrag(e) {
            if (!dragging) return;
            dragging = false;
            try {
                hit.releasePointerCapture(e.pointerId);
            } catch (_) { /* noop */ }
            localStorage.setItem(STORAGE_X, String(Math.round(state.shiftX)));

            if (!moved) {
                setExpression('happy', 2000);
                if (!reducedMotion) {
                    if (state.task === 1) runAction('bounce', 700, true);
                    else if (state.task === 2) runAction('look', 900, true);
                    else runAction('wave', 950, true);
                }
                announceSelf();
            }
        }

        hit.addEventListener('pointerup', endDrag);
        hit.addEventListener('pointercancel', endDrag);
    }

    function bindServiceReceiptBridge() {
        if (typeof MutationObserver !== 'function') return;

        var targets = [];
        ['result', 'tasklog', 'navLog', 'log'].forEach(function (id) {
            var el = document.getElementById(id);
            if (el) targets.push(el);
        });
        if (!targets.length) return;

        function flushReceiptFromDom() {
            if (state.receiptDebounceTimer) {
                clearTimeout(state.receiptDebounceTimer);
            }
            state.receiptDebounceTimer = setTimeout(function () {
                /* Prefer newest first line of the first mutating log; api.js already feeds pet. */
                for (var i = 0; i < targets.length; i++) {
                    var raw = (targets[i].textContent || '').trim();
                    if (!raw || raw === '等待操作...' || raw === '系统待机…') continue;
                    var first = raw.split('\n')[0].trim();
                    if (first) {
                        relayServiceReceipt(first);
                        return;
                    }
                }
            }, RECEIPT_OBSERVER_DEBOUNCE_MS);
        }

        state.receiptObserver = new MutationObserver(flushReceiptFromDom);
        for (var j = 0; j < targets.length; j++) {
            state.receiptObserver.observe(targets[j], {
                childList: true,
                characterData: true,
                subtree: true
            });
        }
    }

    function connectRos() {
        /* Gen2: state/tasks come from /api via api.js — do not open Gen1 Foxglove topics. */
        return;
    }

    function ensureCss() {
        if (document.querySelector('link[href*="desktop_pet.css"]')) return;
        var link = document.createElement('link');
        link.rel = 'stylesheet';
        link.href = '/css/desktop_pet.css?v=' + PET_ASSET_VER;
        document.head.appendChild(link);
    }

    function greetOnce() {
        if (state.greeted) return;
        state.greeted = true;
        setTimeout(function () {
            showBubble(pick(GREETING_LINES), 'info');
            if (!reducedMotion) runAction('wave', 900, true);
        }, 600);
    }

    function init() {
        if (document.getElementById('desktop-pet')) return;
        if (!document.body) {
            document.addEventListener('DOMContentLoaded', init);
            return;
        }
        ensureCss();
        buildDom();
        bindInteractions();
        bindServiceReceiptBridge();
        connectRos();
        scheduleNextAction();
        document.addEventListener('visibilitychange', onVisibility);
        greetOnce();
        console.info('[DesktopPet] ready (eco=' + ecoMode + ')');
    }

    var api = {
        init: init,
        showBubble: showBubble,
        setTaskMode: setTaskMode,
        setGen2Mode: setGen2Mode,
        setGen2RunMode: setGen2RunMode,
        setRunMode: setRunMode,
        announce: announceSelf,
        eco: function (on) {
            ecoMode = !!on;
            if (state.root) {
                state.root.classList.toggle('pet-eco', ecoMode);
                state.root.classList.toggle('no-motion', ecoMode || reducedMotion);
            }
            scheduleNextAction();
        }
    };

    global.__DesktopPet = api;

    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', init);
    } else {
        init();
    }
})(typeof window !== 'undefined' ? window : globalThis);
