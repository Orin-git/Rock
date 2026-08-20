/**
 * Gen2 desktop pet (xw_web).
 * Character: Live2D Luo Xiaohei (罗小黑) — local model under
 *   /assets/pet/live2d/luoxiaohei/ + L2Dwidget under /js/vendor/.
 * Low-CPU eco by default. Driven by /js/api.js from Gen2:
 *   /xw/robot_state (mode + run_mode), /xw/task/* via HTTP bridge.
 * No Gen1 mapping_service topics.
 *
 * Speech rule: bubble text is ALWAYS short plain Chinese about what
 * the robot is doing. Never show opcodes, key=value, or English jargon.
 */
(function (global) {
    'use strict';

    if (global.__DesktopPet) {
        return;
    }

    var PET_ASSET_VER = '20260820-pet18';
    var L2D_SCRIPT = '/js/vendor/L2Dwidget.min.js?v=' + PET_ASSET_VER;
    var L2D_MODEL = '/assets/pet/live2d/luoxiaohei/luoxiaohei.model.json?v=' + PET_ASSET_VER;
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
    var EXPR_TO_MOTION = {
        idle: 'idle',
        happy: 'happy',
        wow: 'wow',
        sad: 'sad',
        smirk: 'smirk',
        focus: 'focus'
    };
    var PRIORITY_IDLE = 1;
    var PRIORITY_FORCE = 3;
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
        '罗小黑报到～有事我会告诉你哦！',
        '我来啦～点我可看当前状态～',
        '桌宠上线～省电模式，不费电～'
    ];

    var MODE_ZH = {
        '0': '空闲', '1': '建图', '2': '导航', '3': '跟随', '4': '跌倒监测',
        IDLE: '空闲', MAPPING: '建图', NAVIGATING: '导航', FOLLOWING: '跟随', FALL_DETECT: '跌倒监测'
    };

    /**
     * Hard gate for EVERY bubble. Returns short Chinese or '' to silence.
     */
    function toPetSpeech(input) {
        var raw = String(input || '').replace(/\s+/g, ' ').trim();
        if (!raw) return '';

        raw = raw.replace(/[呢呀哦喵～~]+$/u, '').trim();
        raw = raw.replace(/^任务回执[：:]\s*/, '').replace(/^结果[：:]\s*/, '');

        // Pure Chinese already (digits/°/米/标点 OK) — keep
        if (isCleanChinese(raw)) return raw;

        var out = translateTechLine(raw);
        if (out && isCleanChinese(out)) return out;

        // Last resort: strip Latin leftovers; if still Chinese-ish keep, else silence
        out = raw
            .replace(/[A-Za-z][A-Za-z0-9_./-]*/g, ' ')
            .replace(/[=\[\]<>(){}|]+/g, ' ')
            .replace(/\s+/g, ' ')
            .trim();
        if (out && isCleanChinese(out) && out.length >= 2) return out;
        return '';
    }

    function isCleanChinese(s) {
        if (!s) return false;
        if (/[A-Za-z]{2,}/.test(s)) return false;
        if (/[A-Za-z]+\s*=/.test(s) || /=\s*[A-Za-z0-9]/.test(s)) return false;
        if (/[\[\]]/.test(s)) return false;
        return /[\u4e00-\u9fff]/.test(s);
    }

    function modeZh(v) {
        var k = String(v || '').toUpperCase();
        if (MODE_ZH[k]) return MODE_ZH[k];
        if (MODE_ZH[String(v)]) return MODE_ZH[String(v)];
        return '';
    }

    function fmtM(n) {
        var x = Math.abs(Number(n));
        if (isNaN(x)) return '';
        var t = x >= 1 ? x.toFixed(1) : x.toFixed(2);
        return t.replace(/\.0$/, '') + '米';
    }

    function translateTechLine(raw) {
        var m;
        var low = raw.toLowerCase();

        // set_mode IDLE active=0  /  [set_mode] ...  /  >> set_mode 2
        m = raw.match(/set[_\s-]?mode\s*(?:→|->|:)?\s*([A-Za-z_]+|\d+)/i);
        if (m) {
            var mz = modeZh(m[1]);
            return mz ? ('切换到' + mz) : '切换模式了';
        }
        m = raw.match(/\b(IDLE|MAPPING|NAVIGATING|FOLLOWING|FALL_DETECT)\b/i);
        if (m && /active\s*=/i.test(raw)) {
            return '切换到' + (modeZh(m[1]) || '新状态');
        }

        m = raw.match(/run[_\s-]?mode\s*(?:→|->|:)?\s*(量产|开发者|production|developer|\d)/i);
        if (m) {
            var rm = m[1];
            if (rm === '0' || /prod/i.test(rm) || rm === '量产') return '换成量产形态';
            return '换成开发者形态';
        }

        // [progress] motion started / [result] ...
        m = raw.match(/^\[progress\]\s*(\w+)\s+(.+)$/i);
        if (m) return translateTechLine(m[1] + ' ' + m[2]);
        m = raw.match(/^\[result\]\s*(\w+)\s+code=\d+\s*(.*)$/i);
        if (m) return translateTechLine((m[2] || m[1] + ' done').trim());
        m = raw.match(/^\[(\w+)\]\s*(.*)$/);
        if (m) return translateTechLine((m[2] ? m[1] + ' ' + m[2] : m[1]).trim());

        // motion accepted / back / fwd
        m = raw.match(/\(\s*(fwd|back)\s+([\d.]+)\s*m\s*\)/i);
        if (m) return (m[1].toLowerCase() === 'back' ? '往后走 ' : '往前走 ') + fmtM(m[2]);
        if (/\bback\b/i.test(raw) && !/fallback/i.test(raw)) {
            if (/started|accept/i.test(raw)) return '开始往后走';
            return '往后走中';
        }
        if (/\bfwd\b|\bforward\b/i.test(raw)) {
            if (/started|accept/i.test(raw)) return '开始往前走';
            return '往前走中';
        }
        if (/\bturn\b|err_deg/i.test(raw)) return '转身中';
        if (/^motion\b/i.test(raw) || /\bmotion\b/i.test(raw)) {
            if (/start/i.test(raw)) return '开始走动';
            if (/driv/i.test(raw)) return '走动中';
            if (/done|success|ok/i.test(raw)) return '走到啦';
            if (/fail|error|timeout/i.test(raw)) return '没走成';
            return '走动中';
        }

        if (/\bfollow\b/i.test(raw)) {
            if (/off|stop|false|0\b/i.test(raw) && !/start/i.test(raw)) return '不跟了';
            if (/search/i.test(raw)) return '找你中';
            if (/coast|lost/i.test(raw)) return /lost/i.test(raw) ? '找不到人了' : '靠近你';
            return '跟着你';
        }
        if (/\brecharge\b|\bdock\b/i.test(raw)) {
            if (/off|stop|cancel/i.test(raw)) return '回充停了';
            if (/fail/i.test(raw)) return '回充失败';
            if (/success|charg/i.test(raw)) return '充上电了';
            if (/detect/i.test(raw)) return '找充电桩';
            if (/align|lock/i.test(raw)) return '对准充电桩';
            if (/commit|dock/i.test(raw)) return '贴桩中';
            if (/nav/i.test(raw)) return '去充电桩';
            return '正在回充';
        }
        if (/\bnav\b|navigate|goal|patrol/i.test(raw)) {
            if (/cancel/i.test(raw)) return '导航取消了';
            if (/fail|abort/i.test(raw)) return '没走到';
            if (/success|complete|done|succeed/i.test(raw)) return '到啦';
            if (/patrol/i.test(raw)) return /stop/i.test(raw) ? '巡航停了' : '开始巡航';
            if (/goal/i.test(raw)) return '去那边';
            return '正在前往';
        }
        if (/\bslam\b|\bmapping\b|map\b/i.test(raw) && !/map_name/i.test(raw)) {
            if (/save|ok|success/i.test(raw)) return '地图好了';
            if (/delete/i.test(raw)) return '地图删了';
            return '建图中';
        }
        if (/\bwaypoint\b/i.test(raw)) return '航点更新了';
        if (/\bfall\b/i.test(raw)) return /off|false|0\b/i.test(raw) ? '跌倒监测关了' : '跌倒监测开了';
        if (/\bpointcloud\b/i.test(raw)) return /off|false|0\b/i.test(raw) ? '点云关了' : '点云开了';
        if (/\bgesture\b/i.test(raw)) return /stop|off/i.test(raw) ? '手势关了' : '手势开了';
        if (/initialpose|initial.?pose/i.test(raw)) return '位置定好了';
        if (/\bteleop\b/i.test(raw)) return '遥控中';

        if (MODE_ZH[raw.toUpperCase()]) return '切换到' + MODE_ZH[raw.toUpperCase()];

        if (/fail|error|timeout|unavailable|reject|invalid/i.test(low)) return '没成功，再试一次';
        if (/^(ok|done|success|ready)$/i.test(raw)) return '好了';
        if (/busy/i.test(low)) return '正忙着';
        if (/accepted|started|active/i.test(low)) return '开始干活了';

        return '';
    }

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

    function petBodyMarkup() {
        return [
            '<div class="pet-body-group">',
            '  <div class="pet-ground-shadow" aria-hidden="true"></div>',
            '  <div class="pet-live2d-host" id="desktop-pet-l2d-host" aria-hidden="true"></div>',
            '  <div class="pet-heart" aria-hidden="true">♥</div>',
            '  <div class="pet-mode-glow" aria-hidden="true"></div>',
            '</div>'
        ].join('');
    }

    function loadScriptOnce(src) {
        return new Promise(function (resolve, reject) {
            var existing = document.querySelector('script[data-pet-l2d="1"]');
            if (existing && global.L2Dwidget) {
                resolve();
                return;
            }
            var s = document.createElement('script');
            s.src = src;
            s.async = true;
            s.dataset.petL2d = '1';
            s.onload = function () { resolve(); };
            s.onerror = function () { reject(new Error('L2Dwidget load failed')); };
            document.head.appendChild(s);
        });
    }

    function mountLive2DContainer(el) {
        var host = state.els && state.els.l2dHost;
        if (!host || !el) return;
        el.style.setProperty('position', 'absolute');
        el.style.setProperty('left', '0');
        el.style.setProperty('top', '0');
        el.style.setProperty('right', 'auto');
        el.style.setProperty('bottom', 'auto');
        el.style.setProperty('width', '100%');
        el.style.setProperty('height', '100%');
        el.style.setProperty('z-index', '1');
        el.style.setProperty('opacity', '1');
        el.style.setProperty('pointer-events', 'none');
        host.appendChild(el);
        state.l2dReady = true;
    }

    function playPetMotion(expr) {
        var group = EXPR_TO_MOTION[expr] || 'idle';
        var mgr = global.__petL2DMgr;
        if (!mgr || typeof mgr.getModel !== 'function') return;
        var model = mgr.getModel(0);
        if (!model || typeof model.startRandomMotion !== 'function') return;
        try {
            model.startRandomMotion(group, group === 'idle' ? PRIORITY_IDLE : PRIORITY_FORCE);
        } catch (_) { /* noop */ }
    }

    function initLive2D() {
        if (state.l2dInitStarted) return;
        state.l2dInitStarted = true;
        loadScriptOnce(L2D_SCRIPT).then(function () {
            if (!global.L2Dwidget) {
                console.warn('[DesktopPet] L2Dwidget missing');
                return;
            }
            try {
                global.L2Dwidget.on('create-container', mountLive2DContainer);
            } catch (_) { /* noop */ }
            global.L2Dwidget.init({
                model: {
                    jsonPath: L2D_MODEL,
                    scale: 1
                },
                display: {
                    superSample: ecoMode ? 1 : 1.25,
                    width: 168,
                    height: 220,
                    position: 'right',
                    hOffset: 14,
                    vOffset: 12
                },
                mobile: {
                    show: true,
                    scale: 1,
                    motion: !reducedMotion
                },
                name: {
                    canvas: 'desktop-pet-live2d-canvas',
                    div: 'desktop-pet-live2d'
                },
                react: {
                    opacity: 1
                },
                dialog: {
                    enable: false
                }
            });
            /* Motions load async; retry a few times after init. */
            var tries = 0;
            var timer = setInterval(function () {
                tries += 1;
                if (global.__petL2DMgr && global.__petL2DMgr.getModel(0)) {
                    clearInterval(timer);
                    playPetMotion(state.expression || 'idle');
                    return;
                }
                if (tries > 40) clearInterval(timer);
            }, 250);
        }).catch(function (err) {
            console.warn('[DesktopPet] Live2D init failed', err);
        });
    }

    function buildDom() {
        var root = document.createElement('div');
        root.className = 'desktop-pet expr-idle task-mode-0';
        root.id = 'desktop-pet';
        root.setAttribute('aria-live', 'polite');
        root.setAttribute('title', '罗小黑桌宠（点击查看状态）');

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
            '      <div class="desktop-pet__svg-wrap">' + petBodyMarkup() + '</div>',
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
            svgWrap: root.querySelector('.desktop-pet__svg-wrap'),
            l2dHost: root.querySelector('.pet-live2d-host')
        };

        var saved = parseFloat(localStorage.getItem(STORAGE_X) || '0');
        if (!isNaN(saved)) {
            setShift(Math.max(-WALK_RANGE, Math.min(WALK_RANGE, saved)), false);
        }

        try {
            sessionStorage.removeItem('desktop_pet_hidden');
        } catch (_) { /* noop */ }

        initLive2D();
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
            showBubble(pick(TASK_CATCHPHRASES[n] || TASK_CATCHPHRASES[0]), 'info', { cute: true });
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
        var msg = toPetSpeech(message);
        if (!msg) return '';

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
        /* cute tails are Chinese-only; re-gate in case */
        return toPetSpeech(out) || msg;
    }

    function showBubble(message, level, opts) {
        if (!message || !state.els.bubble) return;

        var lv = level || 'info';
        var plain = !opts || opts.plain !== false;
        /* Default plain for all external/status speech; only intentional cute lines opt in. */
        if (opts && opts.cute) plain = false;

        var text = plain ? toPetSpeech(message) : cuteify(message, lv);
        if (!text) return;

        var now = Date.now();
        if (text === state.lastMsg && now - state.lastMsgAt < DEDUPE_MS) {
            return;
        }
        state.lastMsg = text;
        state.lastMsgAt = now;

        if (state.speaking) {
            if (state.speakQueue.length < 3) {
                state.speakQueue.push({ message: text, level: lv, raw: true });
            }
            return;
        }

        speakNow(text, lv);
    }

    function inferReceiptLevel(text) {
        var msg = String(text || '');
        if (!msg) return 'info';
        if (/(失败|错误|异常|超时|未就绪|未连接|断开|拒绝|无法|没成功|没走)/.test(msg)) {
            return 'error';
        }
        if (/(成功|完成|已启动|已停止|已关闭|好了|到啦|走到|开了)/.test(msg)) {
            return 'success';
        }
        return 'info';
    }

    function relayServiceReceipt(rawText) {
        var text = toPetSpeech(rawText);
        if (!text) return;
        if (text === state.lastReceiptText) return;
        state.lastReceiptText = text;
        showBubble(text, inferReceiptLevel(text), { plain: true });
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
        showBubble(slogan + ' 当前：' + task + ' · ' + run, 'info', { cute: true });
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
            showBubble(pick(GREETING_LINES), 'info', { cute: true });
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
        toPetSpeech: toPetSpeech,
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
