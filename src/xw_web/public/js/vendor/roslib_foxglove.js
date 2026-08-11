/**
 * ROSLIB-compatible API backed by Foxglove WebSocket protocol (ROS 2 CDR).
 * Requires: foxglove_bundle.js (FoxgloveLibs), ros2_schemas.js (ROS2_SCHEMAS)
 */
(function (global) {
    const LOG = '[Foxglove Mig]';
    const { FoxgloveClient, MessageReader, MessageWriter, parseMessageDefinition } = FoxgloveLibs;
    // foxglove_bridge >= 3.2 (Foxglove SDK) requires foxglove.sdk.v1.
    // Older bridges used foxglove.websocket.v1 — offer both for compatibility.
    const FOXGLOVE_WS_SUBPROTOCOLS = ['foxglove.sdk.v1', FoxgloveClient.SUPPORTED_SUBPROTOCOL];

    function log(...args) {
        console.log(LOG, ...args);
    }

    function warn(...args) {
        console.warn(LOG, ...args);
    }

    function normalizeTypeName(typeName) {
        if (global.ROS2_SCHEMAS && global.ROS2_SCHEMAS.normalizeTypeName) {
            return global.ROS2_SCHEMAS.normalizeTypeName(typeName);
        }
        if (!typeName) {
            return typeName;
        }
        const t = String(typeName).trim();
        if (t.includes('/msg/') || t.includes('/srv/')) {
            return t;
        }
        const slash = t.indexOf('/');
        if (slash < 0) {
            return t;
        }
        return `${t.slice(0, slash)}/msg/${t.slice(slash + 1)}`;
    }

    function normalizeServiceType(serviceType) {
        if (!serviceType) {
            return serviceType;
        }
        let t = String(serviceType).trim();
        if (t.includes('/srv/')) {
            return t;
        }
        const slash = t.indexOf('/');
        if (slash < 0) {
            return t;
        }
        const pkg = t.slice(0, slash);
        const rest = t.slice(slash + 1);
        if (rest.startsWith('srv/')) {
            return `${pkg}/${rest}`;
        }
        return `${pkg}/srv/${rest}`;
    }

    function serviceRequestSchema(serviceType) {
        const base = normalizeServiceType(serviceType);
        return `${base}_Request`;
    }

    function serviceResponseSchema(serviceType) {
        const base = normalizeServiceType(serviceType);
        return `${base}_Response`;
    }

    function lookupFallbackSchema(schemaName) {
        if (global.ROS2_SCHEMAS && global.ROS2_SCHEMAS.lookup) {
            return global.ROS2_SCHEMAS.lookup(schemaName);
        }
        return null;
    }

    function resolveSchemaText(schemaName, schemaText) {
        if (schemaText) {
            return schemaText;
        }
        return lookupFallbackSchema(schemaName);
    }

    const codecCache = new Map();

    function getCodec(schemaName, schemaText) {
        const text = resolveSchemaText(schemaName, schemaText);
        if (!text) {
            return null;
        }
        const key = `${schemaName}::${text}`;
        if (codecCache.has(key)) {
            return codecCache.get(key);
        }
        const definitions = parseMessageDefinition(text, { ros2: true });
        const codec = {
            definitions: definitions,
            reader: new MessageReader(definitions),
            writer: new MessageWriter(definitions),
        };
        codecCache.set(key, codec);
        return codec;
    }

    function unwrapMessage(message) {
        if (!message) {
            return message;
        }
        if (typeof message === 'object' && message.values && typeof message.values === 'object') {
            return { ...message.values };
        }
        return message;
    }

    class Ros {
        constructor() {
            this.isConnected = false;
            this._url = null;
            this._client = null;
            this._ws = null;
            this._listeners = { connection: [], error: [], close: [] };
            this._topicListeners = new Map();
            this._channelsByTopic = new Map();
            this._channelById = new Map();
            this._servicesByName = new Map();
            this._subscriptions = new Map();
            this._subIdToTopic = new Map();
            this._pendingTopics = new Map();
            this._clientChannels = new Map();
            this._serviceCalls = new Map();
            this._pendingServiceCalls = new Map();
            this._serviceReadyWaiters = new Map();
            this._nextCallId = 1;
            this._onConnectionQueue = [];
            this.socket = {
                readyState: WebSocket.CLOSED,
                addEventListener: () => {},
                removeEventListener: () => {},
                send: () => {
                    warn('ros.socket.send is deprecated; use ROSLIB.Topic.publish');
                },
            };
        }

        connect(url) {
            this._url = url;
            if (this._ws) {
                try {
                    this._ws.close();
                } catch (e) {
                    /* ignore */
                }
            }

            log('connecting', url, 'subprotocols=', FOXGLOVE_WS_SUBPROTOCOLS);
            this._ws = new WebSocket(url, FOXGLOVE_WS_SUBPROTOCOLS);
            this.socket.readyState = WebSocket.CONNECTING;
            this._client = new FoxgloveClient({ ws: this._ws });

            this._client.on('open', () => {
                log('websocket open', url);
            });

            this._client.on('serverInfo', (info) => {
                log('serverInfo', info && info.name, info && info.capabilities);
            });

            this._client.on('advertise', (channels) => {
                channels.forEach((ch) => this._registerChannel(ch));
            });

            this._client.on('unadvertise', (channelIds) => {
                channelIds.forEach((id) => {
                    const ch = this._channelById.get(id);
                    if (ch) {
                        this._channelsByTopic.delete(ch.topic);
                        this._channelById.delete(id);
                    }
                });
            });

            this._client.on('advertiseServices', (services) => {
                services.forEach((svc) => {
                    this._servicesByName.set(svc.name, svc);
                    log('service advertised', svc.name);
                    this._flushPendingServiceCalls(svc.name);
                    this._notifyServiceReady(svc.name);
                });
            });

            this._client.on('unadvertiseServices', (serviceIds) => {
                serviceIds.forEach((id) => {
                    for (const [name, svc] of this._servicesByName.entries()) {
                        if (svc.id === id) {
                            this._servicesByName.delete(name);
                        }
                    }
                });
            });

            this._client.on('message', (event) => {
                const subId = event.subscriptionId;
                const sub = this._subscriptions.get(subId);
                if (!sub) {
                    return;
                }
                try {
                    const payload = sub.codec.reader.readMessage(event.data);
                    const msg = unwrapMessage(payload);
                    if (sub.topic === '/map') {
                        log('received /map', getMapByteSize(msg));
                    } else if (
                        sub.topic === '/navigation_status_code' ||
                        sub.topic === '/navigation_final_result'
                    ) {
                        const v = msg && msg.data !== undefined ? msg.data : msg;
                        log('received', sub.topic, 'data=', v);
                    }
                    sub.callbacks.forEach((cb) => {
                        try {
                            cb(msg);
                        } catch (err) {
                            warn('topic callback error', sub.topic, err);
                        }
                    });
                    const topicHandlers = this._topicListeners.get(sub.topic);
                    if (topicHandlers) {
                        topicHandlers.forEach((cb) => {
                            try {
                                cb(msg);
                            } catch (err) {
                                warn('topic listener error', sub.topic, err);
                            }
                        });
                    }
                } catch (err) {
                    warn('decode failed', sub.topic, err);
                }
            });

            this._client.on('serviceCallResponse', (event) => {
                const pending = this._serviceCalls.get(event.callId);
                if (!pending) {
                    return;
                }
                this._serviceCalls.delete(event.callId);
                try {
                    const response = pending.codec.reader.readMessage(event.data);
                    pending.success(unwrapMessage(response));
                } catch (err) {
                    if (pending.fail) {
                        pending.fail(err);
                    } else {
                        warn('service response decode failed', pending.serviceName, err);
                    }
                }
            });

            this._client.on('serviceCallFailure', (event) => {
                const pending = this._serviceCalls.get(event.callId);
                if (!pending) {
                    return;
                }
                this._serviceCalls.delete(event.callId);
                const err = new Error(event.message || 'service call failed');
                if (pending.fail) {
                    pending.fail(err);
                } else {
                    warn('service call failure', pending.serviceName, err);
                }
            });

            this._client.on('error', (error) => {
                this._emit('error', error);
            });

            this._client.on('close', (event) => {
                this.isConnected = false;
                this.socket.readyState = WebSocket.CLOSED;
                log('connection closed', event && event.code);
                this._emit('close', event);
            });

            this._ws.addEventListener('open', () => {
                this.isConnected = true;
                this.socket.readyState = WebSocket.OPEN;
                log('connected', url);
                const queue = this._onConnectionQueue.slice();
                this._onConnectionQueue = [];
                queue.forEach((fn) => {
                    try {
                        fn();
                    } catch (e) {
                        warn('onConnection queue item failed', e);
                    }
                });
                this._emit('connection');
            });
        }

        _emit(name, arg) {
            const list = this._listeners[name] || [];
            list.forEach((cb) => {
                try {
                    cb(arg);
                } catch (e) {
                    warn('listener error', name, e);
                }
            });
        }

        on(name, cb) {
            if (name === 'connection' || name === 'error' || name === 'close') {
                this._listeners[name].push(cb);
                return;
            }
            if (!this._topicListeners.has(name)) {
                this._topicListeners.set(name, new Set());
            }
            this._topicListeners.get(name).add(cb);
        }

        off(name, cb) {
            if (name === 'connection' || name === 'error' || name === 'close') {
                this._listeners[name] = (this._listeners[name] || []).filter((x) => x !== cb);
                return;
            }
            const set = this._topicListeners.get(name);
            if (set) {
                set.delete(cb);
            }
        }

        close() {
            if (this._client) {
                this._client.close();
            }
            this.isConnected = false;
            this.socket.readyState = WebSocket.CLOSED;
        }

        callOnConnection(fn) {
            if (this.isConnected) {
                fn();
            } else {
                this._onConnectionQueue.push(fn);
            }
        }

        _registerChannel(channel) {
            this._channelById.set(channel.id, channel);
            this._channelsByTopic.set(channel.topic, channel);
            log('channel advertised', channel.topic, channel.schemaName);
            this._trySubscribeTopic(channel.topic);
        }

        _trySubscribeTopic(topicName) {
            const pending = this._pendingTopics.get(topicName);
            if (!pending || pending.subscribed) {
                return;
            }
            const channel = this._channelsByTopic.get(topicName);
            if (!channel) {
                return;
            }
            const schemaName = channel.schemaName || pending.messageType;
            const codec = getCodec(schemaName, channel.schema);
            if (!codec) {
                warn('no schema for topic', topicName, schemaName);
                return;
            }
            const subId = this._client.subscribe(channel.id);
            this._subscriptions.set(subId, {
                topic: topicName,
                callbacks: pending.callbacks,
                codec: codec,
            });
            this._subIdToTopic.set(subId, topicName);
            pending.subscribed = true;
            pending.subscriptionId = subId;
            log('subscribed', topicName, 'channelId=', channel.id);
        }

        _ensureClientChannel(topic, messageType) {
            const existing = this._clientChannels.get(topic);
            if (existing) {
                return existing;
            }
            const schemaName = normalizeTypeName(messageType);
            const schema = lookupFallbackSchema(schemaName);
            if (!schema) {
                warn('cannot advertise without schema', topic, schemaName);
                return null;
            }
            const channelId = this._client.advertise({
                topic: topic,
                encoding: 'cdr',
                schemaName: schemaName,
                schema: schema,
                schemaEncoding: 'ros2msg',
            });
            const record = { channelId: channelId, schemaName: schemaName, schema: schema };
            this._clientChannels.set(topic, record);
            log('client advertised', topic, schemaName);
            return record;
        }

        _publish(topic, messageType, msg) {
            if (!this.isConnected || !this._client) {
                warn('publish skipped, not connected', topic);
                return false;
            }
            const record = this._ensureClientChannel(topic, messageType);
            if (!record) {
                return false;
            }
            const codec = getCodec(record.schemaName, record.schema);
            if (!codec) {
                return false;
            }
            const data = codec.writer.writeMessage(unwrapMessage(msg));
            this._client.sendMessage(record.channelId, data);
            log('published', topic);
            return true;
        }

        _subscribe(topicObj, callback) {
            const topic = topicObj.name;
            if (!this._pendingTopics.has(topic)) {
                this._pendingTopics.set(topic, {
                    messageType: topicObj.messageType,
                    callbacks: new Set(),
                    subscribed: false,
                });
            }
            const pending = this._pendingTopics.get(topic);
            pending.callbacks.add(callback);
            this._trySubscribeTopic(topic);
        }

        _unsubscribe(topicObj, callback) {
            const topic = topicObj.name;
            const pending = this._pendingTopics.get(topic);
            if (!pending) {
                return;
            }
            if (callback) {
                pending.callbacks.delete(callback);
            } else {
                pending.callbacks.clear();
            }
            if (pending.callbacks.size === 0 && pending.subscriptionId != null) {
                this._client.unsubscribe(pending.subscriptionId);
                this._subscriptions.delete(pending.subscriptionId);
                this._subIdToTopic.delete(pending.subscriptionId);
                pending.subscribed = false;
                pending.subscriptionId = null;
            }
        }

        _notifyServiceReady(serviceName) {
            const waiters = this._serviceReadyWaiters.get(serviceName);
            if (!waiters || waiters.length === 0) {
                return;
            }
            this._serviceReadyWaiters.delete(serviceName);
            waiters.forEach((entry) => {
                clearTimeout(entry.timerId);
                try {
                    entry.callback();
                } catch (e) {
                    warn('service ready callback error', serviceName, e);
                }
            });
        }

        _flushPendingServiceCalls(serviceName) {
            const queue = this._pendingServiceCalls.get(serviceName);
            if (!queue || queue.length === 0) {
                return;
            }
            this._pendingServiceCalls.delete(serviceName);
            log('flushing queued service calls', serviceName, 'count=', queue.length);
            queue.forEach((item) => {
                this._callService(item.serviceObj, item.request, item.successCb, item.failCb, true);
            });
        }

        whenServiceReady(serviceName, callback, options) {
            const opts = options || {};
            const timeoutMs = opts.timeoutMs != null ? opts.timeoutMs : 60000;
            if (this._servicesByName.has(serviceName)) {
                callback();
                return;
            }
            if (!this._serviceReadyWaiters.has(serviceName)) {
                this._serviceReadyWaiters.set(serviceName, []);
            }
            const entry = {
                callback: callback,
                timerId: setTimeout(() => {
                    const list = this._serviceReadyWaiters.get(serviceName) || [];
                    const idx = list.indexOf(entry);
                    if (idx >= 0) {
                        list.splice(idx, 1);
                    }
                    const err = new Error(`Timeout waiting for service: ${serviceName}`);
                    warn(err.message);
                    if (typeof opts.onTimeout === 'function') {
                        opts.onTimeout(err);
                    }
                }, timeoutMs),
            };
            this._serviceReadyWaiters.get(serviceName).push(entry);
            log('waiting for service advertise', serviceName, 'timeoutMs=', timeoutMs);
        }

        _callService(serviceObj, request, successCb, failCb, fromQueue) {
            const serviceName = serviceObj.name;
            const service = this._servicesByName.get(serviceName);
            if (!service) {
                if (!fromQueue) {
                    if (!this._pendingServiceCalls.has(serviceName)) {
                        this._pendingServiceCalls.set(serviceName, []);
                    }
                    this._pendingServiceCalls.get(serviceName).push({
                        serviceObj: serviceObj,
                        request: request,
                        successCb: successCb,
                        failCb: failCb,
                    });
                    log('service call queued until advertise', serviceName);
                    return;
                }
                const err = new Error(`Service not advertised yet: ${serviceName}`);
                warn(err.message);
                if (failCb) {
                    failCb(err);
                }
                return;
            }
            const reqSchemaName =
                (service.request && service.request.schemaName) ||
                serviceRequestSchema(serviceObj.serviceType);
            const reqSchemaText =
                (service.request && service.request.schema) ||
                service.requestSchema ||
                lookupFallbackSchema(reqSchemaName);
            const resSchemaName =
                (service.response && service.response.schemaName) ||
                serviceResponseSchema(serviceObj.serviceType);
            const resSchemaText =
                (service.response && service.response.schema) ||
                service.responseSchema ||
                lookupFallbackSchema(resSchemaName);

            const reqCodec = getCodec(reqSchemaName, reqSchemaText);
            const resCodec = getCodec(resSchemaName, resSchemaText);
            if (!reqCodec || !resCodec) {
                const err = new Error(`Missing service schema for ${serviceName}`);
                warn(err.message);
                if (failCb) {
                    failCb(err);
                }
                return;
            }

            const callId = this._nextCallId++;
            const data = reqCodec.writer.writeMessage(unwrapMessage(request));
            this._serviceCalls.set(callId, {
                serviceName: serviceName,
                success: successCb,
                fail: failCb,
                codec: resCodec,
            });
            this._client.sendServiceCallRequest({
                serviceId: service.id,
                callId: callId,
                encoding: 'cdr',
                data: data,
            });
            log('service call', serviceName, 'callId=', callId);
        }
    }

    function getMapByteSize(msg) {
        if (!msg || !msg.data) {
            return '0 cells';
        }
        const len = Array.isArray(msg.data) || ArrayBuffer.isView(msg.data) ? msg.data.length : 0;
        const w = msg.info && msg.info.width ? msg.info.width : '?';
        const h = msg.info && msg.info.height ? msg.info.height : '?';
        return `${w}x${h}, data.length=${len}`;
    }

    class Topic {
        constructor(options) {
            this.ros = options.ros;
            this.name = options.name;
            this.messageType = normalizeTypeName(options.messageType);
            this.qos = options.qos;
            this._advertised = false;
        }

        subscribe(callback) {
            this.ros._subscribe(this, callback);
        }

        unsubscribe(callback) {
            this.ros._unsubscribe(this, callback);
        }

        publish(message) {
            return this.ros._publish(this.name, this.messageType, unwrapMessage(message));
        }

        advertise() {
            this._advertised = true;
            this.ros._ensureClientChannel(this.name, this.messageType);
        }

        unadvertise() {
            this._advertised = false;
            const record = this.ros._clientChannels.get(this.name);
            if (record && this.ros._client) {
                this.ros._client.unadvertise(record.channelId);
                this.ros._clientChannels.delete(this.name);
            }
        }
    }

    class Service {
        constructor(options) {
            this.ros = options.ros;
            this.name = options.name;
            this.serviceType = options.serviceType;
        }

        callService(request, successCallback, failedCallback) {
            this.ros._callService(this, unwrapMessage(request), successCallback, failedCallback);
        }
    }

    function Message(values) {
        if (!(this instanceof Message)) {
            return Object.assign({}, values || {});
        }
        Object.assign(this, values || {});
    }

    function ServiceRequest(values) {
        if (!(this instanceof ServiceRequest)) {
            return Object.assign({}, values || {});
        }
        Object.assign(this, values || {});
    }

    function Vector3(values) {
        this.x = 0;
        this.y = 0;
        this.z = 0;
        if (values) {
            Object.assign(this, values);
        }
    }

    function Quaternion(values) {
        this.x = 0;
        this.y = 0;
        this.z = 0;
        this.w = 1;
        if (values) {
            Object.assign(this, values);
        }
    }

    function Pose(values) {
        this.position = new Vector3();
        this.orientation = new Quaternion();
        if (values) {
            if (values.position) {
                this.position = new Vector3(values.position);
            }
            if (values.orientation) {
                this.orientation = new Quaternion(values.orientation);
            }
        }
    }

    const ROSLIB = {
        Ros: Ros,
        Topic: Topic,
        Service: Service,
        Message: Message,
        ServiceRequest: ServiceRequest,
        Vector3: Vector3,
        Quaternion: Quaternion,
        Pose: Pose,
        REVISION: 'foxglove-shim-1.0',
    };

    global.ROSLIB = ROSLIB;
    global.getFoxgloveWsUrl = function getFoxgloveWsUrl(ip) {
        const host =
            ip ||
            (typeof getCurrentIP === 'function'
                ? getCurrentIP()
                : (function () {
                      const m = window.location.href.match(/:\/\/([^:]+)/);
                      return m ? m[1] : 'localhost';
                  })());
        return `ws://${host}:8765`;
    };

    log('roslib_foxglove shim loaded');
})(typeof window !== 'undefined' ? window : globalThis);
