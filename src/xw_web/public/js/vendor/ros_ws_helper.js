(function (global) {
    const LOG = '[Foxglove Mig]';

    function createManagedRosConnection(options) {
        if (!options) {
            throw new Error('createManagedRosConnection requires options');
        }

        const url = options.url || (typeof global.getFoxgloveWsUrl === 'function' ? global.getFoxgloveWsUrl() : null);
        if (!url) {
            throw new Error('createManagedRosConnection requires a url or getFoxgloveWsUrl()');
        }

        const ros = new ROSLIB.Ros();
        const name = options.name || 'foxglove';
        const initialReconnectDelayMs = options.initialReconnectDelayMs || 800;
        const maxReconnectDelayMs = options.maxReconnectDelayMs || 5000;
        const backoffMultiplier = options.backoffMultiplier || 1.6;
        const heartbeatEnabled = options.enableHeartbeat !== false;
        const heartbeatIntervalMs = options.heartbeatIntervalMs || 15000;
        const heartbeatTopicName = options.heartbeatTopicName || '/websocket_heartbeat';
        let reconnectDelayMs = initialReconnectDelayMs;
        let reconnectTimer = null;
        let heartbeatTimer = null;
        let heartbeatTopic = null;
        let manuallyStopped = false;
        let connecting = false;

        function stopHeartbeat() {
            if (heartbeatTimer) {
                clearInterval(heartbeatTimer);
                heartbeatTimer = null;
            }

            if (heartbeatTopic) {
                try {
                    heartbeatTopic.unadvertise();
                } catch (error) {
                    console.warn(LOG, name + ' heartbeat unadvertise failed:', error);
                }
                heartbeatTopic = null;
            }
        }

        function sendHeartbeat() {
            if (!heartbeatEnabled || ros.isConnected !== true) {
                return;
            }

            if (!heartbeatTopic) {
                heartbeatTopic = new ROSLIB.Topic({
                    ros: ros,
                    name: heartbeatTopicName,
                    messageType: 'std_msgs/msg/String',
                });

                try {
                    heartbeatTopic.advertise();
                } catch (error) {
                    console.warn(LOG, name + ' heartbeat advertise failed:', error);
                    heartbeatTopic = null;
                    return;
                }
            }

            try {
                heartbeatTopic.publish(
                    new ROSLIB.Message({
                        data: JSON.stringify({
                            source: name,
                            kind: 'heartbeat',
                            timestamp: new Date().toISOString(),
                            url: url,
                        }),
                    })
                );

                if (typeof options.onHeartbeat === 'function') {
                    options.onHeartbeat({ name: name, topic: heartbeatTopicName, ros: ros });
                }
            } catch (error) {
                if (typeof options.onHeartbeatError === 'function') {
                    options.onHeartbeatError(error, { name: name, topic: heartbeatTopicName, ros: ros });
                }
            }
        }

        function startHeartbeat() {
            if (!heartbeatEnabled) {
                return;
            }

            stopHeartbeat();
            sendHeartbeat();
            heartbeatTimer = setInterval(sendHeartbeat, heartbeatIntervalMs);
        }

        function clearReconnectTimer() {
            if (!reconnectTimer) {
                return;
            }

            clearTimeout(reconnectTimer);
            reconnectTimer = null;
        }

        function scheduleReconnect(reason) {
            if (manuallyStopped || reconnectTimer) {
                return;
            }

            const delayMs = reconnectDelayMs;
            if (typeof options.onReconnectScheduled === 'function') {
                options.onReconnectScheduled({
                    delayMs: delayMs,
                    reason: reason,
                    name: name,
                    ros: ros,
                });
            }

            reconnectTimer = setTimeout(function () {
                reconnectTimer = null;
                connect();
            }, delayMs);

            reconnectDelayMs = Math.min(
                Math.round(reconnectDelayMs * backoffMultiplier),
                maxReconnectDelayMs
            );
        }

        function connect() {
            if (manuallyStopped || connecting) {
                return;
            }

            connecting = true;
            try {
                ros.connect(url);
            } catch (error) {
                connecting = false;
                if (typeof options.onError === 'function') {
                    options.onError(error, { phase: 'connect', ros: ros, name: name });
                }
                scheduleReconnect(error);
            }
        }

        ros.on('connection', function () {
            connecting = false;
            clearReconnectTimer();
            reconnectDelayMs = initialReconnectDelayMs;
            startHeartbeat();
            console.log(LOG, name, 'connected', url);
            if (typeof options.onConnection === 'function') {
                options.onConnection(ros);
            }
        });

        ros.on('error', function (error) {
            connecting = false;
            stopHeartbeat();
            if (manuallyStopped) {
                return;
            }

            if (typeof options.onError === 'function') {
                options.onError(error, { phase: 'runtime', ros: ros, name: name });
            }
            scheduleReconnect(error);
        });

        ros.on('close', function () {
            connecting = false;
            stopHeartbeat();
            if (manuallyStopped) {
                return;
            }

            if (typeof options.onClose === 'function') {
                options.onClose(ros);
            }
            scheduleReconnect(new Error(name + ' closed'));
        });

        ros.managedClose = function () {
            manuallyStopped = true;
            clearReconnectTimer();
            stopHeartbeat();
            try {
                ros.close();
            } catch (error) {
                console.warn(LOG, name + ' managedClose failed:', error);
            }
        };

        ros.reconnectNow = function () {
            manuallyStopped = false;
            reconnectDelayMs = initialReconnectDelayMs;
            clearReconnectTimer();
            connect();
        };

        connect();
        return ros;
    }

    global.createManagedRosConnection = createManagedRosConnection;
})(window);
