'use strict';

const { statusSetter, HzMeter, formatHz, shortType, useConnection } = require('../lib/node-common');
const { sameType, similarNames } = require('../lib/type-registry');

const DIAGNOSE_AFTER = 5000;
const RETRY_TYPE_EVERY = 5000;

const QOS_EXPLANATION = (topic) =>
    `no data on ${topic} for ${DIAGNOSE_AFTER / 1000} s although it is advertised. Either the publisher is idle, ` +
    'or the QoS profiles do not match: a BEST_EFFORT publisher (typical for sensor data like /scan, ' +
    'camera images, odometry) never delivers to a RELIABLE subscriber, and a TRANSIENT_LOCAL-only ' +
    'publisher can refuse a VOLATILE one. Compare with `ros2 topic info -v ' + topic + '` and ' +
    '`ros2 topic hz ' + topic + '` on the robot. If rosbridge subscribed before the publisher existed, ' +
    'restarting this flow lets rosbridge pick a matching QoS.';

module.exports = function (RED) {
    function Ros2SubscribeNode(config) {
        RED.nodes.createNode(this, config);
        const node = this;
        const setStatus = statusSetter(node);
        const conn = RED.nodes.getNode(config.connection);
        const topic = (config.topic || '').trim();
        const configuredType = (config.rosType || '').trim();
        const throttle = Math.max(0, parseInt(config.throttle, 10) || 0);
        const queue = Math.max(0, parseInt(config.queue, 10) || 0);

        if (!conn) {
            setStatus('error', 'no connection configured');
            return;
        }
        if (!topic) {
            setStatus('error', 'no topic configured');
            return;
        }

        const hz = new HzMeter();
        let unsubscribe = null;
        let currentType = null;
        let received = 0;
        let receivedAtArm = 0;
        let starting = false;
        let retryTimer = null;
        let watchdog = null;
        let warnedMissing = false;
        let warnedQos = false;
        let closed = false;

        function clearTimers() {
            clearTimeout(retryTimer);
            clearTimeout(watchdog);
            retryTimer = watchdog = null;
        }

        async function start() {
            if (closed || unsubscribe || starting || !conn.client.connected) return;
            starting = true;
            try {
                let advertised = null;
                try {
                    advertised = await conn.registry.topicType(topic);
                } catch (err) {
                    if (!configuredType) throw err;
                    // rosapi missing: a configured type still lets us subscribe
                }
                if (closed || unsubscribe || !conn.client.connected) return;

                if (configuredType && advertised && !sameType(configuredType, advertised)) {
                    const text = `type mismatch: ${topic} is ${advertised}, node expects ${configuredType}`;
                    setStatus('error', text);
                    node.error(`${text} — clear the type on the node to auto-detect, or fix it`);
                    return;
                }
                if (!configuredType && !advertised) {
                    await reportMissing();
                    retryTimer = setTimeout(start, RETRY_TYPE_EVERY);
                    return;
                }

                currentType = configuredType || advertised;
                unsubscribe = conn.client.subscribe(topic, onMessage, {
                    type: currentType,
                    throttle_rate: throttle,
                    queue_length: queue
                });
                setStatus('wait', `subscribed · waiting for data · ${shortType(currentType)}`);
                armWatchdog();
            } catch (err) {
                setStatus('error', err.message);
                node.error(`cannot subscribe to ${topic}: ${err.message}`);
                retryTimer = setTimeout(start, RETRY_TYPE_EVERY);
            } finally {
                starting = false;
            }
        }

        async function reportMissing() {
            let suggestions = [];
            try {
                const topics = await conn.registry.listTopics();
                suggestions = similarNames(topic, topics.map((t) => t.name));
            } catch (_) { /* suggestions are best effort */ }
            const hint = suggestions.length ? ` (did you mean ${suggestions.join(', ')}?)` : '';
            setStatus('wait', `topic not advertised${hint}`);
            if (!warnedMissing) {
                warnedMissing = true;
                node.warn(`${topic} is not advertised by any ROS node yet${hint}. ` +
                    'Waiting for it to appear — or set the message type on the node to subscribe right away.');
            }
        }

        function armWatchdog() {
            clearTimeout(watchdog);
            receivedAtArm = received;
            watchdog = setTimeout(diagnose, DIAGNOSE_AFTER);
        }

        async function diagnose() {
            watchdog = null;
            if (closed || !unsubscribe || received !== receivedAtArm) return;
            let topics = null;
            try {
                topics = await conn.registry.listTopics();
            } catch (_) { /* fall through to the generic hint */ }
            if (closed || received !== receivedAtArm) return;
            if (topics && !topics.some((t) => t.name === topic)) {
                await reportMissing();
            } else {
                setStatus('warn', 'no data — check QoS / publisher');
                if (!warnedQos) {
                    warnedQos = true;
                    node.warn(QOS_EXPLANATION(topic));
                }
            }
        }

        function onMessage(message) {
            received++;
            hz.tick();
            node.send({
                payload: message,
                topic,
                ros: { topic, type: currentType, receivedAt: Date.now() }
            });
        }

        const statusTimer = setInterval(() => {
            if (!unsubscribe || !conn.client.connected || received === 0) return;
            const rate = hz.rate();
            if (rate > 0) {
                setStatus('ok', `${formatHz(rate)} · ${shortType(currentType)}`);
            } else {
                const ago = Math.round((Date.now() - hz.last) / 1000);
                setStatus('idle', `idle · last message ${ago}s ago`);
            }
        }, 1000);

        const onStatus = (level, text, id) => {
            if (!unsubscribe || id !== unsubscribe.id) return;
            if (level === 'error' || level === 'err') {
                setStatus('error', text);
                node.error(`rosbridge refused the subscription to ${topic}: ${text}`);
            } else if (level === 'warning' || level === 'warn') {
                node.warn(`rosbridge: ${text}`);
            }
        };
        conn.client.on('status', onStatus);

        const release = useConnection(node, conn, setStatus, (state) => {
            if (state === 'connected') {
                // the client replays the subscription itself; restart the diagnosis
                if (unsubscribe) {
                    setStatus('wait', `subscribed · waiting for data · ${shortType(currentType)}`);
                    armWatchdog();
                } else {
                    start();
                }
            } else {
                clearTimers();
            }
        });

        node.on('close', (done) => {
            closed = true;
            clearTimers();
            clearInterval(statusTimer);
            conn.client.removeListener('status', onStatus);
            if (unsubscribe) unsubscribe();
            release();
            setStatus.clear();
            done();
        });
    }

    RED.nodes.registerType('ros2-subscribe', Ros2SubscribeNode);
};
