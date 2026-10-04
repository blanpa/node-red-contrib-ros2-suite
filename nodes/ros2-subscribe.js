'use strict';

const { statusSetter, HzMeter, formatHz, shortType, useConnection, qosFromConfig, qosText } = require('../lib/node-common');
const { sameType, similarNames } = require('../lib/type-registry');

const DIAGNOSE_AFTER = 5000;
const RETRY_TYPE_EVERY = 5000;

// rosbridge (2.x) fixes the QoS when the first of its clients subscribes to a
// topic: BEST_EFFORT + VOLATILE, or TRANSIENT_LOCAL + RELIABLE when every
// publisher present at that moment is latched. Such a subscription never
// receives VOLATILE or BEST_EFFORT publishers that join later.
const QOS_EXPLANATION = (topic, qosLabel) =>
    `no data on ${topic} for ${DIAGNOSE_AFTER / 1000} s although it is advertised. Likely causes: ` +
    '(1) the publisher is idle — check `ros2 topic hz ' + topic + '`; ' +
    '(2) QoS: rosbridge fixes its QoS when the topic is first subscribed through it (by any rosbridge client) — ' +
    'TRANSIENT_LOCAL + RELIABLE if all publishers present then were latched, which never receives VOLATILE or ' +
    'BEST_EFFORT publishers that joined later. Compare with `ros2 topic info -v ' + topic + '` and set the QoS ' +
    'on this node to match the publisher (this node asked for: ' + qosLabel + '); a changed QoS takes effect ' +
    'only once no rosbridge client keeps the topic subscribed; ' +
    '(3) discovery works but data does not arrive: DDS shared-memory transport between containers without a ' +
    'shared /dev/shm, firewalls, or large messages (images, point clouds) lost over Wi-Fi with BEST_EFFORT.';

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
        const compression = config.compression === 'cbor' ? 'cbor' : 'none';
        const buffers = !!config.buffers;
        const qos = qosFromConfig(config);

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
        let decodeBinary = null;
        let warnedDecoder = false;
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

                const type = configuredType || advertised;
                // CBOR delivers uint8[] as Buffers already; JSON needs the type definition
                decodeBinary = null;
                if (buffers && compression !== 'cbor') {
                    try {
                        decodeBinary = await conn.registry.binaryDecoder(type);
                    } catch (err) {
                        if (!warnedDecoder) {
                            warnedDecoder = true;
                            node.warn(`cannot load the definition of ${type} (${err.message}) — uint8[] fields stay base64 strings`);
                        }
                    }
                    if (closed || unsubscribe || !conn.client.connected) return;
                }

                currentType = type;
                unsubscribe = conn.client.subscribe(topic, onMessage, {
                    type: currentType,
                    throttle_rate: throttle,
                    queue_length: queue,
                    compression,
                    qos
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
                    node.warn(QOS_EXPLANATION(topic, qosText(qos)));
                }
            }
            // keep the status current (e.g. a publisher appears later) until data arrives
            if (!closed && unsubscribe && received === receivedAtArm && conn.client.connected) {
                watchdog = setTimeout(diagnose, DIAGNOSE_AFTER);
            }
        }

        function onMessage(message) {
            received++;
            hz.tick();
            if (decodeBinary) decodeBinary(message);
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
