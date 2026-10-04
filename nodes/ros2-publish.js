'use strict';

const { statusSetter, shortType, pick, useConnection, qosFromConfig, makeStamper } = require('../lib/node-common');
const { similarNames, encodeBuffers } = require('../lib/type-registry');

// rosbridge creates the ROS publisher on "advertise"; messages sent before
// subscribers have matched it are silently dropped by DDS.
const FRESH_ADVERTISE_DELAY = 250;

const delay = (ms) => new Promise((r) => setTimeout(r, ms));

module.exports = function (RED) {
    function Ros2PublishNode(config) {
        RED.nodes.createNode(this, config);
        const node = this;
        const setStatus = statusSetter(node);
        const conn = RED.nodes.getNode(config.connection);
        const configTopic = (config.topic || '').trim();
        const configType = (config.rosType || '').trim();
        const validation = ['strict', 'warn', 'off'].includes(config.validation) ? config.validation : 'strict';
        const latch = !!config.latch;
        const qos = qosFromConfig(config);
        const allowOverride = !!config.allowOverride;

        if (!conn) {
            setStatus('error', 'no connection configured');
            node.on('input', (msg, send, done) => done(new Error('no ROS 2 connection configured on this node')));
            return;
        }

        const stamp = makeStamper(node, conn, config.stamp);
        const handles = new Map(); // topic -> {type, handle, readyAt}
        let sent = 0;
        let lastType = configType;
        let warnedNoValidation = false;

        function handleFor(topic, type) {
            const cur = handles.get(topic);
            if (cur && cur.type === type) return cur;
            if (cur) {
                cur.handle.unadvertise();
                handles.delete(topic);
            }
            const entry = {
                type,
                handle: conn.client.advertise(topic, type, { latch, qos }),
                readyAt: Date.now() + FRESH_ADVERTISE_DELAY
            };
            handles.set(topic, entry);
            return entry;
        }

        async function resolveType(topic, msg) {
            if (typeof msg.rosType === 'string' && msg.rosType.trim()) return msg.rosType.trim();
            if (configType) return configType;
            const cached = handles.get(topic);
            if (cached) return cached.type;
            let advertised;
            let topics = [];
            try {
                advertised = await conn.registry.topicType(topic);
                if (!advertised) topics = (await conn.registry.listTopics()).map((t) => t.name);
            } catch (err) {
                throw new Error(`no message type for ${topic} and rosapi could not tell (${err.message}) — set the type on the node or pass msg.rosType`, { cause: err });
            }
            if (!advertised) {
                const similar = similarNames(topic, topics);
                const hint = similar.length ? ` (similar: ${similar.join(', ')})` : '';
                throw new Error(`no message type for ${topic} — nothing advertises it yet, so set the type on the node (e.g. geometry_msgs/msg/Twist) or pass msg.rosType${hint}`);
            }
            return advertised;
        }

        async function prepare(type, payload) {
            payload = encodeBuffers(payload); // Buffers travel as base64
            payload = await stamp(type, 'msg', payload);
            // std_msgs-style wrappers: publish "hello" to std_msgs/String as {data: "hello"}
            const primitive = payload === null || ['string', 'number', 'boolean'].includes(typeof payload);
            if (!primitive && validation === 'off') return payload;
            let fields;
            try {
                fields = await conn.registry.fieldNames(type);
            } catch (err) {
                if (primitive) throw new Error(`msg.payload must be an object for ${type} (could not load its definition: ${err.message})`, { cause: err });
                if (!warnedNoValidation) {
                    warnedNoValidation = true;
                    node.warn(`cannot validate ${type}: ${err.message} — publishing without validation`);
                }
                return payload;
            }
            if (primitive && payload !== null && fields.length === 1 && fields[0] === 'data') {
                payload = { data: payload };
            }
            if (validation === 'off') return payload;
            const { errors, warnings } = await conn.registry.validate(type, payload);
            for (const w of warnings) node.warn(w);
            if (errors.length) {
                const text = `invalid ${shortType(type)}: ${errors.join('; ')}`;
                if (validation === 'strict') throw Object.assign(new Error(text), { validation: errors });
                node.warn(`${text} — publishing anyway (validation: warn)`);
            }
            return payload;
        }

        node.on('input', async (msg, send, done) => {
            const topic = pick(msg.topic, configTopic, allowOverride);
            try {
                if (!topic) throw new Error('no topic — set it on the node or pass msg.topic');
                if (!conn.client.connected) {
                    throw new Error(`not connected to rosbridge at ${conn.url} — message to ${topic} dropped`);
                }
                const type = await resolveType(topic, msg);
                const payload = await prepare(type, msg.payload === undefined ? {} : msg.payload);
                const entry = handleFor(topic, type);
                const wait = entry.readyAt - Date.now();
                if (wait > 0) await delay(wait);
                entry.handle.publish(payload);
                sent++;
                lastType = type;
                setStatus('ok', `sent ${sent} · ${shortType(type)}`);
                msg.ros = { topic, type };
                send(msg);
                done();
            } catch (err) {
                setStatus('error', err.message);
                done(err);
            }
        });

        const release = useConnection(node, conn, setStatus, (state) => {
            if (state !== 'connected') return;
            if (sent) setStatus('ok', `sent ${sent} · ${shortType(lastType)}`);
            else setStatus('idle', configTopic ? `ready · ${configTopic}` : 'ready');
            // The client re-advertised everything, so rosbridge's publishers are new again.
            for (const entry of handles.values()) entry.readyAt = Date.now() + FRESH_ADVERTISE_DELAY;
            // Advertise early so subscribers have matched before the first message.
            if (configTopic && configType && !handles.has(configTopic)) {
                try {
                    handleFor(configTopic, configType);
                } catch (err) {
                    setStatus('error', err.message);
                }
            }
        });

        node.on('close', (done) => {
            for (const { handle } of handles.values()) handle.unadvertise();
            handles.clear();
            release();
            setStatus.clear();
            done();
        });
    }

    RED.nodes.registerType('ros2-publish', Ros2PublishNode);
};
