'use strict';

const { statusSetter, pick, useConnection } = require('../lib/node-common');
const { toEuler, frameId } = require('../lib/tf');

module.exports = function (RED) {
    function Ros2TfNode(config) {
        RED.nodes.createNode(this, config);
        const node = this;
        const setStatus = statusSetter(node);
        const conn = RED.nodes.getNode(config.connection);
        const configOperation = config.operation === 'frames' ? 'frames' : 'lookup';
        const configTarget = frameId(config.target);
        const configSource = frameId(config.source);
        const allowOverride = !!config.allowOverride;
        const maxAgeSec = Number(config.maxAge);
        const maxAge = Number.isFinite(maxAgeSec) && maxAgeSec > 0 ? maxAgeSec * 1000 : 0;

        if (!conn) {
            setStatus('error', 'no connection configured');
            node.on('input', (msg, send, done) => done(new Error('no ROS 2 connection configured on this node')));
            return;
        }

        // The connection keeps one transform tree for all tf nodes.
        const tf = conn.tfAcquire();

        node.on('input', (msg, send, done) => {
            const operation = msg.operation === 'frames' || msg.operation === 'lookup' ? msg.operation : configOperation;
            try {
                if (!conn.client.connected) throw new Error(`not connected to rosbridge at ${conn.url}`);
                if (operation === 'frames') {
                    msg.payload = tf.frames();
                    msg.ros = { operation };
                    setStatus('ok', `${msg.payload.length} frames`);
                } else {
                    const target = frameId(pick(msg.target, configTarget, allowOverride));
                    const source = frameId(pick(msg.source, configSource, allowOverride));
                    if (!target || !source) throw new Error('no frames — set target and source on the node or pass msg.target / msg.source');
                    const { ageMs, static: isStatic, ...transform } = tf.lookup(target, source);
                    if (maxAge && ageMs !== null && ageMs > maxAge) {
                        throw new Error(`the transform from ${source} to ${target} is ${(ageMs / 1000).toFixed(1)} s old (limit ${maxAge / 1000} s) — is its publisher still running?`);
                    }
                    msg.payload = transform;
                    msg.ros = { operation, target, source, ageMs, static: isStatic, rpy: toEuler(transform.transform.rotation) };
                    setStatus('ok', `${source} → ${target}`);
                }
                send(msg);
                done();
            } catch (err) {
                setStatus('error', err.message);
                done(err);
            }
        });

        const release = useConnection(node, conn, setStatus, (state) => {
            if (state === 'connected') setStatus('idle', configTarget && configSource ? `ready · ${configSource} → ${configTarget}` : 'ready');
        });
        node.on('close', (done) => {
            conn.tfRelease();
            release();
            setStatus.clear();
            done();
        });
    }

    RED.nodes.registerType('ros2-tf', Ros2TfNode);
};
