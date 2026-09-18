'use strict';

const { statusSetter, useConnection } = require('../lib/node-common');

const WHATS = ['topics', 'services', 'actions', 'nodes', 'all', 'template'];
const KINDS = ['msg', 'request', 'response', 'goal', 'result', 'feedback'];

module.exports = function (RED) {
    function Ros2BrowseNode(config) {
        RED.nodes.createNode(this, config);
        const node = this;
        const setStatus = statusSetter(node);
        const conn = RED.nodes.getNode(config.connection);
        const configWhat = WHATS.includes(config.what) ? config.what : 'topics';
        const configType = (config.rosType || '').trim();
        const configKind = KINDS.includes(config.kind) ? config.kind : 'msg';

        if (!conn) {
            setStatus('error', 'no connection configured');
            node.on('input', (msg, send, done) => done(new Error('no ROS 2 connection configured on this node')));
            return;
        }

        async function browse(what, msg) {
            const r = conn.registry;
            switch (what) {
                case 'topics': return r.listTopics();
                case 'services': return r.listServices();
                case 'actions': return r.listActions();
                case 'nodes': return r.listNodes();
                case 'all': {
                    const [topics, services, actions, nodes] = await Promise.all([
                        r.listTopics(), r.listServices(), r.listActions(), r.listNodes()
                    ]);
                    return { topics, services, actions, nodes };
                }
                case 'template': {
                    const type = (typeof msg.rosType === 'string' && msg.rosType.trim()) || configType;
                    const kind = KINDS.includes(msg.kind) ? msg.kind : configKind;
                    if (!type) throw new Error('template needs a type — set it on the node or pass msg.rosType (e.g. geometry_msgs/msg/Twist)');
                    msg.rosType = type;
                    return r.template(type, kind);
                }
                default:
                    throw new Error(`cannot browse "${what}" — use one of ${WHATS.join(', ')}`);
            }
        }

        node.on('input', async (msg, send, done) => {
            const what = typeof msg.payload === 'string' && WHATS.includes(msg.payload.trim())
                ? msg.payload.trim()
                : configWhat;
            setStatus('busy', `${what}…`);
            try {
                if (!conn.client.connected) throw new Error(`not connected to rosbridge at ${conn.url}`);
                const result = await browse(what, msg);
                msg.payload = result;
                msg.ros = { what, at: Date.now() };
                const n = Array.isArray(result) ? `${result.length} ${what}` : what;
                setStatus('ok', n);
                send(msg);
                done();
            } catch (err) {
                setStatus('error', err.message);
                done(err);
            }
        });

        const release = useConnection(node, conn, setStatus, (state) => {
            if (state === 'connected') setStatus('idle', 'ready');
        });
        node.on('close', (done) => {
            release();
            setStatus.clear();
            done();
        });
    }

    RED.nodes.registerType('ros2-browse', Ros2BrowseNode);
};
