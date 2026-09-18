'use strict';

const { RosbridgeClient } = require('../lib/rosbridge-client');
const { TypeRegistry } = require('../lib/type-registry');

const KINDS = new Set(['msg', 'request', 'response', 'goal', 'result', 'feedback']);

function buildUrl(config) {
    const url = (config.url || '').trim();
    if (url) return url;
    const host = (config.host || '').trim() || 'localhost';
    const port = parseInt(config.port, 10) || 9090;
    const path = (config.path || '').trim().replace(/^\/?/, '/');
    return `${config.tls ? 'wss' : 'ws'}://${host}:${port}${path === '/' ? '' : path}`;
}

function positive(value, fallback) {
    const n = Number(value);
    return Number.isFinite(n) && n > 0 ? n : fallback;
}

module.exports = function (RED) {
    function Ros2ConnectionNode(config) {
        RED.nodes.createNode(this, config);
        const node = this;
        node.name = config.name;
        node.url = buildUrl(config);
        node.serviceTimeout = positive(config.serviceTimeout, 10) * 1000;

        node.client = new RosbridgeClient({
            url: node.url,
            token: node.credentials && node.credentials.token,
            reconnectMin: positive(config.reconnectMin, 1) * 1000,
            reconnectMax: positive(config.reconnectMax, 30) * 1000,
            serviceTimeout: node.serviceTimeout
        });
        node.registry = new TypeRegistry(node.client);
        node.users = new Set();

        let lastLogged = null;
        node.client.on('state', (state, err) => {
            if (state === 'connected') {
                node.log(`connected to ${node.url}`);
                lastLogged = null;
            } else if (state === 'disconnected' && err && err.message !== lastLogged) {
                // log each distinct failure once, not every retry
                lastLogged = err.message;
                node.warn(`rosbridge at ${node.url} unreachable: ${err.message} — is rosbridge running? (ros2 launch rosbridge_server rosbridge_websocket_launch.xml)`);
            }
        });
        node.client.on('warning', (text) => node.warn(text));

        // Connect lazily: an unused config node should not hold a socket open.
        node.register = (user) => {
            node.users.add(user.id);
            if (!node.client._wanted) node.client.connect();
        };
        node.deregister = (user) => {
            node.users.delete(user.id);
        };

        node.ensureConnected = (timeout = 4000) => {
            const client = node.client;
            if (client.connected) return Promise.resolve();
            if (!client._wanted) client.connect();
            return new Promise((resolve, reject) => {
                const timer = setTimeout(() => {
                    client.removeListener('state', onState);
                    const why = client.lastError ? client.lastError.message : client.state;
                    reject(new Error(`cannot reach rosbridge at ${node.url}: ${why}`));
                }, timeout);
                function onState(state) {
                    if (state !== 'connected') return;
                    clearTimeout(timer);
                    client.removeListener('state', onState);
                    resolve();
                }
                client.on('state', onState);
            });
        };

        node.on('close', (removed, done) => {
            node.client.close().then(() => done(), () => done());
        });
    }

    RED.nodes.registerType('ros2-connection', Ros2ConnectionNode, {
        credentials: { token: { type: 'password' } }
    });

    // ---- editor endpoints ----------------------------------------------------

    const canRead = RED.auth.needsPermission('flows.read');

    function getConnection(req, res) {
        const conn = RED.nodes.getNode(req.params.id);
        if (!conn || conn.type !== 'ros2-connection') {
            res.status(404).json({ error: 'connection is not deployed yet — deploy once, then browsing works' });
            return null;
        }
        return conn;
    }

    function route(path, fn) {
        RED.httpAdmin.get(`/ros2-suite/:id/${path}`, canRead, async (req, res) => {
            const conn = getConnection(req, res);
            if (!conn) return;
            try {
                await conn.ensureConnected();
                res.json(await fn(conn, req));
            } catch (err) {
                res.status(503).json({ error: err.message });
            }
        });
    }

    route('topics', (conn) => conn.registry.listTopics());
    route('services', (conn) => conn.registry.listServices());
    route('actions', (conn) => conn.registry.listActions());
    route('nodes', (conn) => conn.registry.listNodes());
    route('template', async (conn, req) => {
        const type = String(req.query.type || '').trim();
        const kind = String(req.query.kind || 'msg');
        if (!type) throw new Error('no type given');
        if (!KINDS.has(kind)) throw new Error(`kind must be one of ${[...KINDS].join(', ')}`);
        return { type, kind, template: await conn.registry.template(type, kind) };
    });

    RED.httpAdmin.get('/ros2-suite/:id/state', canRead, (req, res) => {
        const conn = getConnection(req, res);
        if (!conn) return;
        const c = conn.client;
        res.json({
            state: c.state,
            url: conn.url,
            lastError: c.lastError ? c.lastError.message : null,
            nextRetryAt: c.nextRetryAt,
            users: conn.users.size
        });
    });
};

module.exports.buildUrl = buildUrl;
