'use strict';

const { RosbridgeClient } = require('../lib/rosbridge-client');
const { TypeRegistry } = require('../lib/type-registry');

const KINDS = new Set(['msg', 'request', 'response', 'goal', 'result', 'feedback']);
const PROBE_IDLE = 60000;

function buildUrl(config) {
    const url = (config.url || '').trim();
    if (url) return url;
    const host = (config.host || '').trim() || 'localhost';
    const port = parseInt(config.port, 10) || 9090;
    const path = (config.path || '').trim().replace(/^\/?/, '/');
    const tls = config.tls === true || config.tls === 'true';
    return `${tls ? 'wss' : 'ws'}://${host}:${port}${path === '/' ? '' : path}`;
}

function positive(value, fallback) {
    const n = Number(value);
    return Number.isFinite(n) && n > 0 ? n : fallback;
}

function verifyTls(config) {
    return !(config.verifyTls === false || config.verifyTls === 'false');
}

function ensureConnected(client, timeout = 4000) {
    if (client.connected) return Promise.resolve();
    if (!client._wanted) client.connect();
    return new Promise((resolve, reject) => {
        const timer = setTimeout(() => {
            client.removeListener('state', onState);
            const why = client.lastError ? client.lastError.message : client.state;
            reject(new Error(`cannot reach rosbridge at ${client.url}: ${why}`));
        }, timeout);
        function onState(state) {
            if (state !== 'connected') return;
            clearTimeout(timer);
            client.removeListener('state', onState);
            resolve();
        }
        client.on('state', onState);
    });
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
            serviceTimeout: node.serviceTimeout,
            wsOptions: { rejectUnauthorized: verifyTls(config) }
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
        node.ensureConnected = (timeout) => ensureConnected(node.client, timeout);

        node.on('close', (removed, done) => {
            node.client.close().then(() => done(), () => done());
        });
    }

    RED.nodes.registerType('ros2-connection', Ros2ConnectionNode, {
        credentials: { token: { type: 'password' } }
    });

    // ---- editor endpoints ----------------------------------------------------

    const canRead = RED.auth.needsPermission('flows.read');
    // Probing opens a server-side connection to a URL chosen in the editor.
    const canProbe = RED.auth.needsPermission('flows.write');

    const handlers = {
        topics: (reg) => reg.listTopics(),
        services: (reg) => reg.listServices(),
        actions: (reg) => reg.listActions(),
        nodes: (reg) => reg.listNodes(),
        template: async (reg, params) => {
            const type = String(params.type || '').trim();
            const kind = String(params.kind || 'msg');
            if (!type) throw new Error('no type given');
            if (!KINDS.has(kind)) throw new Error(`kind must be one of ${[...KINDS].join(', ')}`);
            return { type, kind, template: await reg.template(type, kind) };
        }
    };

    function getConnection(req, res) {
        const conn = RED.nodes.getNode(req.params.id);
        if (!conn || conn.type !== 'ros2-connection') {
            res.status(404).json({ error: 'connection is not deployed yet', notDeployed: true });
            return null;
        }
        return conn;
    }

    for (const [what, fn] of Object.entries(handlers)) {
        RED.httpAdmin.get(`/ros2-suite/:id/${what}`, canRead, async (req, res) => {
            const conn = getConnection(req, res);
            if (!conn) return;
            try {
                await conn.ensureConnected();
                res.json(await fn(conn.registry, req.query));
            } catch (err) {
                res.status(503).json({ error: err.message });
            }
        });
    }

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

    // Autocomplete for connections that are new or edited but not deployed:
    // the editor posts the settings from its dialog and gets a short-lived client.
    const probes = new Map(); // key -> {client, registry, timer}

    function probeFor(body) {
        const url = buildUrl(body || {});
        if (!/^wss?:\/\/[^/\s]+/i.test(url)) throw new Error(`not a WebSocket URL: ${url}`);
        let token = typeof body.token === 'string' && body.token ? body.token : null;
        if (!token && body.id) {
            // unchanged token: reuse the deployed node's credential
            const deployed = RED.nodes.getNode(body.id);
            if (deployed && deployed.type === 'ros2-connection') token = deployed.credentials && deployed.credentials.token;
        }
        const verify = verifyTls(body);
        const key = JSON.stringify([url, token, verify]);
        let probe = probes.get(key);
        if (!probe) {
            const client = new RosbridgeClient({ url, token, wsOptions: { rejectUnauthorized: verify }, reconnectMin: 2000, reconnectMax: 10000 });
            probe = { client, registry: new TypeRegistry(client), timer: null };
            probes.set(key, probe);
        }
        clearTimeout(probe.timer);
        probe.timer = setTimeout(() => {
            probes.delete(key);
            probe.client.close();
        }, PROBE_IDLE);
        probe.timer.unref();
        return probe;
    }

    for (const [what, fn] of Object.entries(handlers)) {
        RED.httpAdmin.post(`/ros2-suite/probe/${what}`, canProbe, async (req, res) => {
            try {
                const probe = probeFor(req.body || {});
                await ensureConnected(probe.client);
                res.json(await fn(probe.registry, req.body || {}));
            } catch (err) {
                res.status(503).json({ error: err.message });
            }
        });
    }
};

module.exports.buildUrl = buildUrl;
