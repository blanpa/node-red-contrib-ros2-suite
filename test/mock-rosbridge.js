'use strict';

// A small rosbridge v2 stand-in: enough protocol and rosapi to exercise the
// client, the registry and the nodes without a ROS installation.

const { WebSocketServer } = require('ws');
const { normalizeType } = require('../lib/type-registry');

function T(type, fields) {
    return {
        type,
        fieldnames: fields.map((f) => f[0]),
        fieldtypes: fields.map((f) => f[1]),
        fieldarraylen: fields.map((f) => (f[2] === undefined ? -1 : f[2])),
        examples: fields.map(() => ''),
        constnames: [],
        constvalues: []
    };
}

const TYPEDEFS = {
    'geometry_msgs/Twist': T('geometry_msgs/Twist', [['linear', 'geometry_msgs/Vector3'], ['angular', 'geometry_msgs/Vector3']]),
    'geometry_msgs/Vector3': T('geometry_msgs/Vector3', [['x', 'float64'], ['y', 'float64'], ['z', 'float64']]),
    'std_msgs/String': T('std_msgs/String', [['data', 'string']]),
    'std_msgs/Header': T('std_msgs/Header', [['stamp', 'builtin_interfaces/Time'], ['frame_id', 'string']]),
    'builtin_interfaces/Time': T('builtin_interfaces/Time', [['sec', 'int32'], ['nanosec', 'uint32']]),
    'turtlesim/Pose': T('turtlesim/Pose', [
        ['x', 'float32'], ['y', 'float32'], ['theta', 'float32'],
        ['linear_velocity', 'float32'], ['angular_velocity', 'float32']
    ]),
    'test_msgs/Arrays': T('test_msgs/Arrays', [
        ['header', 'std_msgs/Header'],
        ['data', 'uint8', 0],
        ['fixed', 'float64', 3],
        ['names', 'string', 0],
        ['points', 'geometry_msgs/Vector3', 0],
        ['count', 'int8']
    ]),
    'turtlesim/TeleportAbsolute_Request': T('turtlesim/TeleportAbsolute_Request', [['x', 'float32'], ['y', 'float32'], ['theta', 'float32']]),
    'turtlesim/TeleportAbsolute_Response': T('turtlesim/TeleportAbsolute_Response', []),
    'std_srvs/Trigger_Request': T('std_srvs/Trigger_Request', []),
    'std_srvs/Trigger_Response': T('std_srvs/Trigger_Response', [['success', 'bool'], ['message', 'string']]),
    'turtlesim/RotateAbsolute_Goal': T('turtlesim/RotateAbsolute_Goal', [['theta', 'float32']]),
    'turtlesim/RotateAbsolute_Result': T('turtlesim/RotateAbsolute_Result', [['delta', 'float32']]),
    'turtlesim/RotateAbsolute_Feedback': T('turtlesim/RotateAbsolute_Feedback', [['remaining', 'float32']])
};

function typedefClosure(typeName) {
    const out = [];
    const seen = new Set();
    const visit = (t) => {
        const key = normalizeType(t);
        if (seen.has(key) || !TYPEDEFS[key]) return;
        seen.add(key);
        const def = TYPEDEFS[key];
        out.push(def);
        def.fieldtypes.forEach(visit);
    };
    visit(typeName);
    return out;
}

class MockRosbridge {
    constructor(opts = {}) {
        this.port = opts.port || 0;
        this.wss = null;
        this.sockets = new Set();
        this.received = [];
        this._waiters = [];
        this._pendingClientCalls = new Map();
        this._seq = 0;
        this.rosapiEnabled = true;

        this.topics = [
            { name: '/turtle1/pose', type: 'turtlesim/msg/Pose' },
            { name: '/turtle1/cmd_vel', type: 'geometry_msgs/msg/Twist' },
            { name: '/chatter', type: 'std_msgs/msg/String' },
            { name: '/rosout', type: 'rcl_interfaces/msg/Log' },
            { name: '/turtle1/rotate_absolute/_action/feedback', type: 'turtlesim/action/RotateAbsolute_FeedbackMessage' },
            { name: '/turtle1/rotate_absolute/_action/status', type: 'action_msgs/msg/GoalStatusArray' }
        ];
        this.nodes = ['/turtlesim', '/rosbridge_websocket', '/rosapi'];

        // name -> {type, handler(args) => values | Promise; throw to fail; return MockRosbridge.NEVER to hang}
        this.services = new Map([
            ['/turtle1/teleport_absolute', { type: 'turtlesim/srv/TeleportAbsolute', handler: () => ({}) }],
            ['/reset', { type: 'std_srvs/srv/Empty', handler: () => ({}) }],
            ['/slow_service', { type: 'std_srvs/srv/Trigger', handler: () => MockRosbridge.NEVER }],
            ['/failing_service', { type: 'std_srvs/srv/Trigger', handler: () => { throw new Error('boom'); } }]
        ]);

        // name -> {type, feedback: [..], intervalMs, status, result, reject}
        this.actions = new Map([
            ['/turtle1/rotate_absolute', {
                type: 'turtlesim/action/RotateAbsolute',
                feedback: [{ remaining: 1.0 }, { remaining: 0.5 }, { remaining: 0.1 }],
                intervalMs: 20,
                status: 4,
                result: { delta: 1.2 }
            }]
        ]);
        this._goals = new Map(); // goal id -> {timer, socket, action}
        this.clientServices = new Map(); // service -> socket
    }

    get url() {
        return `ws://127.0.0.1:${this.port}`;
    }

    start() {
        return new Promise((resolve, reject) => {
            this.wss = new WebSocketServer({ port: this.port, host: '127.0.0.1' });
            this.wss.once('error', reject);
            this.wss.once('listening', () => {
                this.port = this.wss.address().port;
                resolve(this);
            });
            this.wss.on('connection', (ws, req) => {
                ws.subs = new Map(); // topic -> Set(id)
                ws.headers = req.headers;
                this.sockets.add(ws);
                ws.on('message', (data) => this._onFrame(ws, JSON.parse(data.toString())));
                ws.on('close', () => {
                    this.sockets.delete(ws);
                    for (const [svc, s] of this.clientServices) if (s === ws) this.clientServices.delete(svc);
                });
            });
        });
    }

    stop() {
        for (const g of this._goals.values()) clearInterval(g.timer);
        this._goals.clear();
        return new Promise((resolve) => {
            if (!this.wss) return resolve();
            for (const ws of this.sockets) ws.terminate();
            this.wss.close(() => resolve());
            this.wss = null;
        });
    }

    async restart() {
        await this.stop();
        await this.start();
    }

    dropClients() {
        for (const ws of this.sockets) ws.terminate();
    }

    get lastHeaders() {
        const [ws] = this.sockets;
        return ws ? ws.headers : null;
    }

    /** Resolve with the first received frame (past or future) matching pred. */
    waitFor(pred, timeout = 2000) {
        const hit = this.received.find(pred);
        if (hit) return Promise.resolve(hit);
        return new Promise((resolve, reject) => {
            const w = { pred, resolve, timer: null };
            w.timer = setTimeout(() => {
                this._waiters = this._waiters.filter((x) => x !== w);
                reject(new Error('mock: timed out waiting for frame'));
            }, timeout);
            this._waiters.push(w);
        });
    }

    clearReceived() {
        this.received = [];
    }

    publish(topic, msg) {
        for (const ws of this.sockets) {
            if (ws.subs.has(topic) && ws.subs.get(topic).size) ws.send(JSON.stringify({ op: 'publish', topic, msg }));
        }
    }

    sendStatus(id, level, msg) {
        for (const ws of this.sockets) ws.send(JSON.stringify({ op: 'status', id, level, msg }));
    }

    /** Call a service the client advertised. */
    callClientService(service, args = {}) {
        const ws = this.clientServices.get(service);
        if (!ws) return Promise.reject(new Error(`mock: nobody advertises ${service}`));
        const id = `mock_call:${++this._seq}`;
        return new Promise((resolve) => {
            this._pendingClientCalls.set(id, resolve);
            ws.send(JSON.stringify({ op: 'call_service', id, service, args }));
        });
    }

    subscriberCount(topic) {
        let n = 0;
        for (const ws of this.sockets) n += ws.subs.has(topic) ? ws.subs.get(topic).size : 0;
        return n;
    }

    _send(ws, frame) {
        if (ws.readyState === ws.OPEN) ws.send(JSON.stringify(frame));
    }

    _onFrame(ws, m) {
        this.received.push(m);
        for (const w of this._waiters.slice()) {
            if (w.pred(m)) {
                clearTimeout(w.timer);
                this._waiters = this._waiters.filter((x) => x !== w);
                w.resolve(m);
            }
        }
        switch (m.op) {
            case 'subscribe':
                if (!ws.subs.has(m.topic)) ws.subs.set(m.topic, new Set());
                ws.subs.get(m.topic).add(m.id);
                break;
            case 'unsubscribe':
                if (ws.subs.has(m.topic)) ws.subs.get(m.topic).delete(m.id);
                break;
            case 'advertise':
                if (!this.topics.find((t) => t.name === m.topic)) this.topics.push({ name: m.topic, type: m.type });
                break;
            case 'publish':
                this.publish(m.topic, m.msg);
                break;
            case 'call_service':
                this._onCallService(ws, m);
                break;
            case 'advertise_service':
                this.clientServices.set(m.service, ws);
                break;
            case 'unadvertise_service':
                this.clientServices.delete(m.service);
                break;
            case 'service_response': {
                const resolve = this._pendingClientCalls.get(m.id);
                if (resolve) {
                    this._pendingClientCalls.delete(m.id);
                    resolve(m);
                }
                break;
            }
            case 'send_action_goal':
                this._onGoal(ws, m);
                break;
            case 'cancel_action_goal': {
                const g = this._goals.get(m.id);
                if (g) {
                    clearInterval(g.timer);
                    this._goals.delete(m.id);
                    this._send(ws, { op: 'action_result', id: m.id, action: g.action, values: { delta: 0 }, status: 5, result: true });
                }
                break;
            }
            default:
                break;
        }
    }

    _onGoal(ws, m) {
        const a = this.actions.get(m.action);
        if (!a) {
            this._send(ws, { op: 'action_result', id: m.id, action: m.action, values: `Action ${m.action} does not exist`, result: false });
            return;
        }
        if (a.reject) {
            this._send(ws, { op: 'action_result', id: m.id, action: m.action, values: 'Action goal was rejected', result: false });
            return;
        }
        let i = 0;
        const g = { action: m.action, timer: null };
        this._goals.set(m.id, g);
        g.timer = setInterval(() => {
            if (i < a.feedback.length) {
                this._send(ws, { op: 'action_feedback', id: m.id, action: m.action, values: a.feedback[i++] });
                return;
            }
            if (a.hang) return;
            clearInterval(g.timer);
            this._goals.delete(m.id);
            this._send(ws, { op: 'action_result', id: m.id, action: m.action, values: a.result, status: a.status, result: true });
        }, a.intervalMs || 20);
    }

    async _onCallService(ws, m) {
        const reply = (values, result = true) => this._send(ws, { op: 'service_response', id: m.id, service: m.service, values, result });
        if (m.service.startsWith('/rosapi/')) {
            if (!this.rosapiEnabled) return; // behave like a missing rosapi node: no answer
            try {
                reply(this._rosapi(m.service.slice('/rosapi/'.length), m.args || {}));
            } catch (err) {
                reply(err.message, false);
            }
            return;
        }
        const svc = this.services.get(m.service);
        if (!svc) {
            reply(`Service ${m.service} does not exist`, false);
            return;
        }
        try {
            const values = await svc.handler(m.args || {});
            if (values === MockRosbridge.NEVER) return;
            reply(values);
        } catch (err) {
            reply(err.message, false);
        }
    }

    _rosapi(name, args) {
        switch (name) {
            case 'topics':
                return { topics: this.topics.map((t) => t.name), types: this.topics.map((t) => t.type) };
            case 'topic_type': {
                const t = this.topics.find((x) => x.name === args.topic);
                return { type: t ? t.type : '' };
            }
            case 'services':
                return { services: [...this.services.keys(), ...this.clientServices.keys(), '/rosapi/topics', '/rosapi/nodes'] };
            case 'service_type': {
                const s = this.services.get(args.service);
                return { type: s ? s.type : '' };
            }
            case 'nodes':
                return { nodes: this.nodes };
            case 'action_servers':
                return { action_servers: [...this.actions.keys()] };
            case 'message_details':
                return { typedefs: typedefClosure(args.type) };
            case 'service_request_details':
                return { typedefs: typedefClosure(`${normalizeType(args.type)}_Request`) };
            case 'service_response_details':
                return { typedefs: typedefClosure(`${normalizeType(args.type)}_Response`) };
            case 'action_goal_details':
                return { typedefs: typedefClosure(`${normalizeType(args.type)}_Goal`) };
            case 'action_result_details':
                return { typedefs: typedefClosure(`${normalizeType(args.type)}_Result`) };
            case 'action_feedback_details':
                return { typedefs: typedefClosure(`${normalizeType(args.type)}_Feedback`) };
            default:
                throw new Error(`Service /rosapi/${name} does not exist`);
        }
    }
}

MockRosbridge.NEVER = Symbol('never');
MockRosbridge.TYPEDEFS = TYPEDEFS;

module.exports = { MockRosbridge };
