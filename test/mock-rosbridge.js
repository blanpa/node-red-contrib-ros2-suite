'use strict';

// A small rosbridge v2 stand-in: enough protocol and rosapi to exercise the
// client, the registry and the nodes without a ROS installation. Typedefs use
// the spellings Jazzy's rosapi reports (double/float for float64/float32).

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
    'geometry_msgs/Vector3': T('geometry_msgs/Vector3', [['x', 'double'], ['y', 'double'], ['z', 'double']]),
    'std_msgs/String': T('std_msgs/String', [['data', 'string']]),
    'std_msgs/Header': T('std_msgs/Header', [['stamp', 'builtin_interfaces/Time'], ['frame_id', 'string']]),
    'builtin_interfaces/Time': T('builtin_interfaces/Time', [['sec', 'int32'], ['nanosec', 'uint32']]),
    'turtlesim/Pose': T('turtlesim/Pose', [
        ['x', 'float'], ['y', 'float'], ['theta', 'float'],
        ['linear_velocity', 'float'], ['angular_velocity', 'float']
    ]),
    'test_msgs/Arrays': T('test_msgs/Arrays', [
        ['header', 'std_msgs/Header'],
        ['data', 'uint8', 0],
        ['fixed', 'float64', 3],
        ['names', 'string', 0],
        ['points', 'geometry_msgs/Vector3', 0],
        ['count', 'int8']
    ]),
    'tf2_msgs/TFMessage': T('tf2_msgs/TFMessage', [['transforms', 'geometry_msgs/TransformStamped', 0]]),
    'geometry_msgs/TransformStamped': T('geometry_msgs/TransformStamped', [
        ['header', 'std_msgs/Header'], ['child_frame_id', 'string'], ['transform', 'geometry_msgs/Transform']
    ]),
    'geometry_msgs/Transform': T('geometry_msgs/Transform', [['translation', 'geometry_msgs/Vector3'], ['rotation', 'geometry_msgs/Quaternion']]),
    'geometry_msgs/Quaternion': T('geometry_msgs/Quaternion', [['x', 'double'], ['y', 'double'], ['z', 'double'], ['w', 'double']]),
    'test_msgs/Plan_Request': T('test_msgs/Plan_Request', [['start', 'test_msgs/Stamped'], ['tolerance', 'float32']]),
    'test_msgs/Stamped': T('test_msgs/Stamped', [['header', 'std_msgs/Header'], ['value', 'float64']]),
    'turtlesim/TeleportAbsolute_Request': T('turtlesim/TeleportAbsolute_Request', [['x', 'float'], ['y', 'float'], ['theta', 'float']]),
    'turtlesim/TeleportAbsolute_Response': T('turtlesim/TeleportAbsolute_Response', []),
    'std_srvs/Trigger_Request': T('std_srvs/Trigger_Request', []),
    'std_srvs/Trigger_Response': T('std_srvs/Trigger_Response', [['success', 'bool'], ['message', 'string']]),
    'turtlesim/RotateAbsolute_Goal': T('turtlesim/RotateAbsolute_Goal', [['theta', 'float32']]),
    'turtlesim/RotateAbsolute_Result': T('turtlesim/RotateAbsolute_Result', [['delta', 'float32']]),
    'turtlesim/RotateAbsolute_Feedback': T('turtlesim/RotateAbsolute_Feedback', [['remaining', 'float32']])
};

// Minimal CBOR encoder, enough for the frames rosbridge sends with compression: "cbor".
function cborHead(major, n) {
    if (n < 24) return Buffer.from([(major << 5) | n]);
    if (n < 0x100) return Buffer.from([(major << 5) | 24, n]);
    if (n < 0x10000) { const b = Buffer.alloc(3); b[0] = (major << 5) | 25; b.writeUInt16BE(n, 1); return b; }
    const b = Buffer.alloc(5); b[0] = (major << 5) | 26; b.writeUInt32BE(n, 1); return b;
}

function cborEncode(v) {
    if (v === null) return Buffer.from([0xf6]);
    if (v === true) return Buffer.from([0xf5]);
    if (v === false) return Buffer.from([0xf4]);
    if (typeof v === 'number') {
        if (Number.isInteger(v) && Math.abs(v) < 2 ** 32) return v >= 0 ? cborHead(0, v) : cborHead(1, -1 - v);
        const b = Buffer.alloc(9); b[0] = 0xfb; b.writeDoubleBE(v, 1); return b;
    }
    if (typeof v === 'string') { const s = Buffer.from(v, 'utf8'); return Buffer.concat([cborHead(3, s.length), s]); }
    if (Buffer.isBuffer(v)) return Buffer.concat([cborHead(2, v.length), v]);
    if (v instanceof Float32Array) { // RFC 8746 tag 85: float32, little endian
        const bytes = Buffer.from(v.buffer, v.byteOffset, v.byteLength);
        return Buffer.concat([cborHead(6, 85), cborHead(2, bytes.length), bytes]);
    }
    if (Array.isArray(v)) return Buffer.concat([cborHead(4, v.length), ...v.map(cborEncode)]);
    const keys = Object.keys(v);
    return Buffer.concat([cborHead(5, keys.length), ...keys.flatMap((k) => [cborEncode(k), cborEncode(v[k])])]);
}

const PARAM_FIELDS = [null, 'bool_value', 'integer_value', 'double_value', 'string_value',
    'byte_array_value', 'bool_array_value', 'integer_array_value', 'double_array_value', 'string_array_value'];

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
        this.distro = opts.distro || 'jazzy';

        this.topics = [
            { name: '/turtle1/pose', type: 'turtlesim/msg/Pose' },
            { name: '/turtle1/cmd_vel', type: 'geometry_msgs/msg/Twist' },
            { name: '/chatter', type: 'std_msgs/msg/String' },
            { name: '/arrays', type: 'test_msgs/msg/Arrays' },
            { name: '/tf', type: 'tf2_msgs/msg/TFMessage' },
            { name: '/tf_static', type: 'tf2_msgs/msg/TFMessage' },
            { name: '/rosout', type: 'rcl_interfaces/msg/Log' }
        ];
        // Like Jazzy's rosapi, hidden topics (e.g. <action>/_action/feedback) are not listed.
        this.interfaces = [
            'geometry_msgs/msg/Twist', 'std_msgs/msg/String', 'turtlesim/msg/Pose',
            'turtlesim/srv/TeleportAbsolute', 'std_srvs/srv/Trigger',
            'turtlesim/action/RotateAbsolute', 'tf2_msgs/action/LookupTransform'
        ];
        this.nodes = ['/turtlesim', '/rosbridge_websocket', '/rosapi'];

        // name -> {type, handler(args) => values | Promise; throw to fail; return MockRosbridge.NEVER to hang}
        this.services = new Map([
            ['/turtle1/teleport_absolute', { type: 'turtlesim/srv/TeleportAbsolute', handler: () => ({}) }],
            ['/reset', { type: 'std_srvs/srv/Empty', handler: () => ({}) }],
            ['/plan', { type: 'test_msgs/srv/Plan', handler: () => ({}) }],
            ['/slow_service', { type: 'std_srvs/srv/Trigger', handler: () => MockRosbridge.NEVER }],
            ['/failing_service', { type: 'std_srvs/srv/Trigger', handler: () => { throw new Error('boom'); } }],
            ['/turtlesim/get_parameters', {
                type: 'rcl_interfaces/srv/GetParameters',
                // like rclcpp: one undeclared name empties the whole answer
                handler: ({ names }) => ({
                    values: names.every((n) => this.parameters.has(n)) ? names.map((n) => this.parameters.get(n)) : []
                })
            }],
            ['/turtlesim/get_parameter_types', {
                type: 'rcl_interfaces/srv/GetParameterTypes',
                // uint8[] goes over the wire as base64, like in the real rosbridge
                handler: ({ names }) => ({
                    types: names.every((n) => this.parameters.has(n))
                        ? Buffer.from(names.map((n) => this.parameters.get(n).type)).toString('base64')
                        : ''
                })
            }],
            ['/turtlesim/set_parameters', {
                type: 'rcl_interfaces/srv/SetParameters',
                handler: ({ parameters }) => ({
                    results: parameters.map(({ name, value }) => {
                        const cur = this.parameters.get(name);
                        if (cur && cur.type !== value.type) return { successful: false, reason: `wrong type for ${name}` };
                        this.parameters.set(name, { type: value.type, [PARAM_FIELDS[value.type]]: value[PARAM_FIELDS[value.type]] });
                        return { successful: true, reason: '' };
                    })
                })
            }],
            ['/turtlesim/list_parameters', {
                type: 'rcl_interfaces/srv/ListParameters',
                handler: () => ({ result: { names: [...this.parameters.keys()], prefixes: [] } })
            }],
            ['/turtlesim/describe_parameters', {
                type: 'rcl_interfaces/srv/DescribeParameters',
                handler: ({ names }) => ({
                    descriptors: names.every((n) => this.parameters.has(n))
                        ? names.map((name) => ({ name, type: this.parameters.get(name).type, description: `about ${name}`, read_only: false }))
                        : []
                })
            }]
        ]);
        // name -> rcl_interfaces/msg/ParameterValue (only the field in use)
        this.parameters = new Map([
            ['background_r', { type: 2, integer_value: 69 }],
            ['background_g', { type: 2, integer_value: 86 }],
            ['gain', { type: 3, double_value: 0.5 }],
            ['use_sim_time', { type: 1, bool_value: false }]
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
        this.clientActions = new Map(); // action -> {socket, type}
        this.clientGoals = new Map(); // goal id -> {feedback: [], resolve}
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
                ws.cbor = new Set(); // topics subscribed with compression: "cbor"
                ws.headers = req.headers;
                this.sockets.add(ws);
                ws.on('message', (data) => this._onFrame(ws, JSON.parse(data.toString())));
                ws.on('close', () => {
                    this.sockets.delete(ws);
                    for (const [svc, s] of this.clientServices) if (s === ws) this.clientServices.delete(svc);
                    for (const [act, a] of this.clientActions) if (a.socket === ws) this.clientActions.delete(act);
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

    /** Like rosbridge: with CBOR, uint8[] are byte strings (pass Buffers) and float32[] typed arrays (pass Float32Array). */
    publish(topic, msg) {
        for (const ws of this.sockets) {
            if (!ws.subs.has(topic) || !ws.subs.get(topic).size) continue;
            if (ws.cbor.has(topic)) ws.send(cborEncode({ op: 'publish', topic, msg }), { binary: true });
            else ws.send(JSON.stringify({ op: 'publish', topic, msg }));
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

    /** Send a goal to an action the client advertised: {id, feedback: [], result: Promise<frame>}. */
    sendClientGoal(action, args = {}) {
        const a = this.clientActions.get(action);
        if (!a) throw new Error(`mock: nobody advertises ${action}`);
        const id = `action_goal:${action}:${++this._seq}`;
        const goal = { id, feedback: [], resolve: null };
        goal.result = new Promise((resolve) => { goal.resolve = resolve; });
        this.clientGoals.set(id, goal);
        a.socket.send(JSON.stringify({ op: 'send_action_goal', id, action, action_type: a.type, args, feedback: true }));
        return goal;
    }

    cancelClientGoal(action, id) {
        const a = this.clientActions.get(action);
        if (a) a.socket.send(JSON.stringify({ op: 'cancel_action_goal', id, action }));
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
                if (m.compression === 'cbor') ws.cbor.add(m.topic);
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
            case 'advertise_action':
                this.clientActions.set(m.action, { socket: ws, type: m.type });
                break;
            case 'unadvertise_action':
                this.clientActions.delete(m.action);
                break;
            case 'action_feedback': {
                const goal = this.clientGoals.get(m.id);
                if (goal) goal.feedback.push(m.values);
                break;
            }
            case 'action_result': {
                const goal = this.clientGoals.get(m.id);
                if (goal) {
                    this.clientGoals.delete(m.id);
                    goal.resolve(m);
                }
                break;
            }
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
                if (g && this.actions.get(g.action).ignoreCancel) {
                    clearInterval(g.timer);
                    this._goals.delete(m.id);
                    this._send(ws, { op: 'action_result', id: m.id, action: g.action, values: { delta: 1 }, status: 4, result: true });
                } else if (g) {
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
            case 'interfaces':
                return { interfaces: this.interfaces };
            case 'get_ros_version':
                return { version: 2, distro: this.distro };
            case 'action_type': {
                if (['humble', 'jazzy', 'kilted'].includes(this.distro)) {
                    this.rosapiEnabled = false; // like the real rosapi: it dies
                    throw new Error('rosapi crashed');
                }
                const a = this.actions.get(args.action);
                return { type: a ? a.type : '' };
            }
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
MockRosbridge.cborEncode = cborEncode;

module.exports = { MockRosbridge };
