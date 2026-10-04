'use strict';

const EventEmitter = require('events');
const crypto = require('crypto');
const WebSocket = require('ws');
const cbor = require('./cbor');

// action_msgs/msg/GoalStatus codes
const GOAL_STATUS = ['unknown', 'accepted', 'executing', 'canceling', 'succeeded', 'canceled', 'aborted'];

function goalStatusName(code) {
    return GOAL_STATUS[code] || 'unknown';
}

/**
 * rosbridge v2 protocol client (JSON over WebSocket).
 *
 * Events:
 *   state(state, err)        disconnected | connecting | connected | closing
 *   warning(text)            non-fatal problems (bad frames, throwing handlers)
 *   status(level, text, id)  rosbridge "status" frames
 */
class RosbridgeClient extends EventEmitter {
    constructor(opts = {}) {
        super();
        this.setMaxListeners(0);
        this.url = opts.url || 'ws://localhost:9090';
        this.token = opts.token || null;
        this.reconnectMin = opts.reconnectMin ?? 1000;
        this.reconnectMax = opts.reconnectMax ?? 30000;
        this.connectTimeout = opts.connectTimeout ?? 10000;
        this.pingInterval = opts.pingInterval ?? 15000;
        this.serviceTimeout = opts.serviceTimeout ?? 10000;
        this.wsOptions = opts.wsOptions || {};

        this.state = 'disconnected';
        this.lastError = null;
        this.nextRetryAt = null;

        this._ws = null;
        this._closing = null; // socket we are closing ourselves
        this._wanted = false;
        this._attempt = 0;
        this._retryTimer = null;
        this._pingTimer = null;
        this._alive = true;
        this._seq = 0;
        this._session = crypto.randomBytes(3).toString('hex');

        this._subs = new Map();     // topic -> Map<id, {handler, opts, lastAt}>
        this._adverts = new Map();  // topic -> {id, type, latch, qos, refs}
        this._servers = new Map();  // service -> {type, handler}
        this._actionServers = new Map(); // action -> {type, onGoal, onCancel, goals: Set<id>}
        this._calls = new Map();    // id -> {resolve, reject, timer, service}
        this._goals = new Map();    // goalId -> {action, onFeedback, onResult, onError, timer}
    }

    get connected() {
        return this.state === 'connected';
    }

    _nextId(prefix) {
        return `${prefix}:${this._session}:${++this._seq}`;
    }

    _setState(state, err) {
        if (err) this.lastError = err;
        if (this.state === state) return;
        this.state = state;
        this.emit('state', state, err || null);
    }

    connect() {
        this._wanted = true;
        if (this._ws) return;
        clearTimeout(this._retryTimer);
        this._retryTimer = null;
        this.nextRetryAt = null;
        this._setState('connecting');

        const options = { handshakeTimeout: this.connectTimeout, ...this.wsOptions };
        if (this.token) options.headers = { ...(options.headers || {}), Authorization: `Bearer ${this.token}` };

        let ws;
        try {
            ws = new WebSocket(this.url, options);
        } catch (err) {
            // Malformed URL: retrying will not fix it.
            this._wanted = false;
            this._setState('disconnected', new Error(`invalid rosbridge URL "${this.url}": ${err.message}`));
            return;
        }
        this._ws = ws;
        let socketError = null;
        ws.on('open', () => this._onOpen(ws));
        ws.on('message', (data, isBinary) => this._onMessage(data, isBinary));
        ws.on('pong', () => { this._alive = true; });
        ws.on('error', (err) => { socketError = readableError(err); });
        ws.on('close', (code, reason) => {
            if (this._ws !== ws) return;
            const byUs = this._closing === ws;
            if (byUs) this._closing = null;
            let err = byUs ? null : socketError;
            if (!err && this._wanted && !byUs) {
                const why = reason && reason.length ? `: ${reason}` : '';
                err = new Error(`connection closed (code ${code}${why})`);
            }
            this._onClose(err, byUs);
        });
    }

    /** Close for good (no reconnect). Resolves once the socket is closed. */
    close() {
        this._wanted = false;
        clearTimeout(this._retryTimer);
        this._retryTimer = null;
        this.nextRetryAt = null;
        const ws = this._ws;
        if (!ws) {
            this._setState('disconnected');
            return Promise.resolve();
        }
        this._closing = ws;
        this._setState('closing');
        return new Promise((resolve) => {
            const kill = setTimeout(() => { try { ws.terminate(); } catch (_) { /* ignore */ } }, 2000);
            ws.once('close', () => { clearTimeout(kill); resolve(); });
            try { ws.close(); } catch (_) { ws.terminate(); }
        });
    }

    _onOpen(ws) {
        if (this._ws !== ws) return;
        this._attempt = 0;
        this.lastError = null;
        this._alive = true;
        clearInterval(this._pingTimer);
        if (this.pingInterval > 0) {
            this._pingTimer = setInterval(() => {
                // A robot dropping off Wi-Fi never sends a FIN; pings catch the half-open socket.
                if (!this._alive) { ws.terminate(); return; }
                this._alive = false;
                try { ws.ping(); } catch (_) { /* close handler follows */ }
            }, this.pingInterval);
        }
        this._setState('connected');
        this._replay();
    }

    _onClose(err, byUs = false) {
        clearInterval(this._pingTimer);
        this._pingTimer = null;
        this._ws = null;

        const reason = this._wanted ? 'connection to rosbridge lost' : 'connection to rosbridge closed';
        const detail = err ? ` (${err.message})` : '';
        for (const [id, call] of this._calls) {
            clearTimeout(call.timer);
            this._calls.delete(id);
            call.reject(new Error(`${reason} while calling ${call.service}${detail}`));
        }
        for (const [goalId, goal] of this._goals) {
            clearTimeout(goal.timer);
            this._goals.delete(goalId);
            safeCall(this, goal.onError, new Error(`${reason} while goal on ${goal.action} was running${detail}`));
        }
        // rosbridge aborts the goals of an action server whose client went away
        for (const [action, server] of this._actionServers) {
            for (const id of server.goals) safeCall(this, server.onLost, id, action);
            server.goals.clear();
        }

        this._setState('disconnected', err);
        if (!this._wanted) return;
        // connect() was called while our own close() was still in flight
        if (byUs) this.connect();
        else this._scheduleReconnect();
    }

    _scheduleReconnect() {
        const base = Math.min(this.reconnectMax, this.reconnectMin * Math.pow(2, this._attempt));
        const delay = Math.round(base * (0.85 + Math.random() * 0.3));
        this._attempt++;
        this.nextRetryAt = Date.now() + delay;
        this._retryTimer = setTimeout(() => {
            this._retryTimer = null;
            if (this._wanted) this.connect();
        }, delay);
    }

    _replay() {
        for (const [topic, a] of this._adverts) this._send(advertiseFrame(topic, a));
        for (const [service, s] of this._servers) {
            this._send({ op: 'advertise_service', service, type: s.type });
        }
        for (const [action, s] of this._actionServers) {
            this._send({ op: 'advertise_action', action, type: s.type });
        }
        for (const [topic, subs] of this._subs) {
            for (const [id, s] of subs) this._send(subscribeFrame(id, topic, s.opts));
        }
    }

    _send(frame) {
        if (!this.connected || !this._ws) return false;
        this._ws.send(JSON.stringify(frame));
        return true;
    }

    _requireConnected(what) {
        if (!this.connected) {
            throw new Error(`not connected to rosbridge at ${this.url} — cannot ${what} (state: ${this.state})`);
        }
    }

    _onMessage(data, isBinary) {
        let m;
        try {
            // binary frames carry CBOR (subscriptions with compression: "cbor")
            m = isBinary ? cbor.decode(data) : JSON.parse(data.toString());
        } catch (err) {
            this.emit('warning', `ignoring ${isBinary ? 'undecodable binary' : 'non-JSON'} frame from rosbridge: ${err.message}`);
            return;
        }
        if (m === null || typeof m !== 'object') {
            this.emit('warning', 'ignoring a frame from rosbridge that is not an object');
            return;
        }
        switch (m.op) {
            case 'publish': return this._onPublish(m);
            case 'service_response': return this._onServiceResponse(m);
            case 'call_service': return this._onServiceRequest(m);
            case 'send_action_goal': return this._onActionGoal(m);
            case 'cancel_action_goal': return this._onActionCancel(m);
            case 'action_feedback': return this._onActionFeedback(m);
            case 'action_result': return this._onActionResult(m);
            case 'status': return this._onStatus(m);
            default:
                this.emit('warning', `unsupported rosbridge op "${m.op}"`);
        }
    }

    // ---- topics ------------------------------------------------------------

    /**
     * Subscribe to a topic. Several subscriptions to the same topic share the
     * incoming frames: rosbridge tags `publish` frames with the topic only, and
     * throttles all of them at the smallest throttle_rate. A subscription that
     * asked for a slower rate is throttled here.
     * opts: type, throttle_rate, queue_length, compression, qos
     * @returns {Function} unsubscribe, with `.id` for status routing
     */
    subscribe(topic, handler, opts = {}) {
        const id = this._nextId('sub');
        const clean = {
            type: opts.type || undefined,
            throttle_rate: opts.throttle_rate || undefined,
            queue_length: opts.queue_length || undefined,
            compression: opts.compression === 'cbor' ? 'cbor' : undefined,
            qos: opts.qos || undefined
        };
        if (!this._subs.has(topic)) this._subs.set(topic, new Map());
        this._subs.get(topic).set(id, { handler, opts: clean, lastAt: 0 });
        this._send(subscribeFrame(id, topic, clean));

        let active = true;
        const unsubscribe = () => {
            if (!active) return;
            active = false;
            const subs = this._subs.get(topic);
            if (subs) {
                subs.delete(id);
                if (subs.size === 0) this._subs.delete(topic);
            }
            this._send({ op: 'unsubscribe', id, topic });
        };
        unsubscribe.id = id;
        return unsubscribe;
    }

    _onPublish(m) {
        const subs = this._subs.get(m.topic);
        if (!subs) return;
        let shared = Infinity; // the rate rosbridge applies: the smallest one asked for
        if (subs.size > 1) for (const s of subs.values()) shared = Math.min(shared, s.opts.throttle_rate || 0);
        const now = subs.size > 1 ? Date.now() : 0;
        for (const sub of subs.values()) {
            const { handler } = sub;
            const own = sub.opts.throttle_rate || 0;
            if (subs.size > 1 && own > shared) {
                // a few ms of tolerance, so jitter does not swallow every other message
                if (now - sub.lastAt < own * 0.9) continue;
                sub.lastAt = now;
            }
            try {
                handler(m.msg, m.topic);
            } catch (err) {
                this.emit('warning', `subscriber for ${m.topic} threw: ${err.message}`);
            }
        }
    }

    /**
     * Advertise a topic. Advertisements are ref-counted per topic.
     * @returns {{id, publish(msg), unadvertise()}}
     */
    advertise(topic, type, opts = {}) {
        let a = this._adverts.get(topic);
        if (a) {
            if (a.type !== type) {
                throw new Error(`${topic} is already advertised as ${a.type} by another node; cannot advertise it as ${type}`);
            }
            if (a.latch !== !!opts.latch || JSON.stringify(a.qos || null) !== JSON.stringify(opts.qos || null)) {
                this.emit('warning', `${topic} is already advertised with other latch / QoS settings by another node; ` +
                    'those stay in effect for every publisher of the topic on this connection');
            }
            a.refs++;
        } else {
            a = { id: this._nextId('adv'), type, latch: !!opts.latch, qos: opts.qos || null, refs: 1 };
            this._adverts.set(topic, a);
            this._send(advertiseFrame(topic, a));
        }
        let active = true;
        return {
            id: a.id,
            publish: (msg) => {
                if (!active) throw new Error(`${topic} was unadvertised`);
                this._requireConnected(`publish to ${topic}`);
                this._send({ op: 'publish', id: a.id, topic, msg, latch: a.latch || undefined });
            },
            unadvertise: () => {
                if (!active) return;
                active = false;
                const cur = this._adverts.get(topic);
                if (cur && --cur.refs <= 0) {
                    this._adverts.delete(topic);
                    this._send({ op: 'unadvertise', id: cur.id, topic });
                }
            }
        };
    }

    // ---- services ----------------------------------------------------------

    callService(service, args = {}, opts = {}) {
        return new Promise((resolve, reject) => {
            this._requireConnected(`call ${service}`);
            const id = this._nextId('call');
            const timeout = opts.timeout ?? this.serviceTimeout;
            const call = { resolve, reject, service, timer: null };
            if (timeout > 0) {
                call.timer = setTimeout(() => {
                    this._calls.delete(id);
                    reject(Object.assign(
                        new Error(`no response from ${service} within ${fmtSeconds(timeout)} — is the service server running? (ros2 service list)`),
                        { code: 'TIMEOUT' }
                    ));
                }, timeout);
            }
            this._calls.set(id, call);
            // rosbridge has its own per-call limit (5 s by default since Jazzy);
            // send ours so it does not cut longer calls short. 0 = wait forever.
            const frame = { op: 'call_service', id, service, args, timeout: timeout > 0 ? timeout / 1000 : 0 };
            if (opts.type) frame.type = opts.type;
            this._send(frame);
        });
    }

    _onServiceResponse(m) {
        const call = this._calls.get(m.id);
        if (!call) return;
        this._calls.delete(m.id);
        clearTimeout(call.timer);
        if (m.result === false) {
            const text = typeof m.values === 'string' ? m.values : JSON.stringify(m.values);
            if (/timeout exceeded/i.test(text)) {
                call.reject(Object.assign(new Error(`no response from ${call.service} within rosbridge's time limit — is the service server running? (ros2 service list)`), { code: 'TIMEOUT' }));
                return;
            }
            call.reject(Object.assign(new Error(`${call.service} failed: ${text}`), { code: 'SERVICE_FAILED' }));
        } else {
            call.resolve(m.values ?? {});
        }
    }

    /**
     * Provide a service. handler(args, respond, requestId); respond(values, ok = true)
     * must be called exactly once.
     */
    advertiseService(service, type, handler) {
        if (this._servers.has(service)) {
            throw new Error(`${service} is already provided by another node in this Node-RED instance`);
        }
        const entry = { type, handler };
        this._servers.set(service, entry);
        this._send({ op: 'advertise_service', service, type });
        let active = true;
        return {
            unadvertise: () => {
                if (!active) return;
                active = false;
                if (this._servers.get(service) === entry) {
                    this._servers.delete(service);
                    this._send({ op: 'unadvertise_service', service });
                }
            }
        };
    }

    _onServiceRequest(m) {
        const server = this._servers.get(m.service);
        let answered = false;
        const respond = (values, ok = true) => {
            if (answered) return false;
            answered = true;
            return this._send({ op: 'service_response', id: m.id, service: m.service, values: values ?? {}, result: ok !== false });
        };
        if (!server) {
            respond({}, false);
            return;
        }
        try {
            server.handler(m.args || {}, respond, m.id);
        } catch (err) {
            this.emit('warning', `service handler for ${m.service} threw: ${err.message}`);
            respond({}, false);
        }
    }

    // ---- actions -----------------------------------------------------------

    /**
     * Send an action goal.
     * callbacks: onFeedback(values), onResult({status, statusCode, values}), onError(err)
     * Exactly one of onResult / onError is called.
     */
    sendGoal(action, actionType, args = {}, opts = {}) {
        this._requireConnected(`send a goal to ${action}`);
        const goalId = this._nextId('goal');
        const goal = {
            action,
            onFeedback: opts.onFeedback,
            onResult: opts.onResult,
            onError: opts.onError,
            timer: null
        };
        if (opts.timeout > 0) {
            goal.timer = setTimeout(() => {
                if (!this._goals.has(goalId)) return;
                this._send({ op: 'cancel_action_goal', id: goalId, action });
                this._goals.delete(goalId);
                safeCall(this, goal.onError, Object.assign(
                    new Error(`goal on ${action} did not finish within ${fmtSeconds(opts.timeout)} — cancel requested`),
                    { code: 'TIMEOUT' }
                ));
            }, opts.timeout);
        }
        this._goals.set(goalId, goal);
        this._send({ op: 'send_action_goal', id: goalId, action, action_type: actionType, args, feedback: true });
        return { goalId, cancel: () => this.cancelGoal(goalId), abandon: () => this.abandonGoal(goalId) };
    }

    cancelGoal(goalId) {
        const goal = this._goals.get(goalId);
        if (!goal) return false;
        return this._send({ op: 'cancel_action_goal', id: goalId, action: goal.action });
    }

    /** Ask for a cancel and stop tracking the goal: no callback fires afterwards. */
    abandonGoal(goalId) {
        const goal = this._goals.get(goalId);
        if (!goal) return false;
        clearTimeout(goal.timer);
        this._goals.delete(goalId);
        this._send({ op: 'cancel_action_goal', id: goalId, action: goal.action });
        return true;
    }

    get activeGoals() {
        return this._goals.size;
    }

    /**
     * Provide an action. handlers: onGoal(args, goal), onCancel(goalId), onLost(goalId).
     * goal = {id, feedback(values), succeed(values), cancel(values), abort()}; exactly
     * one of succeed / cancel / abort ends it. onLost fires for goals that were
     * running when the connection dropped (rosbridge aborts them).
     */
    advertiseAction(action, type, handlers = {}) {
        if (this._actionServers.has(action)) {
            throw new Error(`${action} is already provided by another node in this Node-RED instance`);
        }
        const entry = { type, onGoal: handlers.onGoal, onCancel: handlers.onCancel, onLost: handlers.onLost, goals: new Set() };
        this._actionServers.set(action, entry);
        this._send({ op: 'advertise_action', action, type });
        let active = true;
        return {
            unadvertise: () => {
                if (!active) return;
                active = false;
                if (this._actionServers.get(action) === entry) {
                    this._actionServers.delete(action);
                    this._send({ op: 'unadvertise_action', action });
                }
            }
        };
    }

    _onActionGoal(m) {
        const server = this._actionServers.get(m.action);
        const finish = (values, status, ok) => {
            if (!server || !server.goals.delete(m.id)) return false;
            return this._send({ op: 'action_result', id: m.id, action: m.action, values: values ?? {}, status, result: ok });
        };
        if (!server) {
            this._send({ op: 'action_result', id: m.id, action: m.action, values: {}, status: 6, result: false });
            return;
        }
        server.goals.add(m.id);
        const goal = {
            id: m.id,
            feedback: (values) => server.goals.has(m.id) &&
                this._send({ op: 'action_feedback', id: m.id, action: m.action, values: values ?? {} }),
            succeed: (values) => finish(values, 4, true),
            cancel: (values) => finish(values, 5, true),
            abort: () => finish({}, 6, false)
        };
        try {
            if (typeof server.onGoal !== 'function') throw new Error('no goal handler');
            server.onGoal(m.args || {}, goal);
        } catch (err) {
            this.emit('warning', `action handler for ${m.action} threw: ${err.message}`);
            goal.abort();
        }
    }

    _onActionCancel(m) {
        const server = this._actionServers.get(m.action);
        if (server && server.goals.has(m.id)) safeCall(this, server.onCancel, m.id);
    }

    _onActionFeedback(m) {
        const goal = this._goals.get(m.id);
        if (goal) safeCall(this, goal.onFeedback, m.values);
    }

    _onActionResult(m) {
        const goal = this._goals.get(m.id);
        if (!goal) return;
        this._goals.delete(m.id);
        clearTimeout(goal.timer);
        if (m.result === false) {
            const text = typeof m.values === 'string' ? m.values : JSON.stringify(m.values);
            safeCall(this, goal.onError, Object.assign(new Error(`goal on ${goal.action} failed: ${text}`), { code: 'GOAL_FAILED' }));
            return;
        }
        const statusCode = typeof m.status === 'number' ? m.status : 4;
        safeCall(this, goal.onResult, { status: goalStatusName(statusCode), statusCode, values: m.values ?? {} });
    }

    // ---- status ------------------------------------------------------------

    _onStatus(m) {
        const level = m.level || 'info';
        const text = m.msg || '';
        const isError = level === 'error' || level === 'err';
        if (m.id && isError) {
            const call = this._calls.get(m.id);
            if (call) {
                this._calls.delete(m.id);
                clearTimeout(call.timer);
                call.reject(Object.assign(new Error(`rosbridge rejected call to ${call.service}: ${text}`), { code: 'ROSBRIDGE_STATUS' }));
            }
            const goal = this._goals.get(m.id);
            if (goal) {
                this._goals.delete(m.id);
                clearTimeout(goal.timer);
                safeCall(this, goal.onError, Object.assign(new Error(`rosbridge rejected goal on ${goal.action}: ${text}`), { code: 'ROSBRIDGE_STATUS' }));
            }
        }
        this.emit('status', level, text, m.id || null);
    }
}

// Node >= 20 reports failed dual-stack connects as AggregateError with an empty message.
function readableError(err) {
    const msg = err && err.message;
    const http = msg && /Unexpected server response: (\d+)/.exec(msg);
    if (http && (http[1] === '401' || http[1] === '403')) {
        return Object.assign(new Error(`refused with HTTP ${http[1]} — check the token of the connection`), { code: err.code });
    }
    if (msg && /self[- ]signed certificate|unable to verify the first certificate/i.test(msg)) {
        return Object.assign(new Error(`${msg} — untick "verify the server certificate" for self-signed certificates`), { code: err.code });
    }
    if (msg) return err;
    const inner = err && Array.isArray(err.errors) ? err.errors.map((e) => e.message || e.code).filter(Boolean) : [];
    const text = inner.length ? [...new Set(inner)].join('; ') : (err && err.code) || String(err);
    return Object.assign(new Error(text), { code: err && err.code });
}

function subscribeFrame(id, topic, opts) {
    const frame = { op: 'subscribe', id, topic };
    if (opts.type) frame.type = opts.type;
    if (opts.throttle_rate) frame.throttle_rate = opts.throttle_rate;
    if (opts.queue_length) frame.queue_length = opts.queue_length;
    if (opts.compression) frame.compression = opts.compression;
    if (opts.qos) frame.qos = opts.qos;
    return frame;
}

function advertiseFrame(topic, a) {
    const frame = { op: 'advertise', id: a.id, topic, type: a.type };
    if (a.latch) frame.latch = true;
    if (a.qos) frame.qos = a.qos;
    return frame;
}

function safeCall(emitter, fn, ...args) {
    if (typeof fn !== 'function') return;
    try {
        fn(...args);
    } catch (err) {
        emitter.emit('warning', `callback threw: ${err.message}`);
    }
}

function fmtSeconds(ms) {
    return `${+(ms / 1000).toFixed(1)} s`;
}

module.exports = { RosbridgeClient, GOAL_STATUS, goalStatusName };
