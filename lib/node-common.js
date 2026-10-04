'use strict';

const { normalizeType } = require('./type-registry');

const LEVELS = {
    ok: { fill: 'green', shape: 'dot' },
    idle: { fill: 'green', shape: 'ring' },
    busy: { fill: 'blue', shape: 'dot' },
    wait: { fill: 'yellow', shape: 'ring' },
    warn: { fill: 'yellow', shape: 'dot' },
    error: { fill: 'red', shape: 'ring' },
    off: { fill: 'grey', shape: 'ring' }
};

const MAX_STATUS = 60;

/** Status setter that skips redundant updates (the editor gets a comms frame per call). */
function statusSetter(node) {
    let last = '';
    const set = (level, text) => {
        const t = text && text.length > MAX_STATUS ? text.slice(0, MAX_STATUS - 1) + '…' : (text || '');
        const key = `${level}|${t}`;
        if (key === last) return;
        last = key;
        node.status(level ? { ...(LEVELS[level] || LEVELS.off), text: t } : {});
    };
    set.clear = () => { last = ''; node.status({}); };
    return set;
}

/** Messages-per-second over a sliding window. */
class HzMeter {
    constructor(windowMs = 3000) {
        this.windowMs = windowMs;
        this.stamps = [];
        this.last = 0;
    }

    tick(now = Date.now()) {
        this.stamps.push(now);
        this.last = now;
        this._trim(now);
    }

    rate(now = Date.now()) {
        this._trim(now);
        const n = this.stamps.length;
        if (n < 2) return n === 1 && now - this.stamps[0] < this.windowMs ? 1000 / this.windowMs : 0;
        const span = Math.max(now - this.stamps[0], this.stamps[n - 1] - this.stamps[0]);
        return span > 0 ? ((n - 1) * 1000) / span : 0;
    }

    _trim(now) {
        const cutoff = now - this.windowMs;
        let i = 0;
        while (i < this.stamps.length && this.stamps[i] < cutoff) i++;
        if (i) this.stamps.splice(0, i);
        // hard cap for very high rates
        if (this.stamps.length > 5000) this.stamps.splice(0, this.stamps.length - 5000);
    }
}

function formatHz(hz) {
    if (hz >= 100) return `${Math.round(hz)} Hz`;
    if (hz >= 10) return `${hz.toFixed(1)} Hz`;
    return `${hz.toFixed(2)} Hz`;
}

/** Short type label for status texts: geometry_msgs/msg/Twist -> geometry_msgs/Twist */
function shortType(type) {
    return normalizeType(type) || '?';
}

/**
 * Pick msg override vs. configured value. With a configured value the msg
 * property only wins when overriding is enabled, so a stray msg.topic from an
 * upstream subscribe node does not redirect a publisher.
 */
function pick(msgValue, configValue, allowOverride) {
    const m = typeof msgValue === 'string' ? msgValue.trim() : '';
    const c = typeof configValue === 'string' ? configValue.trim() : '';
    if (!c) return m;
    return allowOverride && m ? m : c;
}

function stateText(client) {
    switch (client.state) {
        case 'connecting': return 'connecting…';
        case 'closing': return 'closing';
        case 'disconnected': {
            const err = client.lastError ? client.lastError.message : 'not connected';
            if (client.nextRetryAt) {
                const s = Math.max(0, Math.round((client.nextRetryAt - Date.now()) / 1000));
                return `disconnected (retry in ${s}s): ${err}`;
            }
            return `disconnected: ${err}`;
        }
        default: return client.state;
    }
}

/**
 * Wire a runtime node to its connection: registers the node, mirrors
 * connection state into the status dot and calls onState on every change.
 * Returns a cleanup function for the node's close handler.
 */
function useConnection(node, conn, setStatus, onState) {
    const client = conn.client;
    const handler = (state) => {
        if (state !== 'connected') setStatus(state === 'connecting' ? 'wait' : 'error', stateText(client));
        if (onState) onState(state);
    };
    conn.register(node);
    client.on('state', handler);
    handler(client.state);
    return () => {
        client.removeListener('state', handler);
        conn.deregister(node);
    };
}

const STAMPER_RETRY = 10000;

const QOS_PRESETS = {
    default: { reliability: 'reliable', durability: 'volatile', depth: 10 },
    sensor: { reliability: 'best_effort', durability: 'volatile', depth: 5 },
    latched: { reliability: 'reliable', durability: 'transient_local', depth: 1 }
};

/**
 * rosbridge `qos` object for a node's QoS settings, or null for "auto"
 * (rosbridge then picks one from the publishers it sees).
 */
function qosFromConfig(config) {
    const choice = config.qos;
    if (QOS_PRESETS[choice]) return { history: 'keep_last', ...QOS_PRESETS[choice] };
    if (choice !== 'custom') return null;
    const depth = parseInt(config.qosDepth, 10);
    return {
        history: 'keep_last',
        reliability: config.qosReliability === 'best_effort' ? 'best_effort' : 'reliable',
        durability: config.qosDurability === 'transient_local' ? 'transient_local' : 'volatile',
        depth: Number.isInteger(depth) && depth > 0 ? depth : 10
    };
}

/** "reliable · transient_local · 1" */
function qosText(qos) {
    return qos ? `${qos.reliability} · ${qos.durability} · depth ${qos.depth}` : 'auto';
}

/**
 * Header stamping for a node: returns async (type, kind, payload) => payload
 * with the empty stamps of its headers filled, plus `.close()` for the node's
 * close handler. source: off | system | clock.
 */
function makeStamper(node, conn, source) {
    if (source !== 'system' && source !== 'clock') {
        const off = async (type, kind, payload) => payload;
        off.close = () => {};
        return off;
    }
    const stampers = new Map(); // kind:type -> {fn: function | null, retryAt}
    let warnedType = false;
    let warnedClock = false;
    if (source === 'clock') conn.clockAcquire();
    const stamp = async (type, kind, payload) => {
        if (payload === null || typeof payload !== 'object') return payload;
        const key = `${kind}:${type}`;
        let entry = stampers.get(key);
        if (!entry || (entry.retryAt && Date.now() >= entry.retryAt)) {
            try {
                entry = { fn: await conn.registry.stamper(type, kind), retryAt: 0 };
            } catch (err) {
                // do not ask rosapi again for every message
                entry = { fn: null, retryAt: Date.now() + STAMPER_RETRY };
                if (!warnedType) {
                    warnedType = true;
                    node.warn(`cannot load the definition of ${type} (${err.message}) — header stamps are not filled`);
                }
            }
            stampers.set(key, entry);
        }
        if (!entry.fn) return payload;
        const time = conn.rosTime(source);
        if (source === 'clock' && time.source !== 'clock' && !warnedClock) {
            warnedClock = true;
            node.warn('no message on /clock yet — stamping with the system time. Is a simulator publishing /clock?');
        }
        return entry.fn(payload, { sec: time.sec, nanosec: time.nanosec });
    };
    let closed = false;
    stamp.close = () => {
        if (closed) return;
        closed = true;
        if (source === 'clock') conn.clockRelease();
    };
    return stamp;
}

/**
 * Validation for a node: returns async (type, value, kind) that warns or, in
 * strict mode, throws. mode: strict | warn | off. Without a type or a type
 * definition nothing is checked: rosbridge decides.
 */
function makeValidator(node, conn, mode) {
    return async (type, value, kind) => {
        if (mode === 'off' || !type) return;
        let result;
        try {
            result = await conn.registry.validate(type, value, kind);
        } catch (_) {
            return;
        }
        for (const w of result.warnings) node.warn(w);
        if (!result.errors.length) return;
        const text = `invalid ${kind} for ${shortType(type)}: ${result.errors.join('; ')}`;
        if (mode === 'strict') throw Object.assign(new Error(text), { validation: result.errors });
        node.warn(text);
    };
}

module.exports = {
    qosFromConfig,
    qosText,
    makeStamper,
    makeValidator,
    statusSetter,
    HzMeter,
    formatHz,
    shortType,
    pick,
    stateText,
    useConnection
};
