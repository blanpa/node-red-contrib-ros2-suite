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

module.exports = {
    statusSetter,
    HzMeter,
    formatHz,
    shortType,
    pick,
    stateText,
    useConnection
};
