'use strict';

// Transform tree built from /tf and /tf_static, and lookups between frames.
// A transform {translation, rotation} of an edge maps points of the child
// frame into the parent frame, as in geometry_msgs/TransformStamped.

const { similarNames } = require('./type-registry');

// A stamp this much older than the stored one is a jump back in time (simulator
// reset, looping bag), not a late sample.
const TIME_JUMP_MS = 1000;

const IDENTITY = { translation: { x: 0, y: 0, z: 0 }, rotation: { x: 0, y: 0, z: 0, w: 1 } };

function frameId(name) {
    return String(name || '').trim().replace(/^\/+/, '');
}

function multiply(a, b) {
    return {
        x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
    };
}

function normalize(q) {
    const n = Math.hypot(q.x, q.y, q.z, q.w);
    // an all-zero quaternion is what an unset message field holds
    return n ? { x: q.x / n, y: q.y / n, z: q.z / n, w: q.w / n } : { x: 0, y: 0, z: 0, w: 1 };
}

function rotate(q, v) {
    const p = multiply(multiply(q, { x: v.x, y: v.y, z: v.z, w: 0 }), { x: -q.x, y: -q.y, z: -q.z, w: q.w });
    return { x: p.x, y: p.y, z: p.z };
}

/** a ∘ b: apply b first, then a. */
function compose(a, b) {
    const t = rotate(a.rotation, b.translation);
    return {
        translation: { x: a.translation.x + t.x, y: a.translation.y + t.y, z: a.translation.z + t.z },
        rotation: normalize(multiply(a.rotation, b.rotation))
    };
}

function inverse(a) {
    const q = { x: -a.rotation.x, y: -a.rotation.y, z: -a.rotation.z, w: a.rotation.w };
    const t = rotate(q, a.translation);
    return { translation: { x: -t.x, y: -t.y, z: -t.z }, rotation: q };
}

/** Roll, pitch, yaw (radians, ZYX convention as in tf2) of a quaternion. */
function toEuler(q) {
    const sinp = 2 * (q.w * q.y - q.z * q.x);
    return {
        roll: Math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)),
        pitch: Math.abs(sinp) >= 1 ? Math.sign(sinp) * Math.PI / 2 : Math.asin(sinp),
        yaw: Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    };
}

function stampMs(stamp) {
    return stamp ? (stamp.sec || 0) * 1000 + (stamp.nanosec || 0) / 1e6 : 0;
}

class TfBuffer {
    constructor() {
        this._edges = new Map(); // child -> {parent, transform, stamp, static, receivedAt}
    }

    get size() {
        return this._edges.size;
    }

    clear() {
        this._edges.clear();
    }

    /** Add the transforms of a tf2_msgs/TFMessage. */
    add(transforms, isStatic = false, now = Date.now()) {
        if (!Array.isArray(transforms)) return;
        for (const t of transforms) {
            if (!t || !t.transform) continue;
            const child = frameId(t.child_frame_id);
            const parent = frameId(t.header && t.header.frame_id);
            if (!child || !parent || child === parent) continue;
            const old = this._edges.get(child);
            const stamp = (t.header && t.header.stamp) || { sec: 0, nanosec: 0 };
            if (old && !isStatic && !old.static && old.parent === parent) {
                const behind = stampMs(old.stamp) - stampMs(stamp);
                if (behind > TIME_JUMP_MS) {
                    // like tf2: after a jump back the old transforms are worthless
                    for (const [frame, e] of this._edges) if (!e.static) this._edges.delete(frame);
                } else if (behind > 0) {
                    continue; // a late, older sample must not replace a newer one
                }
            }
            const tr = t.transform.translation || {};
            const rot = t.transform.rotation || {};
            this._edges.set(child, {
                parent,
                transform: {
                    translation: { x: tr.x || 0, y: tr.y || 0, z: tr.z || 0 },
                    rotation: normalize({ x: rot.x || 0, y: rot.y || 0, z: rot.z || 0, w: rot.w || 0 })
                },
                stamp,
                static: isStatic,
                receivedAt: now
            });
        }
    }

    /** [{frame, parent, static}] sorted by frame; roots have parent null. */
    frames() {
        const out = new Map();
        for (const [child, e] of this._edges) {
            out.set(child, { frame: child, parent: e.parent, static: e.static });
            if (!this._edges.has(e.parent) && !out.has(e.parent)) out.set(e.parent, { frame: e.parent, parent: null, static: true });
        }
        return [...out.values()].sort((a, b) => a.frame.localeCompare(b.frame));
    }

    _known(frame) {
        if (this._edges.has(frame)) return true;
        for (const e of this._edges.values()) if (e.parent === frame) return true;
        return false;
    }

    // Transform of `frame` in its root, with the edges used.
    _toRoot(frame) {
        let transform = IDENTITY;
        let current = frame;
        const edges = [];
        const seen = new Set([frame]);
        for (let e = this._edges.get(current); e; e = this._edges.get(current)) {
            transform = compose(e.transform, transform);
            edges.push(e);
            current = e.parent;
            if (seen.has(current)) throw new Error(`the TF tree has a loop at frame "${current}"`);
            seen.add(current);
        }
        return { root: current, transform, edges, chain: seen };
    }

    /**
     * Latest transform that maps points of `source` into `target`, as a
     * geometry_msgs/TransformStamped, plus `ageMs` (age of the oldest
     * non-static transform used, null if all were static) and `static`.
     */
    lookup(target, source, now = Date.now()) {
        const to = frameId(target);
        const from = frameId(source);
        if (!to || !from) throw new Error('lookup needs a target frame and a source frame');
        if (!this._edges.size) throw new Error('no transforms received yet on /tf or /tf_static');
        for (const frame of [to, from]) {
            if (this._known(frame)) continue;
            const similar = similarNames(frame, this.frames().map((f) => f.frame));
            const hint = similar.length ? ` — did you mean ${similar.join(', ')}?` : '';
            throw new Error(`frame "${frame}" is not in the TF tree${hint}`);
        }
        const a = this._toRoot(to);
        const b = this._toRoot(from);
        if (a.root !== b.root) {
            throw new Error(`"${to}" and "${from}" are not connected: their trees end at "${a.root}" and "${b.root}"`);
        }
        // only the edges below the common ancestor take part
        const used = [...a.edges.filter((e) => !b.edges.includes(e)), ...b.edges.filter((e) => !a.edges.includes(e))];
        const dynamic = used.filter((e) => !e.static);
        let stamp = { sec: 0, nanosec: 0 };
        let ageMs = null;
        for (const e of dynamic) {
            if (ageMs === null || stampMs(e.stamp) < stampMs(stamp)) stamp = e.stamp;
            ageMs = Math.max(ageMs === null ? 0 : ageMs, now - e.receivedAt);
        }
        return {
            header: { stamp: { sec: stamp.sec || 0, nanosec: stamp.nanosec || 0 }, frame_id: to },
            child_frame_id: from,
            transform: compose(inverse(a.transform), b.transform),
            ageMs,
            static: dynamic.length === 0
        };
    }
}

module.exports = { TfBuffer, compose, inverse, toEuler, frameId };
