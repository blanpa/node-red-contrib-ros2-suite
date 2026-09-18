'use strict';

const LIST_TTL = 5000;
const ROSAPI_TIMEOUT = 5000;
const SERVICE_TYPE_BATCH = 12;

// ROS 2 field types, including the IDL spellings some rosapi versions report.
const PRIMITIVE_ALIASES = {
    bool: 'bool', boolean: 'bool',
    byte: 'byte', octet: 'byte', char: 'char',
    int8: 'int8', uint8: 'uint8', int16: 'int16', uint16: 'uint16',
    int32: 'int32', uint32: 'uint32', int64: 'int64', uint64: 'uint64',
    float32: 'float32', float: 'float32', float64: 'float64', double: 'float64',
    string: 'string', wstring: 'wstring', wchar: 'wstring',
    time: 'time', duration: 'duration'
};

const INT_RANGES = {
    byte: [0, 255], char: [0, 255],
    int8: [-128, 127], uint8: [0, 255],
    int16: [-32768, 32767], uint16: [0, 65535],
    int32: [-2147483648, 2147483647], uint32: [0, 4294967295],
    int64: [-(2 ** 63), 2 ** 63 - 1], uint64: [0, 2 ** 64 - 1]
};

const BINARY_ARRAY_BASES = new Set(['uint8', 'byte', 'char']);

/** `pkg/msg/Name`, `pkg/Name` and `pkg::msg::Name` all become `pkg/Name`. */
function normalizeType(type) {
    if (!type) return '';
    const parts = String(type).trim().replace(/::/g, '/').split('/').filter(Boolean);
    if (parts.length === 3) return `${parts[0]}/${parts[2]}`;
    return parts.join('/');
}

/** Expand to the three-part form rosbridge and rosapi expect for a kind (msg | srv | action). */
function fullType(type, kind = 'msg') {
    if (!type) return '';
    const parts = String(type).trim().replace(/::/g, '/').split('/').filter(Boolean);
    if (parts.length === 2) return `${parts[0]}/${kind}/${parts[1]}`;
    return parts.join('/');
}

function sameType(a, b) {
    return normalizeType(a) === normalizeType(b);
}

/**
 * Parse a rosapi field type into {base, arrayLen, primitive}.
 * arrayLen: -1 scalar, 0 variable-length, n fixed-length.
 */
function parseFieldType(raw, arrayLen = -1) {
    let t = String(raw).trim();
    let len = typeof arrayLen === 'number' ? arrayLen : -1;
    let m;
    if ((m = /^sequence<\s*([^,>]+?)\s*(?:,\s*\d+\s*)?>$/.exec(t))) {
        t = m[1];
        len = 0;
    } else if ((m = /^(.+?)\[(<=)?(\d*)\]$/.exec(t))) {
        t = m[1];
        len = m[2] || m[3] === '' ? 0 : Number(m[3]);
    }
    t = t.replace(/<=\d+$/, ''); // bounded strings: string<=10
    const primitive = PRIMITIVE_ALIASES[t];
    return { base: primitive || normalizeType(t), arrayLen: len, primitive: !!primitive };
}

function primitiveDefault(base) {
    switch (base) {
        case 'bool': return false;
        case 'string':
        case 'wstring': return '';
        case 'time':
        case 'duration': return { sec: 0, nanosec: 0 };
        default: return 0;
    }
}

function levenshtein(a, b) {
    if (a === b) return 0;
    const row = Array.from({ length: b.length + 1 }, (_, i) => i);
    for (let i = 1; i <= a.length; i++) {
        let prev = row[0];
        row[0] = i;
        for (let j = 1; j <= b.length; j++) {
            const tmp = row[j];
            row[j] = Math.min(row[j] + 1, row[j - 1] + 1, prev + (a[i - 1] === b[j - 1] ? 0 : 1));
            prev = tmp;
        }
    }
    return row[b.length];
}

/** Up to `max` names from `candidates` that look like `name`, best first. */
function similarNames(name, candidates, max = 3) {
    const target = String(name).toLowerCase();
    const leaf = target.split('/').pop();
    return candidates
        .map((c) => {
            const lc = String(c).toLowerCase();
            let score = levenshtein(target, lc);
            if (leaf && lc.split('/').pop() === leaf) score -= 3;
            else if (leaf && lc.includes(leaf)) score -= 1;
            return { c, score };
        })
        .filter(({ c, score }) => score <= Math.max(3, Math.ceil(Math.max(target.length, String(c).length) * 0.4)))
        .sort((a, b) => a.score - b.score)
        .slice(0, max)
        .map(({ c }) => c);
}

function describeValue(v) {
    if (v === null) return 'null';
    if (Array.isArray(v)) return 'array';
    if (typeof v === 'string') return `string ${JSON.stringify(v.length > 20 ? v.slice(0, 20) + '…' : v)}`;
    if (typeof v === 'number') return `number ${v}`;
    return typeof v;
}

function isPlainObject(v) {
    return v !== null && typeof v === 'object' && !Array.isArray(v) && !Buffer.isBuffer(v);
}

/**
 * Discovery and type definitions via rosapi, cached per connection.
 * kinds: msg | request | response | goal | result | feedback
 */
class TypeRegistry {
    constructor(client, opts = {}) {
        this.client = client;
        this.ttl = opts.ttl ?? LIST_TTL;
        this.timeout = opts.timeout ?? ROSAPI_TIMEOUT;
        this._lists = new Map();    // name -> {at, promise}
        this._typedefs = new Map(); // kind:type -> Promise<{root, defs}>
        this._serviceTypes = new Map();
        client.on('state', (state) => {
            if (state === 'connected') this.clear();
        });
    }

    clear() {
        this._lists.clear();
        this._typedefs.clear();
        this._serviceTypes.clear();
    }

    _rosapi(name, args = {}) {
        return this.client.callService(`/rosapi/${name}`, args, { timeout: this.timeout }).catch((err) => {
            if (err.code === 'TIMEOUT' || err.code === 'SERVICE_FAILED') {
                err.message = `rosapi did not answer /rosapi/${name} — is rosapi running next to rosbridge? (${err.message})`;
            }
            throw err;
        });
    }

    _cached(key, load) {
        const hit = this._lists.get(key);
        if (hit && Date.now() - hit.at < this.ttl) return hit.promise;
        const promise = load();
        this._lists.set(key, { at: Date.now(), promise });
        promise.catch(() => this._lists.delete(key));
        return promise;
    }

    // ---- discovery ---------------------------------------------------------

    listTopics() {
        return this._cached('topics', async () => {
            const r = await this._rosapi('topics');
            const topics = r.topics || [];
            const types = r.types || [];
            return topics.map((name, i) => ({ name, type: types[i] || '' }))
                .sort((a, b) => a.name.localeCompare(b.name));
        });
    }

    async topicType(topic) {
        const topics = await this.listTopics();
        const hit = topics.find((t) => t.name === topic);
        if (hit && hit.type) return hit.type;
        // rosapi may hide some topics from the list (e.g. hidden action topics)
        const r = await this._rosapi('topic_type', { topic });
        return r.type || null;
    }

    listServices() {
        return this._cached('services', async () => {
            const r = await this._rosapi('services');
            const names = (r.services || []).slice().sort();
            const out = [];
            for (let i = 0; i < names.length; i += SERVICE_TYPE_BATCH) {
                const batch = names.slice(i, i + SERVICE_TYPE_BATCH);
                const types = await Promise.all(batch.map((name) => this.serviceType(name).catch(() => '')));
                batch.forEach((name, j) => out.push({ name, type: types[j] || '' }));
            }
            return out;
        });
    }

    async serviceType(service) {
        if (this._serviceTypes.has(service)) return this._serviceTypes.get(service);
        const r = await this._rosapi('service_type', { service });
        const type = r.type || null;
        if (type) this._serviceTypes.set(service, type);
        return type;
    }

    listActions() {
        return this._cached('actions', async () => {
            const topics = await this.listTopics();
            let names;
            try {
                const r = await this._rosapi('action_servers');
                names = r.action_servers || [];
            } catch (err) {
                // Older rosapi without action_servers: derive from the hidden feedback topics.
                names = topics.filter((t) => t.name.endsWith('/_action/feedback'))
                    .map((t) => t.name.slice(0, -'/_action/feedback'.length));
            }
            const out = [];
            for (const name of names.slice().sort()) {
                out.push({ name, type: (await this._actionTypeFrom(name, topics).catch(() => null)) || '' });
            }
            return out;
        });
    }

    async actionType(action) {
        const topics = await this.listTopics();
        return this._actionTypeFrom(action, topics);
    }

    // rosapi has no action_type service, so infer it from `<action>/_action/feedback`
    // whose type is `pkg/action/Name_FeedbackMessage`.
    async _actionTypeFrom(action, topics) {
        const fbTopic = `${action}/_action/feedback`;
        let fbType = (topics.find((t) => t.name === fbTopic) || {}).type;
        if (!fbType) fbType = (await this._rosapi('topic_type', { topic: fbTopic })).type;
        if (!fbType) return null;
        return fullType(fbType.replace(/_FeedbackMessage$/, ''), 'action');
    }

    listNodes() {
        return this._cached('nodes', async () => {
            const r = await this._rosapi('nodes');
            return (r.nodes || []).slice().sort();
        });
    }

    // ---- type definitions --------------------------------------------------

    typedefs(type, kind = 'msg') {
        if (!type) return Promise.reject(new Error('no type given'));
        const key = `${kind}:${normalizeType(type)}`;
        if (!this._typedefs.has(key)) {
            const p = this._loadTypedefs(type, kind);
            this._typedefs.set(key, p);
            p.catch(() => this._typedefs.delete(key));
        }
        return this._typedefs.get(key);
    }

    async _loadTypedefs(type, kind) {
        let r;
        switch (kind) {
            case 'msg':
                r = await this._rosapi('message_details', { type: fullType(type, 'msg') });
                break;
            case 'request':
                r = await this._rosapi('service_request_details', { type: fullType(type, 'srv') });
                break;
            case 'response':
                r = await this._rosapi('service_response_details', { type: fullType(type, 'srv') });
                break;
            case 'goal':
            case 'result':
            case 'feedback': {
                const t = fullType(type, 'action');
                try {
                    r = await this._rosapi(`action_${kind}_details`, { type: t });
                } catch (err) {
                    const suffix = kind[0].toUpperCase() + kind.slice(1);
                    r = await this._rosapi('message_details', { type: `${t}_${suffix}` });
                }
                break;
            }
            default:
                throw new Error(`unknown type kind "${kind}"`);
        }
        const list = r.typedefs || [];
        if (!list.length) {
            throw new Error(`rosapi knows no type ${type} — check the spelling (e.g. geometry_msgs/msg/Twist) and that the package is installed where rosbridge runs`);
        }
        const defs = new Map();
        for (const d of list) defs.set(normalizeType(d.type), d);
        return { root: normalizeType(list[0].type), defs };
    }

    async template(type, kind = 'msg') {
        const { root, defs } = await this.typedefs(type, kind);
        return buildTemplate(root, defs, 0);
    }

    async validate(type, value, kind = 'msg') {
        const { root, defs } = await this.typedefs(type, kind);
        return validateValue(root, defs, value);
    }

    /** Field names of the root type, e.g. ['data'] for std_msgs/String. */
    async fieldNames(type, kind = 'msg') {
        const { root, defs } = await this.typedefs(type, kind);
        return (defs.get(root) || {}).fieldnames || [];
    }
}

function buildTemplate(typeName, defs, depth) {
    const def = defs.get(typeName);
    if (!def || depth > 32) return {};
    const out = {};
    def.fieldnames.forEach((field, i) => {
        const ft = parseFieldType(def.fieldtypes[i], def.fieldarraylen ? def.fieldarraylen[i] : -1);
        const one = () => (ft.primitive ? primitiveDefault(ft.base) : buildTemplate(ft.base, defs, depth + 1));
        if (ft.arrayLen === -1) out[field] = one();
        else if (ft.arrayLen === 0) out[field] = [];
        else out[field] = Array.from({ length: ft.arrayLen }, one);
    });
    return out;
}

function validateValue(root, defs, value) {
    const errors = [];
    const warnings = [];
    if (!isPlainObject(value)) {
        errors.push(`message must be an object with the fields of ${root}, got ${describeValue(value)}`);
        return { errors, warnings };
    }
    walkMessage(root, defs, value, '', errors, warnings, 0);
    return { errors, warnings };
}

function walkMessage(typeName, defs, value, path, errors, warnings, depth) {
    const def = defs.get(typeName);
    if (!def || depth > 32) return;
    const fields = def.fieldnames;
    for (const key of Object.keys(value)) {
        if (value[key] === undefined) continue;
        if (!fields.includes(key)) {
            const hint = similarNames(key, fields, 1);
            const suggestion = hint.length ? ` — did you mean "${hint[0]}"?` : '';
            errors.push(`${join(path, key)}: unknown field in ${typeName}${suggestion} (fields: ${fields.join(', ') || 'none'})`);
        }
    }
    fields.forEach((field, i) => {
        const v = value[field];
        if (v === undefined) return; // rosbridge fills missing fields with defaults
        const ft = parseFieldType(def.fieldtypes[i], def.fieldarraylen ? def.fieldarraylen[i] : -1);
        const p = join(path, field);
        if (ft.arrayLen === -1) {
            checkItem(ft, defs, v, p, errors, warnings, depth);
            return;
        }
        if (typeof v === 'string' && BINARY_ARRAY_BASES.has(ft.base)) {
            if (!/^[A-Za-z0-9+/]*={0,2}$/.test(v)) errors.push(`${p}: ${ft.base}[] given as string must be base64`);
            return;
        }
        if (Buffer.isBuffer(v)) {
            errors.push(`${p}: Buffer cannot be sent as JSON — use an array of numbers or a base64 string`);
            return;
        }
        if (!Array.isArray(v)) {
            errors.push(`${p}: expected ${ft.base}[] (array), got ${describeValue(v)}`);
            return;
        }
        if (ft.arrayLen > 0 && v.length !== ft.arrayLen) {
            errors.push(`${p}: expected exactly ${ft.arrayLen} items (${ft.base}[${ft.arrayLen}]), got ${v.length}`);
        }
        v.forEach((item, j) => checkItem(ft, defs, item, `${p}[${j}]`, errors, warnings, depth));
    });
}

function checkItem(ft, defs, v, path, errors, warnings, depth) {
    const base = ft.base;
    if (!ft.primitive || base === 'time' || base === 'duration') {
        if (!isPlainObject(v)) {
            errors.push(`${path}: expected ${base} (object), got ${describeValue(v)}`);
            return;
        }
        if (ft.primitive) return;
        walkMessage(base, defs, v, path, errors, warnings, depth + 1);
        return;
    }
    if (base === 'bool') {
        if (typeof v !== 'boolean') errors.push(`${path}: expected bool (true/false), got ${describeValue(v)}`);
        return;
    }
    if (base === 'string' || base === 'wstring') {
        if (typeof v !== 'string') errors.push(`${path}: expected ${base}, got ${describeValue(v)}`);
        return;
    }
    // numeric
    if (typeof v !== 'number') {
        errors.push(`${path}: expected ${base} (number), got ${describeValue(v)}`);
        return;
    }
    if (!Number.isFinite(v)) {
        errors.push(`${path}: ${v} cannot be sent — JSON has no NaN/Infinity`);
        return;
    }
    const range = INT_RANGES[base];
    if (!range) return; // float32/float64
    if (!Number.isInteger(v)) {
        errors.push(`${path}: expected integer ${base}, got ${v}`);
        return;
    }
    if (v < range[0] || v > range[1]) {
        errors.push(`${path}: ${v} is out of range for ${base} (${range[0]}..${range[1]})`);
        return;
    }
    if (!Number.isSafeInteger(v)) {
        warnings.push(`${path}: ${v} exceeds JavaScript's safe integer range; ${base} precision may be lost`);
    }
}

function join(path, key) {
    return path ? `${path}.${key}` : key;
}

module.exports = {
    TypeRegistry,
    normalizeType,
    fullType,
    sameType,
    parseFieldType,
    similarNames,
    buildTemplate,
    validateValue
};
