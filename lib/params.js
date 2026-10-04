'use strict';

// ROS 2 parameters through the parameter services every node offers
// (<node>/get_parameters, set_parameters, list_parameters, …).

// rcl_interfaces/msg/ParameterType
const TYPE_NAMES = [
    'not_set', 'bool', 'integer', 'double', 'string',
    'byte_array', 'bool_array', 'integer_array', 'double_array', 'string_array'
];

const VALUE_FIELDS = [
    null, 'bool_value', 'integer_value', 'double_value', 'string_value',
    'byte_array_value', 'bool_array_value', 'integer_array_value', 'double_array_value', 'string_array_value'
];

const SERVICE_TYPES = {
    get_parameters: 'rcl_interfaces/srv/GetParameters',
    get_parameter_types: 'rcl_interfaces/srv/GetParameterTypes',
    set_parameters: 'rcl_interfaces/srv/SetParameters',
    list_parameters: 'rcl_interfaces/srv/ListParameters',
    describe_parameters: 'rcl_interfaces/srv/DescribeParameters'
};

function typeName(code) {
    return TYPE_NAMES[code] || 'not_set';
}

/** Type code for a name ("double"), a code (3) or nothing (0). */
function typeCode(hint) {
    if (typeof hint === 'number') return TYPE_NAMES[hint] ? hint : 0;
    if (typeof hint !== 'string' || !hint.trim()) return 0;
    const code = TYPE_NAMES.indexOf(hint.trim().toLowerCase());
    if (code < 0) throw new Error(`unknown parameter type "${hint}" — use one of ${TYPE_NAMES.slice(1).join(', ')}`);
    return code;
}

/** `turtlesim` and `/turtlesim/` both become `/turtlesim`. */
function normalizeNode(name) {
    const parts = String(name || '').trim().split('/').filter(Boolean);
    return parts.length ? `/${parts.join('/')}` : '';
}

/** JavaScript value of a rcl_interfaces/msg/ParameterValue; undefined when not set. */
function fromParameterValue(pv) {
    const field = pv && VALUE_FIELDS[pv.type];
    if (!field) return undefined;
    const v = pv[field];
    // rosbridge sends byte[] as base64 or as a list, depending on the version
    if (pv.type === 5 && typeof v === 'string') return Buffer.from(v, 'base64');
    return v;
}

const isInt = (v) => typeof v === 'number' && Number.isInteger(v);
const isNum = (v) => typeof v === 'number' && Number.isFinite(v);

function guessType(value) {
    if (typeof value === 'boolean') return 1;
    if (isInt(value)) return 2;
    if (isNum(value)) return 3;
    if (typeof value === 'string') return 4;
    if (Buffer.isBuffer(value) || value instanceof Uint8Array) return 5;
    if (Array.isArray(value)) {
        if (!value.length) throw new Error('cannot tell the parameter type of an empty array — set the type (e.g. string_array)');
        if (value.every((v) => typeof v === 'boolean')) return 6;
        if (value.every(isInt)) return 7;
        if (value.every(isNum)) return 8;
        if (value.every((v) => typeof v === 'string')) return 9;
        throw new Error('parameter arrays must hold only booleans, only numbers or only strings');
    }
    throw new Error(`a parameter value must be a boolean, number, string or an array of those, got ${value === null ? 'null' : typeof value}`);
}

function fits(code, value) {
    switch (code) {
        case 1: return typeof value === 'boolean';
        case 2: return isInt(value);
        case 3: return isNum(value);
        case 4: return typeof value === 'string';
        case 5: return Buffer.isBuffer(value) || value instanceof Uint8Array ||
            (Array.isArray(value) && value.every((v) => isInt(v) && v >= 0 && v <= 255));
        case 6: return Array.isArray(value) && value.every((v) => typeof v === 'boolean');
        case 7: return Array.isArray(value) && value.every(isInt);
        case 8: return Array.isArray(value) && value.every(isNum);
        case 9: return Array.isArray(value) && value.every((v) => typeof v === 'string');
        default: return false;
    }
}

/**
 * Build a rcl_interfaces/msg/ParameterValue. JavaScript cannot tell 1.0 from 1,
 * so `hint` (the parameter's current or wanted type) decides between integer
 * and double; without it whole numbers become integers.
 */
function toParameterValue(value, hint) {
    const code = typeCode(hint) || guessType(value);
    if (!fits(code, value)) {
        const got = Array.isArray(value) ? 'array' : Buffer.isBuffer(value) ? 'Buffer' : `${typeof value} ${JSON.stringify(value)}`;
        throw new Error(`expected ${typeName(code)}, got ${got}`);
    }
    const out = { type: code };
    out[VALUE_FIELDS[code]] = code === 5 && !Array.isArray(value) ? Array.from(value) : value;
    return out;
}

function call(client, node, name, args, timeout) {
    return client.callService(`${normalizeNode(node)}/${name}`, args, { type: SERVICE_TYPES[name], timeout });
}

/** [{name, type, value}] — type "not_set" and value undefined for unknown parameters. */
async function getParameters(client, node, names, timeout) {
    const r = await call(client, node, 'get_parameters', { names }, timeout);
    const values = r.values || [];
    return names.map((name, i) => ({
        name,
        type: typeName(values[i] && values[i].type),
        value: fromParameterValue(values[i])
    }));
}

/** Type names in the order of `names`. */
async function getParameterTypes(client, node, names, timeout) {
    const r = await call(client, node, 'get_parameter_types', { names }, timeout);
    // `uint8[] types`: rosbridge sends it as base64
    const types = typeof r.types === 'string' ? Buffer.from(r.types, 'base64') : (r.types || []);
    return names.map((_, i) => typeName(types[i]));
}

/** entries: [{name, value: ParameterValue}] → [{name, successful, reason}] */
async function setParameters(client, node, entries, timeout) {
    const r = await call(client, node, 'set_parameters', { parameters: entries }, timeout);
    const results = r.results || [];
    return entries.map(({ name }, i) => ({
        name,
        successful: !!(results[i] && results[i].successful),
        reason: (results[i] && results[i].reason) || ''
    }));
}

/** [{name, type}], sorted by name. */
async function listParameters(client, node, timeout) {
    const r = await call(client, node, 'list_parameters', { prefixes: [], depth: 0 }, timeout);
    const names = ((r.result && r.result.names) || []).slice().sort();
    const types = names.length ? await getParameterTypes(client, node, names, timeout).catch(() => []) : [];
    return names.map((name, i) => ({ name, type: types[i] || '' }));
}

/** rcl_interfaces/msg/ParameterDescriptor for each name, plus `typeName`. */
async function describeParameters(client, node, names, timeout) {
    const r = await call(client, node, 'describe_parameters', { names }, timeout);
    return (r.descriptors || []).map((d) => ({ ...d, typeName: typeName(d.type) }));
}

module.exports = {
    TYPE_NAMES,
    typeName,
    typeCode,
    normalizeNode,
    fromParameterValue,
    toParameterValue,
    getParameters,
    getParameterTypes,
    setParameters,
    listParameters,
    describeParameters
};
