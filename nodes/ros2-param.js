'use strict';

const { statusSetter, pick, useConnection } = require('../lib/node-common');
const { similarNames } = require('../lib/type-registry');
const params = require('../lib/params');

const OPERATIONS = ['get', 'set', 'list', 'describe'];

function isPlainObject(v) {
    return v !== null && typeof v === 'object' && !Array.isArray(v) && !Buffer.isBuffer(v);
}

module.exports = function (RED) {
    function Ros2ParamNode(config) {
        RED.nodes.createNode(this, config);
        const node = this;
        const setStatus = statusSetter(node);
        const conn = RED.nodes.getNode(config.connection);
        const configNode = params.normalizeNode(config.node);
        const configOperation = OPERATIONS.includes(config.operation) ? config.operation : 'get';
        const configParam = (config.param || '').trim();
        const configType = (config.paramType || '').trim();
        const allowOverride = !!config.allowOverride;

        if (!conn) {
            setStatus('error', 'no connection configured');
            node.on('input', (msg, send, done) => done(new Error('no ROS 2 connection configured on this node')));
            return;
        }
        const timeout = conn.serviceTimeout;

        /** Parameter names from msg.param (string or array) or the config, and whether one name was asked for. */
        function paramNames(msg) {
            const fromMsg = Array.isArray(msg.param)
                ? msg.param.filter((p) => typeof p === 'string' && p.trim()).map((p) => p.trim())
                : (typeof msg.param === 'string' && msg.param.trim() ? [msg.param.trim()] : []);
            const useMsg = fromMsg.length && (!configParam || allowOverride);
            if (useMsg) return { names: fromMsg, single: !Array.isArray(msg.param) };
            return { names: configParam ? [configParam] : [], single: true };
        }

        async function explain(rosNode, err) {
            if (err.code !== 'TIMEOUT' && !/does not exist/i.test(err.message)) return err;
            try {
                const nodes = await conn.registry.listNodes();
                if (!nodes.includes(rosNode)) {
                    const similar = similarNames(rosNode, nodes);
                    const hint = similar.length ? ` — did you mean ${similar.join(', ')}?` : '';
                    err.message = `ROS node ${rosNode} is not running${hint} (${err.message})`;
                }
            } catch (_) { /* keep the original error */ }
            return err;
        }

        async function unknownParameters(rosNode, missing) {
            let hint = '';
            try {
                const known = (await params.listParameters(conn.client, rosNode, timeout)).map((p) => p.name);
                const similar = [...new Set(missing.flatMap((name) => similarNames(name, known)))];
                if (similar.length) hint = ` — did you mean ${similar.join(', ')}?`;
            } catch (_) { /* suggestions are best effort */ }
            const plural = missing.length > 1 ? 's' : '';
            return new Error(`${rosNode} has no parameter${plural} ${missing.join(', ')}${hint}`);
        }

        async function get(rosNode, msg) {
            const { names, single } = paramNames(msg);
            if (!names.length) throw new Error('no parameter — set it on the node or pass msg.param ("list" returns all names)');
            const values = await params.getParameters(conn.client, rosNode, names, timeout);
            const missing = values.filter((v) => v.type === 'not_set').map((v) => v.name);
            if (missing.length) throw await unknownParameters(rosNode, missing);
            if (single) {
                msg.payload = values[0].value;
                msg.ros = { node: rosNode, operation: 'get', param: names[0], type: values[0].type };
                return `${names[0]} = ${JSON.stringify(values[0].value)}`;
            }
            msg.payload = Object.fromEntries(values.map((v) => [v.name, v.value]));
            msg.ros = { node: rosNode, operation: 'get', param: names, types: Object.fromEntries(values.map((v) => [v.name, v.type])) };
            return `got ${names.length} parameters`;
        }

        async function set(rosNode, msg) {
            const { names } = paramNames(msg);
            let wanted;
            if (names.length === 1) {
                if (msg.payload === undefined) throw new Error(`no value for ${names[0]} — pass it as msg.payload`);
                wanted = [[names[0], msg.payload]];
            } else if (!names.length && isPlainObject(msg.payload) && Object.keys(msg.payload).length) {
                wanted = Object.entries(msg.payload);
            } else {
                throw new Error('set needs one parameter name with the value in msg.payload, or no name and msg.payload = {name: value, …}');
            }

            const forced = (typeof msg.paramType === 'string' && msg.paramType.trim()) || configType;
            let current = [];
            if (!forced) {
                // the declared type decides whether 1 is sent as integer or double
                current = await params.getParameterTypes(conn.client, rosNode, wanted.map(([name]) => name), timeout).catch(() => []);
            }
            const entries = wanted.map(([name, value], i) => {
                try {
                    return { name, value: params.toParameterValue(value, forced || current[i]) };
                } catch (err) {
                    throw new Error(`${name}: ${err.message}`, { cause: err });
                }
            });
            const results = await params.setParameters(conn.client, rosNode, entries, timeout);
            const failed = results.filter((r) => !r.successful);
            if (failed.length) {
                throw new Error(`${rosNode} rejected ${failed.map((r) => `${r.name}${r.reason ? ` (${r.reason})` : ''}`).join(', ')}`);
            }
            const types = Object.fromEntries(entries.map((e) => [e.name, params.typeName(e.value.type)]));
            msg.ros = wanted.length === 1 && names.length
                ? { node: rosNode, operation: 'set', param: names[0], type: types[names[0]] }
                : { node: rosNode, operation: 'set', param: Object.keys(types), types };
            return wanted.length === 1 ? `${wanted[0][0]} := ${JSON.stringify(wanted[0][1])}` : `set ${wanted.length} parameters`;
        }

        async function list(rosNode, msg) {
            msg.payload = await params.listParameters(conn.client, rosNode, timeout);
            msg.ros = { node: rosNode, operation: 'list' };
            return `${msg.payload.length} parameters`;
        }

        async function describe(rosNode, msg) {
            const { names, single } = paramNames(msg);
            if (!names.length) throw new Error('no parameter — set it on the node or pass msg.param');
            const descriptors = await params.describeParameters(conn.client, rosNode, names, timeout);
            msg.payload = single ? descriptors[0] : descriptors;
            msg.ros = { node: rosNode, operation: 'describe', param: single ? names[0] : names };
            return `described ${names.join(', ')}`;
        }

        const run = { get, set, list, describe };

        node.on('input', async (msg, send, done) => {
            const rosNode = params.normalizeNode(pick(msg.node, configNode, allowOverride));
            const operation = OPERATIONS.includes(msg.operation) ? msg.operation : configOperation;
            try {
                if (!rosNode) throw new Error('no ROS node — set it on the node or pass msg.node (e.g. /turtlesim)');
                if (!conn.client.connected) throw new Error(`not connected to rosbridge at ${conn.url}`);
                setStatus('busy', `${operation}…`);
                const text = await run[operation](rosNode, msg).catch((err) => explain(rosNode, err).then((e) => { throw e; }));
                setStatus('ok', text);
                send(msg);
                done();
            } catch (err) {
                setStatus('error', err.message);
                done(err);
            }
        });

        const release = useConnection(node, conn, setStatus, (state) => {
            if (state === 'connected') setStatus('idle', configNode ? `ready · ${configNode}` : 'ready');
        });
        node.on('close', (done) => {
            release();
            setStatus.clear();
            done();
        });
    }

    RED.nodes.registerType('ros2-param', Ros2ParamNode);
};
