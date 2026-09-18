'use strict';

const { statusSetter, shortType, pick, useConnection } = require('../lib/node-common');
const { fullType, similarNames, unsafeService } = require('../lib/type-registry');

module.exports = function (RED) {
    function Ros2ServiceNode(config) {
        RED.nodes.createNode(this, config);
        const node = this;
        const setStatus = statusSetter(node);
        const conn = RED.nodes.getNode(config.connection);
        const mode = config.mode === 'server' ? 'server' : 'client';
        const configService = (config.service || '').trim();
        const configType = (config.rosType || '').trim();
        const validation = ['strict', 'warn', 'off'].includes(config.validation) ? config.validation : 'warn';
        const allowOverride = !!config.allowOverride;
        const timeoutSec = Number(config.timeout);

        if (!conn) {
            setStatus('error', 'no connection configured');
            node.on('input', (msg, send, done) => done(new Error('no ROS 2 connection configured on this node')));
            return;
        }
        const timeout = Number.isFinite(timeoutSec) && timeoutSec >= 0 && config.timeout !== ''
            ? timeoutSec * 1000
            : conn.serviceTimeout;

        async function check(type, value, kind) {
            if (validation === 'off' || !type) return;
            let result;
            try {
                result = await conn.registry.validate(type, value, kind);
            } catch (err) {
                return; // no definition available: let rosbridge decide
            }
            for (const w of result.warnings) node.warn(w);
            if (!result.errors.length) return;
            const text = `invalid ${kind} for ${shortType(type)}: ${result.errors.join('; ')}`;
            if (validation === 'strict') throw Object.assign(new Error(text), { validation: result.errors });
            node.warn(text);
        }

        if (mode === 'client') setupClient();
        else setupServer();

        // ---- client ------------------------------------------------------------

        function setupClient() {
            let pending = 0;
            let calls = 0;

            async function explain(service, err) {
                if (err.code !== 'TIMEOUT' && !/does not exist/i.test(err.message)) return err;
                try {
                    const names = (await conn.registry.listServices()).map((s) => s.name);
                    if (!names.includes(service)) {
                        const similar = similarNames(service, names);
                        const hint = similar.length ? ` — did you mean ${similar.join(', ')}?` : '';
                        err.message = `${service} is not offered by any ROS node${hint} (${err.message})`;
                    }
                } catch (_) { /* keep the original error */ }
                return err;
            }

            function showIdle(last) {
                if (pending) setStatus('busy', `${pending} call${pending > 1 ? 's' : ''} pending`);
                else if (last) setStatus('ok', last);
                else setStatus('idle', configService ? `ready · ${configService}` : 'ready');
            }

            node.on('input', async (msg, send, done) => {
                const service = pick(msg.service, configService, allowOverride);
                pending++;
                showIdle();
                const started = Date.now();
                try {
                    if (!service) throw new Error('no service — set it on the node or pass msg.service');
                    if (unsafeService(service)) throw new Error(unsafeService(service));
                    if (!conn.client.connected) throw new Error(`not connected to rosbridge at ${conn.url} — call to ${service} not sent`);
                    let type = (typeof msg.rosType === 'string' && msg.rosType.trim()) || configType || null;
                    if (!type) type = await conn.registry.serviceType(service).catch(() => null);
                    const request = msg.payload === undefined || msg.payload === null || msg.payload === '' ? {} : msg.payload;
                    await check(type, request, 'request');
                    const response = await conn.client.callService(service, request, {
                        type: type ? fullType(type, 'srv') : undefined,
                        timeout
                    }).catch((err) => explain(service, err).then((e) => { throw e; }));
                    const durationMs = Date.now() - started;
                    msg.payload = response;
                    msg.ros = { service, type, durationMs };
                    pending--;
                    calls++;
                    showIdle(`ok · ${durationMs} ms · ${calls} call${calls > 1 ? 's' : ''}`);
                    send(msg);
                    done();
                } catch (err) {
                    pending--;
                    setStatus('error', err.message);
                    done(err);
                }
            });

            const release = useConnection(node, conn, setStatus, (state) => {
                if (state === 'connected') showIdle();
            });
            node.on('close', (done) => {
                release();
                setStatus.clear();
                done();
            });
        }

        // ---- server ------------------------------------------------------------

        function setupServer() {
            const open = new Map(); // requestId -> {respond, timer}
            let seq = 0;
            let served = 0;
            let server = null;

            function showIdle() {
                if (open.size) setStatus('busy', `${open.size} request${open.size > 1 ? 's' : ''} open`);
                else setStatus(served ? 'ok' : 'idle', served ? `served ${served} · ${configService}` : `serving ${configService}`);
            }

            if (!configService || !configType) {
                setStatus('error', !configService ? 'no service name configured' : 'server mode needs the service type');
                node.on('input', (msg, send, done) => done(new Error('server mode needs a service name and a service type (e.g. std_srvs/srv/Trigger)')));
                return;
            }

            // rosbridge ignores the failure flag of a service_response, so the ROS
            // caller only ever sees a response. Carry the error in the common
            // success/message fields (std_srvs/Trigger, SetBool, …) when they exist.
            async function errorResponse(text) {
                const fields = await conn.registry.fieldNames(configType, 'response').catch(() => []);
                const values = {};
                if (fields.includes('success')) values.success = false;
                if (fields.includes('message')) values.message = text;
                return values;
            }

            function onRequest(args, respond) {
                const requestId = `${node.id}:${++seq}`;
                const entry = { respond, timer: null };
                if (timeout > 0) {
                    entry.timer = setTimeout(() => {
                        open.delete(requestId);
                        errorResponse(`no answer from the Node-RED flow within ${timeout / 1000} s`).then((v) => respond(v, false));
                        setStatus('warn', `request not answered within ${timeout / 1000} s`);
                        node.warn(`${configService}: the flow did not answer within ${timeout / 1000} s — wire the response back into this node and keep msg._ros2`);
                    }, timeout);
                }
                open.set(requestId, entry);
                showIdle();
                node.send({
                    payload: args,
                    service: configService,
                    ros: { service: configService, type: configType, requestId },
                    _ros2: { replyTo: node.id, requestId }
                });
            }

            try {
                server = conn.client.advertiseService(configService, fullType(configType, 'srv'), onRequest);
            } catch (err) {
                setStatus('error', err.message);
                node.error(err.message);
            }

            node.on('input', async (msg, send, done) => {
                const ref = msg._ros2;
                if (!ref || ref.replyTo !== node.id) {
                    done(new Error('server mode expects the reply to a request this node emitted — wire the end of the flow back into this node and keep msg._ros2'));
                    return;
                }
                const entry = open.get(ref.requestId);
                if (!entry) {
                    done(new Error(`request ${ref.requestId} was already answered or timed out`));
                    return;
                }
                open.delete(ref.requestId);
                clearTimeout(entry.timer);
                try {
                    if (msg.error) {
                        const text = typeof msg.error === 'string' ? msg.error : (msg.error.message || JSON.stringify(msg.error));
                        entry.respond(await errorResponse(text), false);
                        node.warn(`${configService}: answered request with an error: ${text}`);
                    } else {
                        const response = msg.payload === undefined || msg.payload === null ? {} : msg.payload;
                        await check(configType, response, 'response').catch(async (err) => {
                            entry.respond(await errorResponse(err.message), false);
                            throw err;
                        });
                        entry.respond(response, true);
                        served++;
                    }
                    showIdle();
                    done();
                } catch (err) {
                    setStatus('error', err.message);
                    done(err);
                }
            });

            const release = useConnection(node, conn, setStatus, (state) => {
                if (state === 'connected') showIdle();
            });
            node.on('close', (done) => {
                for (const { respond, timer } of open.values()) {
                    clearTimeout(timer);
                    respond({}, false);
                }
                open.clear();
                if (server) server.unadvertise();
                release();
                setStatus.clear();
                done();
            });
        }
    }

    RED.nodes.registerType('ros2-service', Ros2ServiceNode);
};
