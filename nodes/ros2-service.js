'use strict';

const { statusSetter, pick, useConnection, makeStamper, makeValidator } = require('../lib/node-common');
const { fullType, similarNames, unsafeService, encodeBuffers } = require('../lib/type-registry');

// Unanswered requests are kept until the timeout; with "no timeout" this caps them.
const MAX_OPEN_REQUESTS = 1000;

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

        const stamp = makeStamper(node, conn, config.stamp);

        const check = makeValidator(node, conn, validation);

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
                    let request = msg.payload === undefined || msg.payload === null || msg.payload === '' ? {} : encodeBuffers(msg.payload);
                    if (type) request = await stamp(type, 'request', request);
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
                stamp.close();
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
            let warnedOverflow = false;

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
                if (open.size > MAX_OPEN_REQUESTS) {
                    const [oldestId, oldest] = open.entries().next().value;
                    open.delete(oldestId);
                    clearTimeout(oldest.timer);
                    errorResponse('too many unanswered requests in the Node-RED flow').then((v) => oldest.respond(v, false));
                    if (!warnedOverflow) {
                        warnedOverflow = true;
                        node.warn(`${configService}: more than ${MAX_OPEN_REQUESTS} requests are waiting for an answer from the flow — the oldest ones are failed. Wire the response back into this node, or set a timeout`);
                    }
                }
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
                    done(new Error(`request ${ref.requestId} was already answered, timed out or lost with the connection`));
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
                        const response = msg.payload === undefined || msg.payload === null ? {} : encodeBuffers(msg.payload);
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
                if (state === 'connected') {
                    showIdle();
                    return;
                }
                // rosbridge fails the ROS callers when its client goes away, so
                // these requests cannot be answered any more
                if (!open.size) return;
                node.warn(`${configService}: ${open.size} open request${open.size > 1 ? 's were' : ' was'} lost with the connection to rosbridge`);
                for (const { timer } of open.values()) clearTimeout(timer);
                open.clear();
            });
            node.on('close', (done) => {
                for (const { respond, timer } of open.values()) {
                    clearTimeout(timer);
                    respond({}, false);
                }
                open.clear();
                stamp.close();
                if (server) server.unadvertise();
                release();
                setStatus.clear();
                done();
            });
        }
    }

    RED.nodes.registerType('ros2-service', Ros2ServiceNode);
};
