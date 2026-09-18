'use strict';

const { statusSetter, pick, useConnection } = require('../lib/node-common');
const { fullType, similarNames } = require('../lib/type-registry');

module.exports = function (RED) {
    function Ros2ActionNode(config) {
        RED.nodes.createNode(this, config);
        const node = this;
        const setStatus = statusSetter(node);
        const conn = RED.nodes.getNode(config.connection);
        const configAction = (config.action || '').trim();
        const configType = (config.rosType || '').trim();
        const validation = ['strict', 'warn', 'off'].includes(config.validation) ? config.validation : 'warn';
        const allowOverride = !!config.allowOverride;
        const concurrent = !!config.concurrent;
        const timeoutSec = Number(config.timeout);
        const timeout = Number.isFinite(timeoutSec) && timeoutSec > 0 ? timeoutSec * 1000 : 0;

        if (!conn) {
            setStatus('error', 'no connection configured');
            node.on('input', (msg, send, done) => done(new Error('no ROS 2 connection configured on this node')));
            return;
        }

        const goals = new Map(); // goalId -> {action, handle}
        let lastOutcome = null;

        function statusEvent(event, goalId, action, extra) {
            // flat and stable, so state machines can switch on payload.event
            return { event, goalId, action, at: new Date().toISOString(), ...extra };
        }

        function showIdle() {
            if (goals.size) setStatus('busy', `${goals.size} goal${goals.size > 1 ? 's' : ''} running`);
            else if (lastOutcome) setStatus(lastOutcome === 'succeeded' ? 'ok' : 'warn', `idle · last: ${lastOutcome}`);
            else setStatus('idle', configAction ? `ready · ${configAction}` : 'ready');
        }

        async function resolveType(action, msg) {
            if (typeof msg.rosType === 'string' && msg.rosType.trim()) return msg.rosType.trim();
            if (configType) return configType;
            let type = null;
            let names = [];
            try {
                type = await conn.registry.actionType(action);
                if (!type) names = (await conn.registry.listActions()).map((a) => a.name);
            } catch (err) {
                throw new Error(`no action type for ${action} and rosapi could not tell (${err.message}) — set the type on the node or pass msg.rosType`);
            }
            if (!type) {
                const similar = similarNames(action, names);
                const hint = similar.length ? ` — did you mean ${similar.join(', ')}?` : '';
                throw new Error(`no action server for ${action}${hint} — start it, or set the type on the node to send anyway`);
            }
            return type;
        }

        async function check(type, goal) {
            if (validation === 'off') return;
            let result;
            try {
                result = await conn.registry.validate(type, goal, 'goal');
            } catch (_) {
                return;
            }
            for (const w of result.warnings) node.warn(w);
            if (!result.errors.length) return;
            const text = `invalid goal for ${type}: ${result.errors.join('; ')}`;
            if (validation === 'strict') throw Object.assign(new Error(text), { validation: result.errors });
            node.warn(text);
        }

        function cancel(msg, send, done) {
            const action = pick(msg.action, configAction, allowOverride);
            const ids = msg.goalId ? [msg.goalId] : [...goals.keys()];
            const targets = ids.filter((id) => goals.has(id));
            if (!targets.length) {
                done(msg.goalId ? new Error(`no running goal ${msg.goalId} to cancel`) : undefined);
                return;
            }
            for (const id of targets) {
                goals.get(id).handle.cancel();
                const status = RED.util.cloneMessage(msg);
                delete status.cancel;
                status.payload = statusEvent('cancel-requested', id, goals.get(id).action || action);
                send([null, null, status]);
            }
            done();
        }

        node.on('input', async (msg, send, done) => {
            if (msg.cancel === true) {
                cancel(msg, send, done);
                return;
            }
            const action = pick(msg.action, configAction, allowOverride);
            const fail = (err, goalId = null) => {
                const status = RED.util.cloneMessage(msg);
                status.payload = statusEvent('failed', goalId, action || null, { error: err.message });
                send([null, null, status]);
                setStatus('error', err.message);
                done(err);
            };
            try {
                if (!action) throw new Error('no action — set it on the node or pass msg.action');
                if (!concurrent && goals.size) {
                    throw new Error(`a goal is already running on ${action} — send msg.cancel = true first, or enable "concurrent goals"`);
                }
                if (!conn.client.connected) throw new Error(`not connected to rosbridge at ${conn.url} — goal not sent`);
                const type = await resolveType(action, msg);
                const goal = msg.payload === undefined || msg.payload === null || msg.payload === '' ? {} : msg.payload;
                await check(type, goal);

                const base = RED.util.cloneMessage(msg);
                let goalId = null;
                const out = (index, payload, ros) => {
                    const m = RED.util.cloneMessage(base);
                    m.payload = payload;
                    m.ros = { action, type, goalId, ...ros };
                    const arr = [null, null, null];
                    arr[index] = m;
                    send(arr);
                };
                const finish = () => {
                    goals.delete(goalId);
                    showIdle();
                };

                const handle = conn.client.sendGoal(action, fullType(type, 'action'), goal, {
                    timeout,
                    onFeedback: (values) => {
                        out(0, values, { event: 'feedback' });
                        out(2, statusEvent('feedback', goalId, action), { event: 'feedback' });
                    },
                    onResult: ({ status, statusCode, values }) => {
                        lastOutcome = status;
                        finish();
                        out(1, values, { status, statusCode });
                        const event = ['succeeded', 'aborted', 'canceled'].includes(status) ? status : 'failed';
                        const extra = { status };
                        if (event === 'failed') extra.error = `goal ended with status ${status}`;
                        out(2, statusEvent(event, goalId, action, extra), { status, statusCode });
                        if (event === 'failed') done(new Error(`goal on ${action} ended with status ${status}`));
                        else done();
                    },
                    onError: (err) => {
                        lastOutcome = 'failed';
                        finish();
                        out(2, statusEvent('failed', goalId, action, { error: err.message }), {});
                        setStatus('error', err.message);
                        done(err);
                    }
                });
                goalId = handle.goalId;
                goals.set(goalId, { action, handle });
                showIdle();
                out(2, statusEvent('sent', goalId, action), {});
            } catch (err) {
                fail(err);
            }
        });

        const release = useConnection(node, conn, setStatus, (state) => {
            if (state === 'connected') showIdle();
        });

        node.on('close', (done) => {
            for (const { handle } of goals.values()) handle.cancel();
            goals.clear();
            release();
            setStatus.clear();
            done();
        });
    }

    RED.nodes.registerType('ros2-action', Ros2ActionNode);
};
