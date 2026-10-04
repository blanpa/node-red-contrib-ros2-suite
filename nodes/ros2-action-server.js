'use strict';

const { statusSetter, useConnection, makeValidator } = require('../lib/node-common');
const { fullType, encodeBuffers } = require('../lib/type-registry');

const OUTCOMES = ['succeeded', 'canceled', 'aborted'];

module.exports = function (RED) {
    function Ros2ActionServerNode(config) {
        RED.nodes.createNode(this, config);
        const node = this;
        const setStatus = statusSetter(node);
        const conn = RED.nodes.getNode(config.connection);
        const action = (config.action || '').trim();
        const type = (config.rosType || '').trim();
        const validation = ['strict', 'warn', 'off'].includes(config.validation) ? config.validation : 'warn';

        if (!conn) {
            setStatus('error', 'no connection configured');
            node.on('input', (msg, send, done) => done(new Error('no ROS 2 connection configured on this node')));
            return;
        }
        if (!action || !type) {
            setStatus('error', !action ? 'no action name configured' : 'the action type is required');
            node.on('input', (msg, send, done) => done(new Error('the action server needs an action name and an action type (e.g. example_interfaces/action/Fibonacci)')));
            return;
        }

        const goals = new Map(); // goalId -> goal handle of the client
        let served = 0;
        let server = null;

        function showIdle() {
            if (goals.size) setStatus('busy', `${goals.size} goal${goals.size > 1 ? 's' : ''} running`);
            else setStatus(served ? 'ok' : 'idle', served ? `served ${served} · ${action}` : `serving ${action}`);
        }

        const validate = makeValidator(node, conn, validation);
        const check = (value, kind) => validate(type, value, kind);

        const meta = (goalId) => ({
            action,
            ros: { action, type, goalId },
            _ros2: { replyTo: node.id, goalId }
        });

        try {
            server = conn.client.advertiseAction(action, fullType(type, 'action'), {
                onGoal: (args, goal) => {
                    goals.set(goal.id, goal);
                    showIdle();
                    node.send([{ payload: args, ...meta(goal.id) }, null]);
                },
                onCancel: (goalId) => {
                    node.send([null, { payload: { goalId }, ...meta(goalId) }]);
                },
                onLost: (goalId) => {
                    if (!goals.delete(goalId)) return;
                    node.warn(`${action}: goal ${goalId} was lost with the connection to rosbridge`);
                }
            });
        } catch (err) {
            setStatus('error', err.message);
            node.error(err.message);
        }

        node.on('input', async (msg, send, done) => {
            const ref = msg._ros2;
            if (!ref || ref.replyTo !== node.id) {
                done(new Error('the action server expects feedback or the result for a goal it emitted — wire the flow back into this node and keep msg._ros2'));
                return;
            }
            const goal = goals.get(ref.goalId);
            if (!goal) {
                done(new Error(`goal ${ref.goalId} has already ended or was lost with the connection`));
                return;
            }
            try {
                const values = msg.payload === undefined || msg.payload === null ? {} : encodeBuffers(msg.payload);
                if (msg.feedback === true) {
                    await check(values, 'feedback');
                    goal.feedback(values);
                    done();
                    return;
                }
                let outcome = OUTCOMES.includes(msg.status) ? msg.status : 'succeeded';
                if (msg.error) {
                    const text = typeof msg.error === 'string' ? msg.error : (msg.error.message || JSON.stringify(msg.error));
                    node.warn(`${action}: aborting goal ${ref.goalId}: ${text}`);
                    outcome = 'aborted';
                }
                if (outcome !== 'aborted') {
                    await check(values, 'result').catch((err) => {
                        goals.delete(ref.goalId);
                        goal.abort();
                        showIdle();
                        throw err;
                    });
                }
                if (!goals.delete(ref.goalId)) {
                    done(new Error(`goal ${ref.goalId} ended while its result was being checked`));
                    return;
                }
                if (outcome === 'succeeded') goal.succeed(values);
                else if (outcome === 'canceled') goal.cancel(values);
                else goal.abort();
                served++;
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
            for (const goal of goals.values()) goal.abort();
            goals.clear();
            if (server) server.unadvertise();
            release();
            setStatus.clear();
            done();
        });
    }

    RED.nodes.registerType('ros2-action-server', Ros2ActionServerNode);
};
