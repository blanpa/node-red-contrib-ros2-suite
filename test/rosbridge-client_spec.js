'use strict';

const assert = require('assert');
const { RosbridgeClient, goalStatusName } = require('../lib/rosbridge-client');
const { MockRosbridge } = require('./mock-rosbridge');

function once(emitter, event, pred = () => true, timeout = 3000) {
    return new Promise((resolve, reject) => {
        const timer = setTimeout(() => {
            emitter.removeListener(event, handler);
            reject(new Error(`timed out waiting for ${event}`));
        }, timeout);
        function handler(...args) {
            if (!pred(...args)) return;
            clearTimeout(timer);
            emitter.removeListener(event, handler);
            resolve(args);
        }
        emitter.on(event, handler);
    });
}

const delay = (ms) => new Promise((r) => setTimeout(r, ms));

describe('RosbridgeClient', function () {
    let mock;
    let client;

    beforeEach(async function () {
        mock = new MockRosbridge();
        await mock.start();
        client = new RosbridgeClient({ url: mock.url, reconnectMin: 50, reconnectMax: 200, serviceTimeout: 500 });
    });

    afterEach(async function () {
        await client.close();
        await mock.stop();
    });

    async function connect() {
        const p = once(client, 'state', (s) => s === 'connected');
        client.connect();
        await p;
    }

    it('connects and reports states', async function () {
        const states = [];
        client.on('state', (s) => states.push(s));
        await connect();
        assert.deepStrictEqual(states, ['connecting', 'connected']);
        await client.close();
        assert.strictEqual(client.state, 'disconnected');
    });

    it('sends a bearer token when configured', async function () {
        await client.close();
        client = new RosbridgeClient({ url: mock.url, token: 's3cret' });
        await connect();
        assert.strictEqual(mock.lastHeaders.authorization, 'Bearer s3cret');
    });

    it('fans out one topic to several subscribers', async function () {
        await connect();
        const a = [];
        const b = [];
        const unA = client.subscribe('/chatter', (m) => a.push(m.data), { type: 'std_msgs/msg/String' });
        client.subscribe('/chatter', (m) => b.push(m.data));
        await mock.waitFor((f) => f.op === 'subscribe' && f.id === unA.id);
        await delay(20);
        mock.publish('/chatter', { data: 'hello' });
        await delay(50);
        assert.deepStrictEqual(a, ['hello']);
        assert.deepStrictEqual(b, ['hello']);

        unA();
        await mock.waitFor((f) => f.op === 'unsubscribe' && f.id === unA.id);
        mock.publish('/chatter', { data: 'again' });
        await delay(50);
        assert.deepStrictEqual(a, ['hello']);
        assert.deepStrictEqual(b, ['hello', 'again']);
    });

    it('replays subscriptions, advertisements and services after reconnect', async function () {
        await connect();
        const got = [];
        client.subscribe('/chatter', (m) => got.push(m.data), { type: 'std_msgs/msg/String', throttle_rate: 100 });
        const pub = client.advertise('/out', 'std_msgs/msg/String');
        client.advertiseService('/nr/trigger', 'std_srvs/srv/Trigger', (args, respond) => respond({ success: true, message: 'ok' }));

        const lost = once(client, 'state', (s) => s === 'disconnected');
        const back = once(client, 'state', (s) => s === 'connected');
        await mock.restart();
        await lost;
        mock.clearReceived();
        await back;

        const sub = await mock.waitFor((f) => f.op === 'subscribe' && f.topic === '/chatter');
        assert.strictEqual(sub.throttle_rate, 100);
        await mock.waitFor((f) => f.op === 'advertise' && f.topic === '/out');
        await mock.waitFor((f) => f.op === 'advertise_service' && f.service === '/nr/trigger');

        await delay(20);
        mock.publish('/chatter', { data: 'after' });
        await delay(50);
        assert.deepStrictEqual(got, ['after']);

        pub.publish({ data: 'x' });
        await mock.waitFor((f) => f.op === 'publish' && f.topic === '/out');
        const res = await mock.callClientService('/nr/trigger', {});
        assert.deepStrictEqual(res.values, { success: true, message: 'ok' });
    });

    it('fails pending calls and goals when the connection drops', async function () {
        await connect();
        const call = client.callService('/slow_service', {}, { timeout: 0 });
        let goalErr;
        mock.actions.get('/turtle1/rotate_absolute').hang = true;
        client.sendGoal('/turtle1/rotate_absolute', 'turtlesim/action/RotateAbsolute', { theta: 1 }, {
            onError: (e) => { goalErr = e; }
        });
        await mock.waitFor((f) => f.op === 'send_action_goal');
        mock.dropClients();
        await assert.rejects(call, /connection to rosbridge lost/);
        assert.match(goalErr.message, /connection to rosbridge lost/);
        assert.strictEqual(client.activeGoals, 0);
    });

    it('resolves service calls', async function () {
        await connect();
        const r = await client.callService('/rosapi/topics');
        assert.ok(r.topics.includes('/turtle1/pose'));
        const ok = await client.callService('/turtle1/teleport_absolute', { x: 1, y: 1, theta: 0 }, { type: 'turtlesim/srv/TeleportAbsolute' });
        assert.deepStrictEqual(ok, {});
        const frame = await mock.waitFor((f) => f.op === 'call_service' && f.service === '/turtle1/teleport_absolute');
        assert.strictEqual(frame.type, 'turtlesim/srv/TeleportAbsolute');
    });

    it('rejects failed service calls with the rosbridge message', async function () {
        await connect();
        await assert.rejects(client.callService('/failing_service'), /\/failing_service failed: boom/);
        await assert.rejects(client.callService('/nope'), /does not exist/);
    });

    it('times out service calls', async function () {
        await connect();
        const t0 = Date.now();
        await assert.rejects(client.callService('/slow_service', {}, { timeout: 100 }), (err) => {
            assert.strictEqual(err.code, 'TIMEOUT');
            assert.match(err.message, /no response from \/slow_service within 0.1 s/);
            return true;
        });
        assert.ok(Date.now() - t0 < 400);
        assert.strictEqual(client._calls.size, 0);
    });

    it('rejects immediately when not connected', async function () {
        await assert.rejects(client.callService('/x'), /not connected to rosbridge/);
        assert.throws(() => client.sendGoal('/a', 'p/action/A', {}), /not connected/);
    });

    it('routes status frames with an id to the waiting call', async function () {
        await connect();
        const statuses = [];
        client.on('status', (level, text, id) => statuses.push({ level, text, id }));
        const p = client.callService('/slow_service', {}, { timeout: 1000 });
        const frame = await mock.waitFor((f) => f.op === 'call_service' && f.service === '/slow_service');
        mock.sendStatus(frame.id, 'error', 'Unable to load the service type');
        await assert.rejects(p, /rosbridge rejected call to \/slow_service: Unable to load the service type/);
        assert.deepStrictEqual(statuses, [{ level: 'error', text: 'Unable to load the service type', id: frame.id }]);
    });

    it('delivers action feedback and result', async function () {
        await connect();
        const feedback = [];
        const result = await new Promise((resolve, reject) => {
            client.sendGoal('/turtle1/rotate_absolute', 'turtlesim/action/RotateAbsolute', { theta: 1.57 }, {
                onFeedback: (f) => feedback.push(f.remaining),
                onResult: resolve,
                onError: reject
            });
        });
        assert.deepStrictEqual(feedback, [1.0, 0.5, 0.1]);
        assert.deepStrictEqual(result, { status: 'succeeded', statusCode: 4, values: { delta: 1.2 } });
        const frame = await mock.waitFor((f) => f.op === 'send_action_goal');
        assert.strictEqual(frame.action_type, 'turtlesim/action/RotateAbsolute');
        assert.strictEqual(frame.feedback, true);
        assert.deepStrictEqual(frame.args, { theta: 1.57 });
    });

    it('cancels a goal', async function () {
        await connect();
        mock.actions.get('/turtle1/rotate_absolute').hang = true;
        const result = await new Promise((resolve, reject) => {
            const g = client.sendGoal('/turtle1/rotate_absolute', 'turtlesim/action/RotateAbsolute', { theta: 3 }, {
                onFeedback: () => g.cancel(),
                onResult: resolve,
                onError: reject
            });
        });
        assert.strictEqual(result.status, 'canceled');
        await mock.waitFor((f) => f.op === 'cancel_action_goal');
    });

    it('cancels and fails a goal on timeout', async function () {
        await connect();
        mock.actions.get('/turtle1/rotate_absolute').hang = true;
        const err = await new Promise((resolve) => {
            client.sendGoal('/turtle1/rotate_absolute', 'turtlesim/action/RotateAbsolute', { theta: 3 }, {
                timeout: 100,
                onResult: () => resolve(new Error('unexpected result')),
                onError: resolve
            });
        });
        assert.strictEqual(err.code, 'TIMEOUT');
        await mock.waitFor((f) => f.op === 'cancel_action_goal');
    });

    it('reports rejected goals as errors', async function () {
        await connect();
        mock.actions.get('/turtle1/rotate_absolute').reject = true;
        const err = await new Promise((resolve) => {
            client.sendGoal('/turtle1/rotate_absolute', 'turtlesim/action/RotateAbsolute', {}, { onError: resolve });
        });
        assert.match(err.message, /rejected/);
    });

    it('ref-counts advertisements and refuses conflicting types', async function () {
        await connect();
        const a = client.advertise('/out', 'std_msgs/msg/String');
        const b = client.advertise('/out', 'std_msgs/msg/String');
        assert.throws(() => client.advertise('/out', 'geometry_msgs/msg/Twist'), /already advertised as std_msgs\/msg\/String/);
        a.unadvertise();
        assert.ok(client._adverts.has('/out'));
        b.unadvertise();
        await mock.waitFor((f) => f.op === 'unadvertise' && f.topic === '/out');
    });

    it('maps goal status codes', function () {
        assert.deepStrictEqual([0, 1, 2, 3, 4, 5, 6, 9].map(goalStatusName),
            ['unknown', 'accepted', 'executing', 'canceling', 'succeeded', 'canceled', 'aborted', 'unknown']);
    });

    it('keeps retrying with backoff while rosbridge is down', async function () {
        await connect();
        const port = mock.port;
        await mock.stop();
        await once(client, 'state', (s) => s === 'disconnected');
        await delay(300);
        assert.ok(client._attempt >= 2, `expected several attempts, got ${client._attempt}`);
        mock = new MockRosbridge({ port });
        await mock.start();
        await once(client, 'state', (s) => s === 'connected', 3000);
    });
});
