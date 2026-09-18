'use strict';

const assert = require('assert');
const helper = require('node-red-node-test-helper');
const { MockRosbridge } = require('./mock-rosbridge');

const connectionNode = require('../nodes/ros2-connection.js');
const subscribeNode = require('../nodes/ros2-subscribe.js');
const publishNode = require('../nodes/ros2-publish.js');
const serviceNode = require('../nodes/ros2-service.js');
const actionNode = require('../nodes/ros2-action.js');
const browseNode = require('../nodes/ros2-browse.js');

const ALL = [connectionNode, subscribeNode, publishNode, serviceNode, actionNode, browseNode];

helper.init(require.resolve('node-red'));

const delay = (ms) => new Promise((r) => setTimeout(r, ms));

async function until(fn, timeout = 2000, what = 'condition') {
    const end = Date.now() + timeout;
    while (Date.now() < end) {
        if (fn()) return;
        await delay(10);
    }
    throw new Error(`timed out waiting for ${what}`);
}

function nextInput(node, timeout = 2000) {
    return new Promise((resolve, reject) => {
        const t = setTimeout(() => reject(new Error(`no message on ${node.id}`)), timeout);
        node.once('input', (msg) => { clearTimeout(t); resolve(msg); });
    });
}

function collect(node) {
    const msgs = [];
    node.on('input', (msg) => msgs.push(msg));
    return msgs;
}

function calls(node, method) {
    const out = [];
    node.on(`call:${method}`, (call) => out.push(call.args[0]));
    return out;
}

describe('nodes', function () {
    let mock;

    before(async function () {
        mock = new MockRosbridge();
        await mock.start();
    });

    after(async function () {
        await mock.stop();
    });

    beforeEach(function (done) {
        mock.clearReceived();
        helper.startServer(done);
    });

    afterEach(async function () {
        await helper.unload();
        await new Promise((r) => helper.stopServer(r));
    });

    function conn(extra = {}) {
        return { id: 'c1', type: 'ros2-connection', host: '127.0.0.1', port: mock.port, reconnectMin: 0.05, reconnectMax: 0.2, ...extra };
    }

    async function load(flow) {
        await helper.load(ALL, flow);
        const c = helper.getNode('c1');
        await until(() => c.client.connected, 2000, 'connection');
        return c;
    }

    describe('loading', function () {
        it('loads every node', async function () {
            await helper.load(ALL, [
                conn(),
                { id: 'sub', type: 'ros2-subscribe', connection: 'c1', topic: '/chatter', wires: [[]] },
                { id: 'pub', type: 'ros2-publish', connection: 'c1', topic: '/chatter', wires: [[]] },
                { id: 'svc', type: 'ros2-service', connection: 'c1', service: '/reset', wires: [[]] },
                { id: 'act', type: 'ros2-action', connection: 'c1', action: '/turtle1/rotate_absolute', wires: [[], [], []] },
                { id: 'brw', type: 'ros2-browse', connection: 'c1', wires: [[]] }
            ]);
            for (const id of ['c1', 'sub', 'pub', 'svc', 'act', 'brw']) assert.ok(helper.getNode(id), `${id} loaded`);
        });

        it('shows an error status without a connection', async function () {
            await helper.load(ALL, [{ id: 'sub', type: 'ros2-subscribe', topic: '/chatter', wires: [[]] }]);
            const sub = helper.getNode('sub');
            // status was set during construction, before a spy could attach
            assert.ok(sub.status.calledWithMatch({ fill: 'red', text: 'no connection configured' }));
        });
    });

    describe('ros2-connection admin endpoints', function () {
        it('connects on demand, lists topics and builds templates', async function () {
            await helper.load(ALL, [conn()]);
            assert.strictEqual(helper.getNode('c1').client.state, 'disconnected'); // lazy until used
            const topics = await helper.request().get('/ros2-suite/c1/topics').expect(200);
            assert.ok(topics.body.find((t) => t.name === '/turtle1/cmd_vel' && t.type === 'geometry_msgs/msg/Twist'));
            const tpl = await helper.request().get('/ros2-suite/c1/template?type=geometry_msgs/msg/Twist&kind=msg').expect(200);
            assert.deepStrictEqual(tpl.body.template, { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } });
            const state = await helper.request().get('/ros2-suite/c1/state').expect(200);
            assert.strictEqual(state.body.state, 'connected');
            await helper.request().get('/ros2-suite/nope/topics').expect(404);
            assert.strictEqual(require('../nodes/ros2-connection.js').buildUrl({ host: 'r', port: 1, tls: 'true', path: 'rb' }), 'wss://r:1/rb');
        });
    });

    describe('ros2-connection probe (undeployed connections)', function () {
        it('lists topics with settings posted by the editor', async function () {
            await helper.load(ALL, []);
            const res = await helper.request().post('/ros2-suite/probe/topics')
                .send({ host: '127.0.0.1', port: mock.port, tls: false }).expect(200);
            assert.ok(res.body.some((t) => t.name === '/turtle1/pose'));
            const tpl = await helper.request().post('/ros2-suite/probe/template')
                .send({ url: mock.url, type: 'std_msgs/msg/String' }).expect(200);
            assert.deepStrictEqual(tpl.body.template, { data: '' });
        });

        it('falls back from the id route with a notDeployed marker and rejects bad URLs', async function () {
            await helper.load(ALL, []);
            const miss = await helper.request().get('/ros2-suite/abc123/topics').expect(404);
            assert.strictEqual(miss.body.notDeployed, true);
            const bad = await helper.request().post('/ros2-suite/probe/topics').send({ url: 'http://x' }).expect(503);
            assert.match(bad.body.error, /not a WebSocket URL/);
        });
    });

    describe('ros2-subscribe', function () {
        it('emits payload, topic and ros metadata with the detected type', async function () {
            await load([
                conn(),
                { id: 'sub', type: 'ros2-subscribe', connection: 'c1', topic: '/turtle1/pose', throttle: 50, wires: [['out']] },
                { id: 'out', type: 'helper' }
            ]);
            const frame = await mock.waitFor((f) => f.op === 'subscribe' && f.topic === '/turtle1/pose');
            assert.strictEqual(frame.type, 'turtlesim/msg/Pose');
            assert.strictEqual(frame.throttle_rate, 50);
            const got = nextInput(helper.getNode('out'));
            mock.publish('/turtle1/pose', { x: 1, y: 2, theta: 0, linear_velocity: 0, angular_velocity: 0 });
            const msg = await got;
            assert.strictEqual(msg.payload.x, 1);
            assert.strictEqual(msg.topic, '/turtle1/pose');
            assert.strictEqual(msg.ros.type, 'turtlesim/msg/Pose');
            assert.ok(msg.ros.receivedAt > 0);
        });

        it('suggests similar topics when the topic is not advertised', async function () {
            await helper.load(ALL, [
                conn(),
                { id: 'sub', type: 'ros2-subscribe', connection: 'c1', topic: '/turtle1/poses', wires: [[]] }
            ]);
            const sub = helper.getNode('sub');
            const warns = calls(sub, 'warn');
            await until(() => warns.length > 0, 2000, 'warning');
            assert.match(warns[0], /\/turtle1\/poses is not advertised by any ROS node yet \(did you mean \/turtle1\/pose/);
            assert.ok(sub.status.calledWithMatch({ fill: 'yellow', text: sinonMatch(/^topic not advertised \(did you mean/) }));
            assert.ok(!mock.received.some((f) => f.op === 'subscribe' && f.topic === '/turtle1/poses'));
        });

        it('reports a type mismatch instead of subscribing', async function () {
            await helper.load(ALL, [
                conn(),
                { id: 'sub', type: 'ros2-subscribe', connection: 'c1', topic: '/chatter', rosType: 'geometry_msgs/msg/Twist', wires: [[]] }
            ]);
            const sub = helper.getNode('sub');
            const errors = calls(sub, 'error');
            await until(() => errors.length > 0, 2000, 'error');
            assert.match(errors[0], /type mismatch: \/chatter is std_msgs\/msg\/String, node expects geometry_msgs\/msg\/Twist/);
        });

        it('explains QoS when an advertised topic stays silent', async function () {
            this.timeout(8000);
            await helper.load(ALL, [
                conn(),
                { id: 'sub', type: 'ros2-subscribe', connection: 'c1', topic: '/chatter', wires: [[]] }
            ]);
            const sub = helper.getNode('sub');
            const warns = calls(sub, 'warn');
            await until(() => warns.length > 0, 7000, 'QoS warning');
            assert.match(warns[0], /never receives VOLATILE or BEST_EFFORT publishers that joined later/);
            assert.ok(sub.status.calledWithMatch({ text: 'no data — check QoS / publisher' }));
        });

        it('keeps diagnosing until a late publisher shows up', async function () {
            this.timeout(22000);
            await helper.load(ALL, [
                conn(),
                { id: 'sub', type: 'ros2-subscribe', connection: 'c1', topic: '/late', rosType: 'std_msgs/msg/String', wires: [[]] }
            ]);
            const sub = helper.getNode('sub');
            await until(() => sub.status.calledWithMatch({ text: sinonMatch(/^topic not advertised/) }), 7000, 'missing status');
            mock.topics.push({ name: '/late', type: 'std_msgs/msg/String' });
            try {
                // next check within 5 s, topic list cached up to 5 s
                await until(() => sub.status.calledWithMatch({ text: 'no data — check QoS / publisher' }), 12000, 'QoS status');
            } finally {
                mock.topics.pop();
            }
        });

        it('shows the message rate', async function () {
            this.timeout(4000);
            await load([
                conn(),
                { id: 'sub', type: 'ros2-subscribe', connection: 'c1', topic: '/chatter', wires: [[]] }
            ]);
            await mock.waitFor((f) => f.op === 'subscribe' && f.topic === '/chatter');
            const timer = setInterval(() => mock.publish('/chatter', { data: 'x' }), 50);
            try {
                const sub = helper.getNode('sub');
                await until(() => sub.status.calledWithMatch({ fill: 'green', text: sinonMatch(/Hz · std_msgs\/String$/) }), 2500, 'Hz status');
            } finally {
                clearInterval(timer);
            }
        });
    });

    describe('ros2-publish', function () {
        it('publishes with the detected type and wraps plain values', async function () {
            await load([
                conn(),
                { id: 'pub', type: 'ros2-publish', connection: 'c1', topic: '/chatter', validation: 'strict', wires: [['out']] },
                { id: 'out', type: 'helper' }
            ]);
            const out = nextInput(helper.getNode('out'));
            helper.getNode('pub').receive({ payload: 'hello' });
            const msg = await out;
            assert.deepStrictEqual(msg.ros, { topic: '/chatter', type: 'std_msgs/msg/String' });
            const adv = await mock.waitFor((f) => f.op === 'advertise' && f.topic === '/chatter');
            assert.strictEqual(adv.type, 'std_msgs/msg/String');
            const pubFrame = await mock.waitFor((f) => f.op === 'publish' && f.topic === '/chatter');
            assert.deepStrictEqual(pubFrame.msg, { data: 'hello' });
        });

        it('blocks invalid messages in strict mode via done(err)', async function () {
            await load([
                conn(),
                { id: 'pub', type: 'ros2-publish', connection: 'c1', topic: '/turtle1/cmd_vel', rosType: 'geometry_msgs/msg/Twist', validation: 'strict', wires: [['out']] },
                { id: 'out', type: 'helper' }
            ]);
            const pub = helper.getNode('pub');
            const sent = collect(helper.getNode('out'));
            const errors = calls(pub, 'error');
            pub.receive({ payload: { linear: { x: 'fast' } } });
            await until(() => errors.length > 0, 2000, 'error');
            assert.match(String(errors[0]), /invalid geometry_msgs\/Twist: linear\.x: expected float64 \(number\), got string "fast"/);
            await delay(300);
            assert.strictEqual(sent.length, 0);
            assert.ok(!mock.received.some((f) => f.op === 'publish' && f.topic === '/turtle1/cmd_vel'));
        });

        it('warns and sends in warn mode', async function () {
            await load([
                conn(),
                { id: 'pub', type: 'ros2-publish', connection: 'c1', topic: '/turtle1/cmd_vel', validation: 'warn', wires: [['out']] },
                { id: 'out', type: 'helper' }
            ]);
            const pub = helper.getNode('pub');
            const warns = calls(pub, 'warn');
            const out = nextInput(helper.getNode('out'));
            pub.receive({ payload: { linear: { x: 1 }, sideways: 2 } });
            await out;
            assert.match(warns[0], /sideways: unknown field .* publishing anyway/);
            await mock.waitFor((f) => f.op === 'publish' && f.topic === '/turtle1/cmd_vel');
        });

        it('asks for a type when nothing advertises the topic', async function () {
            await load([
                conn(),
                { id: 'pub', type: 'ros2-publish', connection: 'c1', topic: '/new_topic', wires: [[]] }
            ]);
            const pub = helper.getNode('pub');
            const errors = calls(pub, 'error');
            pub.receive({ payload: {} });
            await until(() => errors.length > 0, 2000, 'error');
            assert.match(String(errors[0]), /no message type for \/new_topic — nothing advertises it yet, so set the type on the node/);
        });

        it('ignores msg.topic unless overriding is enabled', async function () {
            await load([
                conn(),
                { id: 'pub', type: 'ros2-publish', connection: 'c1', topic: '/chatter', wires: [['out']] },
                { id: 'out', type: 'helper' }
            ]);
            const out = nextInput(helper.getNode('out'));
            helper.getNode('pub').receive({ topic: '/turtle1/pose', payload: { data: 'x' } });
            assert.strictEqual((await out).ros.topic, '/chatter');
        });
    });

    describe('ros2-service', function () {
        it('calls a service in client mode', async function () {
            await load([
                conn(),
                { id: 'svc', type: 'ros2-service', connection: 'c1', mode: 'client', service: '/turtle1/teleport_absolute', wires: [['out']] },
                { id: 'out', type: 'helper' }
            ]);
            const out = nextInput(helper.getNode('out'));
            helper.getNode('svc').receive({ payload: { x: 5.5, y: 5.5, theta: 0 } });
            const msg = await out;
            assert.deepStrictEqual(msg.payload, {});
            assert.strictEqual(msg.ros.type, 'turtlesim/srv/TeleportAbsolute');
            const frame = await mock.waitFor((f) => f.op === 'call_service' && f.service === '/turtle1/teleport_absolute');
            assert.deepStrictEqual(frame.args, { x: 5.5, y: 5.5, theta: 0 });
        });

        it('fails with a suggestion for a missing service', async function () {
            await load([
                conn(),
                { id: 'svc', type: 'ros2-service', connection: 'c1', service: '/turtle1/teleport_absolut', wires: [[]] }
            ]);
            const svc = helper.getNode('svc');
            const errors = calls(svc, 'error');
            svc.receive({ payload: {} });
            await until(() => errors.length > 0, 2000, 'error');
            assert.match(String(errors[0]), /not offered by any ROS node — did you mean \/turtle1\/teleport_absolute/);
        });

        it('refuses services that crash rosapi', async function () {
            await load([
                conn(),
                { id: 'svc', type: 'ros2-service', connection: 'c1', service: '/rosapi/action_type', wires: [[]] }
            ]);
            const svc = helper.getNode('svc');
            const errors = calls(svc, 'error');
            svc.receive({ payload: { action: '/x' } });
            await until(() => errors.length > 0, 2000, 'error');
            assert.match(String(errors[0]), /crashes the rosapi node/);
            assert.ok(!mock.received.some((f) => f.service === '/rosapi/action_type'));
        });

        it('times out', async function () {
            await load([
                conn(),
                { id: 'svc', type: 'ros2-service', connection: 'c1', service: '/slow_service', timeout: 0.1, wires: [[]] }
            ]);
            const svc = helper.getNode('svc');
            const errors = calls(svc, 'error');
            svc.receive({ payload: {} });
            await until(() => errors.length > 0, 2000, 'error');
            assert.match(String(errors[0]), /no response from \/slow_service within 0.1 s/);
        });

        it('serves requests in server mode', async function () {
            await load([
                conn(),
                { id: 'srv', type: 'ros2-service', connection: 'c1', mode: 'server', service: '/nr/trigger', rosType: 'std_srvs/srv/Trigger', wires: [['work']] },
                { id: 'work', type: 'helper' }
            ]);
            await mock.waitFor((f) => f.op === 'advertise_service' && f.service === '/nr/trigger');
            const srv = helper.getNode('srv');
            helper.getNode('work').on('input', (msg) => {
                msg.payload = { success: true, message: `hello ${msg.payload.who || ''}`.trim() };
                srv.receive(msg);
            });
            const res = await mock.callClientService('/nr/trigger', {});
            assert.strictEqual(res.result, true);
            assert.deepStrictEqual(res.values, { success: true, message: 'hello' });
        });

        it('answers with a failure on msg.error', async function () {
            await load([
                conn(),
                { id: 'srv', type: 'ros2-service', connection: 'c1', mode: 'server', service: '/nr/fail', rosType: 'std_srvs/srv/Trigger', wires: [['work']] },
                { id: 'work', type: 'helper' }
            ]);
            await mock.waitFor((f) => f.op === 'advertise_service' && f.service === '/nr/fail');
            const srv = helper.getNode('srv');
            helper.getNode('work').on('input', (msg) => {
                msg.error = 'not today';
                srv.receive(msg);
            });
            const res = await mock.callClientService('/nr/fail', {});
            assert.strictEqual(res.result, false);
            assert.deepStrictEqual(res.values, { success: false, message: 'not today' });
        });

        it('rejects input without a request reference in server mode', async function () {
            await load([
                conn(),
                { id: 'srv', type: 'ros2-service', connection: 'c1', mode: 'server', service: '/nr/x', rosType: 'std_srvs/srv/Trigger', wires: [[]] }
            ]);
            const srv = helper.getNode('srv');
            const errors = calls(srv, 'error');
            srv.receive({ payload: {} });
            await until(() => errors.length > 0, 2000, 'error');
            assert.match(String(errors[0]), /wire the end of the flow back into this node and keep msg._ros2/);
        });
    });

    describe('ros2-action', function () {
        function actionFlow(extra = {}) {
            return [
                conn(),
                { id: 'act', type: 'ros2-action', connection: 'c1', action: '/turtle1/rotate_absolute', ...extra, wires: [['fb'], ['res'], ['st']] },
                { id: 'fb', type: 'helper' },
                { id: 'res', type: 'helper' },
                { id: 'st', type: 'helper' }
            ];
        }

        it('delivers feedback, result and status events on three outputs', async function () {
            await load(actionFlow());
            const feedback = collect(helper.getNode('fb'));
            const status = collect(helper.getNode('st'));
            const result = nextInput(helper.getNode('res'));
            helper.getNode('act').receive({ payload: { theta: 1.57 }, correlation: 'abc' });
            const res = await result;
            assert.deepStrictEqual(res.payload, { delta: 1.2 });
            assert.strictEqual(res.ros.status, 'succeeded');
            assert.strictEqual(res.correlation, 'abc');
            assert.deepStrictEqual(feedback.map((m) => m.payload.remaining), [1.0, 0.5, 0.1]);
            await until(() => status.some((m) => m.payload.event === 'succeeded'), 1000, 'succeeded event');
            const events = status.map((m) => m.payload.event);
            assert.deepStrictEqual(events, ['sent', 'executing', 'succeeded']);
            for (const { payload } of status) {
                assert.strictEqual(payload.action, '/turtle1/rotate_absolute');
                assert.strictEqual(payload.goalId, res.ros.goalId);
                assert.ok(!Number.isNaN(Date.parse(payload.at)));
            }
            assert.strictEqual(status[2].payload.status, 'succeeded');
            const frame = await mock.waitFor((f) => f.op === 'send_action_goal');
            assert.strictEqual(frame.action_type, 'turtlesim/action/RotateAbsolute');
        });

        it('emits one feedback status event per feedback when asked to', async function () {
            await load(actionFlow({ feedbackEvents: 'all' }));
            const status = collect(helper.getNode('st'));
            const result = nextInput(helper.getNode('res'));
            helper.getNode('act').receive({ payload: { theta: 1 } });
            await result;
            await until(() => status.some((m) => m.payload.event === 'succeeded'), 1000, 'succeeded event');
            assert.deepStrictEqual(status.map((m) => m.payload.event),
                ['sent', 'executing', 'feedback', 'feedback', 'feedback', 'succeeded']);
        });

        it('cancels on msg.cancel', async function () {
            mock.actions.get('/turtle1/rotate_absolute').hang = true;
            try {
                await load(actionFlow());
                const act = helper.getNode('act');
                const status = collect(helper.getNode('st'));
                const result = nextInput(helper.getNode('res'));
                act.receive({ payload: { theta: 3 } });
                await until(() => status.length > 0, 1000, 'sent');
                act.receive({ cancel: true });
                const res = await result;
                assert.strictEqual(res.ros.status, 'canceled');
                await until(() => status.some((m) => m.payload.event === 'canceled'), 1000, 'canceled event');
                assert.ok(status.some((m) => m.payload.event === 'cancel-requested'));
            } finally {
                mock.actions.get('/turtle1/rotate_absolute').hang = false;
            }
        });

        it('explains a cancel that the server did not honour', async function () {
            const a = mock.actions.get('/turtle1/rotate_absolute');
            a.hang = true;
            a.ignoreCancel = true;
            try {
                await load(actionFlow());
                const act = helper.getNode('act');
                const warns = calls(act, 'warn');
                const status = collect(helper.getNode('st'));
                act.receive({ payload: { theta: 3 } });
                await until(() => status.length > 0, 1000, 'sent');
                act.receive({ cancel: true });
                await until(() => warns.some((w) => /cancel was requested but the goal ended "succeeded"/.test(w)), 2000, 'cancel warning');
                assert.ok(warns.some((w) => /send_action_goals_in_new_thread:=true/.test(w)));
            } finally {
                a.hang = false;
                a.ignoreCancel = false;
            }
        });

        it('refuses a second goal unless concurrent goals are enabled', async function () {
            mock.actions.get('/turtle1/rotate_absolute').hang = true;
            try {
                await load(actionFlow());
                const act = helper.getNode('act');
                const status = collect(helper.getNode('st'));
                const errors = calls(act, 'error');
                act.receive({ payload: { theta: 1 } });
                await until(() => status.length > 0, 1000, 'sent');
                act.receive({ payload: { theta: 2 } });
                await until(() => errors.length > 0, 1000, 'error');
                assert.match(String(errors[0]), /a goal is already running/);
                const failed = status.find((m) => m.payload.event === 'failed');
                assert.strictEqual(failed.payload.goalId, null);
                assert.match(failed.payload.error, /already running/);
            } finally {
                mock.actions.get('/turtle1/rotate_absolute').hang = false;
            }
        });

        it('fails and cancels on timeout', async function () {
            mock.actions.get('/turtle1/rotate_absolute').hang = true;
            try {
                await load(actionFlow({ timeout: 0.1 }));
                const act = helper.getNode('act');
                const status = collect(helper.getNode('st'));
                act.receive({ payload: { theta: 1 } });
                await until(() => status.some((m) => m.payload.event === 'failed'), 2000, 'failed event');
                assert.match(status.find((m) => m.payload.event === 'failed').payload.error, /did not finish within 0.1 s/);
                await mock.waitFor((f) => f.op === 'cancel_action_goal');
            } finally {
                mock.actions.get('/turtle1/rotate_absolute').hang = false;
            }
        });
    });

    describe('ros2-browse', function () {
        it('lists everything', async function () {
            await load([
                conn(),
                { id: 'brw', type: 'ros2-browse', connection: 'c1', what: 'topics', wires: [['out']] },
                { id: 'out', type: 'helper' }
            ]);
            const out = nextInput(helper.getNode('out'));
            helper.getNode('brw').receive({ payload: 'all' });
            const { payload } = await out;
            assert.ok(payload.topics.some((t) => t.name === '/turtle1/pose'));
            assert.ok(payload.services.some((s) => s.name === '/turtle1/teleport_absolute'));
            assert.deepStrictEqual(payload.actions, [{ name: '/turtle1/rotate_absolute', type: 'turtlesim/action/RotateAbsolute', guessed: true }]);
            assert.ok(payload.nodes.includes('/turtlesim'));
        });

        it('builds a template for msg.rosType', async function () {
            await load([
                conn(),
                { id: 'brw', type: 'ros2-browse', connection: 'c1', what: 'template', wires: [['out']] },
                { id: 'out', type: 'helper' }
            ]);
            const out = nextInput(helper.getNode('out'));
            helper.getNode('brw').receive({ payload: 1, rosType: 'turtlesim/srv/TeleportAbsolute', kind: 'request' });
            assert.deepStrictEqual((await out).payload, { x: 0, y: 0, theta: 0 });
        });
    });
});

// sinon ships with the test helper; use its matcher for partial status text checks
function sinonMatch(re) {
    return require('sinon').match(re);
}
