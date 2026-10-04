'use strict';

// QoS, header stamps, shared throttling, TF and the action server.

const assert = require('assert');
const helper = require('node-red-node-test-helper');
const { RosbridgeClient } = require('../lib/rosbridge-client');
const { TypeRegistry } = require('../lib/type-registry');
const { TfBuffer, toEuler } = require('../lib/tf');
const { qosFromConfig } = require('../lib/node-common');
const { MockRosbridge } = require('./mock-rosbridge');

const ALL = ['connection', 'subscribe', 'publish', 'service', 'action', 'action-server', 'browse', 'param', 'tf']
    .map((n) => require(`../nodes/ros2-${n}.js`));

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

const close = (a, b) => Math.abs(a - b) < 1e-9;
const yaw = (angle) => ({ x: 0, y: 0, z: Math.sin(angle / 2), w: Math.cos(angle / 2) });
const edge = (parent, child, translation, rotation = { x: 0, y: 0, z: 0, w: 1 }, sec = 1) => ({
    header: { stamp: { sec, nanosec: 0 }, frame_id: parent },
    child_frame_id: child,
    transform: { translation: { x: 0, y: 0, z: 0, ...translation }, rotation }
});

describe('TfBuffer', function () {
    function tree() {
        const tf = new TfBuffer();
        tf.add([edge('map', 'odom', { x: 10 })], true, 1000);
        tf.add([edge('odom', 'base_link', { x: 1, y: 2 }, yaw(Math.PI / 2), 5)], false, 1000);
        tf.add([edge('base_link', 'tool', { x: 1 })], true, 1000);
        tf.add([edge('odom', 'other', { y: -1 }, undefined, 7)], false, 1500);
        return tf;
    }

    it('chains transforms up the tree', function () {
        const t = tree().lookup('map', 'tool', 2000);
        // tool sits 1 m ahead of a base that is turned 90° left: ahead is +y in the map
        assert.ok(close(t.transform.translation.x, 11) && close(t.transform.translation.y, 3));
        assert.ok(close(toEuler(t.transform.rotation).yaw, Math.PI / 2));
        assert.deepStrictEqual(t.header, { stamp: { sec: 5, nanosec: 0 }, frame_id: 'map' });
        assert.strictEqual(t.child_frame_id, 'tool');
        assert.strictEqual(t.ageMs, 1000);
        assert.strictEqual(t.static, false);
    });

    it('inverts and goes across branches', function () {
        const tf = tree();
        const inv = tf.lookup('tool', 'map', 2000);
        // the map origin seen from the tool: 11 m and 3 m back, expressed in the turned frame
        assert.ok(close(inv.transform.translation.x, -3) && close(inv.transform.translation.y, 11));
        const across = tf.lookup('other', '/base_link', 2000);
        assert.ok(close(across.transform.translation.x, 1) && close(across.transform.translation.y, 3));
        assert.strictEqual(across.ageMs, 1000); // the oldest of the two dynamic edges
        assert.deepStrictEqual(across.header.stamp, { sec: 5, nanosec: 0 });
        const same = tf.lookup('tool', 'tool');
        assert.ok(close(same.transform.rotation.w, 1));
        const fixed = tf.lookup('base_link', 'tool');
        assert.strictEqual(fixed.static, true);
        assert.strictEqual(fixed.ageMs, null);
    });

    it('lists frames and keeps the newest sample', function () {
        const tf = tree();
        assert.deepStrictEqual(tf.frames().map((f) => `${f.parent}>${f.frame}`),
            ['odom>base_link', 'null>map', 'map>odom', 'odom>other', 'base_link>tool']);
        tf.add([edge('odom', 'base_link', { x: 99 }, undefined, 4)]); // older than sec 5
        assert.ok(close(tf.lookup('odom', 'base_link').transform.translation.x, 1));
        tf.add([edge('odom', 'base_link', { x: 5 }, undefined, 6)]);
        assert.ok(close(tf.lookup('odom', 'base_link').transform.translation.x, 5));
    });

    it('explains failed lookups', function () {
        const tf = new TfBuffer();
        assert.throws(() => tf.lookup('map', 'tool'), /no transforms received yet/);
        tf.add([edge('map', 'base_link', {}), edge('world', 'camera', {})]);
        assert.throws(() => tf.lookup('map', 'base_lnk'), /frame "base_lnk" is not in the TF tree — did you mean base_link/);
        assert.throws(() => tf.lookup('map', 'camera'), /"map" and "camera" are not connected: their trees end at "map" and "world"/);
        tf.add([edge('base_link', 'map', {})]);
        assert.throws(() => tf.lookup('map', 'camera'), /loop/);
    });
});

describe('qosFromConfig', function () {
    it('maps presets and custom settings', function () {
        assert.strictEqual(qosFromConfig({}), null);
        assert.strictEqual(qosFromConfig({ qos: 'auto' }), null);
        assert.deepStrictEqual(qosFromConfig({ qos: 'sensor' }), { history: 'keep_last', reliability: 'best_effort', durability: 'volatile', depth: 5 });
        assert.deepStrictEqual(qosFromConfig({ qos: 'latched' }), { history: 'keep_last', reliability: 'reliable', durability: 'transient_local', depth: 1 });
        assert.deepStrictEqual(qosFromConfig({ qos: 'custom', qosReliability: 'best_effort', qosDurability: 'transient_local', qosDepth: '3' }),
            { history: 'keep_last', reliability: 'best_effort', durability: 'transient_local', depth: 3 });
        assert.strictEqual(qosFromConfig({ qos: 'custom' }).depth, 10);
    });
});

describe('client: QoS, throttling, action server', function () {
    let mock;
    let client;

    beforeEach(async function () {
        mock = new MockRosbridge();
        await mock.start();
        client = new RosbridgeClient({ url: mock.url, reconnectMin: 50, reconnectMax: 200, serviceTimeout: 500 });
        client.connect();
        await until(() => client.connected, 2000, 'connection');
    });

    afterEach(async function () {
        await client.close();
        await mock.stop();
    });

    it('sends QoS with subscribe and advertise, also after a reconnect', async function () {
        const qos = { history: 'keep_last', reliability: 'reliable', durability: 'transient_local', depth: 1 };
        client.subscribe('/chatter', () => {}, { qos });
        client.advertise('/chatter', 'std_msgs/msg/String', { qos });
        for (let round = 0; round < 2; round++) {
            const sub = await mock.waitFor((f) => f.op === 'subscribe' && f.topic === '/chatter', 3000);
            const adv = await mock.waitFor((f) => f.op === 'advertise' && f.topic === '/chatter', 3000);
            assert.deepStrictEqual(sub.qos, qos);
            assert.deepStrictEqual(adv.qos, qos);
            assert.strictEqual(adv.latch, undefined);
            if (round === 0) {
                mock.clearReceived();
                mock.dropClients();
            }
        }
    });

    it('throttles the slower of two subscriptions to one topic itself', async function () {
        let fast = 0;
        let slow = 0;
        client.subscribe('/chatter', () => fast++);
        client.subscribe('/chatter', () => slow++, { throttle_rate: 200 });
        await mock.waitFor((f) => f.op === 'subscribe' && f.throttle_rate === 200);
        for (let i = 0; i < 20; i++) {
            mock.publish('/chatter', { data: String(i) });
            await delay(20);
        }
        await delay(50);
        assert.strictEqual(fast, 20);
        assert.ok(slow >= 2 && slow <= 4, `slow subscriber got ${slow}`);
    });

    it('leaves a single throttled subscription to rosbridge', async function () {
        let got = 0;
        client.subscribe('/chatter', () => got++, { throttle_rate: 500 });
        await mock.waitFor((f) => f.op === 'subscribe');
        for (let i = 0; i < 5; i++) mock.publish('/chatter', { data: 'x' });
        await until(() => got === 5, 1000, 'all messages');
    });

    it('serves action goals: feedback, result, cancel, abort', async function () {
        const cancels = [];
        const handlers = {
            onGoal: (args, goal) => {
                if (args.theta < 0) { goal.abort(); return; }
                goal.feedback({ remaining: 1 });
                if (args.theta === 0) return; // waits for a cancel
                assert.strictEqual(goal.succeed({ delta: args.theta }), true);
                assert.strictEqual(goal.succeed({ delta: 0 }), false); // ends once
                assert.strictEqual(goal.feedback({ remaining: 0 }), false);
            },
            onCancel: (id) => cancels.push(id)
        };
        const server = client.advertiseAction('/nr/rotate', 'turtlesim/action/RotateAbsolute', handlers);
        assert.throws(() => client.advertiseAction('/nr/rotate', 'x/action/Y', {}), /already provided/);
        const adv = await mock.waitFor((f) => f.op === 'advertise_action');
        assert.deepStrictEqual(adv, { op: 'advertise_action', action: '/nr/rotate', type: 'turtlesim/action/RotateAbsolute' });

        const ok = mock.sendClientGoal('/nr/rotate', { theta: 2 });
        const done = await ok.result;
        assert.deepStrictEqual([done.status, done.result, done.values], [4, true, { delta: 2 }]);
        assert.deepStrictEqual(ok.feedback, [{ remaining: 1 }]);

        const bad = await mock.sendClientGoal('/nr/rotate', { theta: -1 }).result;
        assert.deepStrictEqual([bad.status, bad.result], [6, false]);

        const waiting = mock.sendClientGoal('/nr/rotate', { theta: 0 });
        await until(() => waiting.feedback.length === 1, 1000, 'feedback');
        mock.cancelClientGoal('/nr/rotate', waiting.id);
        await until(() => cancels.length === 1, 1000, 'cancel');
        assert.strictEqual(cancels[0], waiting.id);

        server.unadvertise();
        await mock.waitFor((f) => f.op === 'unadvertise_action' && f.action === '/nr/rotate');
    });

    it('re-advertises actions and reports lost goals after a reconnect', async function () {
        const lost = [];
        client.advertiseAction('/nr/rotate', 'turtlesim/action/RotateAbsolute', { onGoal: () => {}, onLost: (id) => lost.push(id) });
        await mock.waitFor((f) => f.op === 'advertise_action');
        const goal = mock.sendClientGoal('/nr/rotate', { theta: 1 });
        await delay(50);
        mock.clearReceived();
        mock.dropClients();
        await mock.waitFor((f) => f.op === 'advertise_action', 3000);
        assert.deepStrictEqual(lost, [goal.id]);
    });

    it('fills header stamps, also in nested messages and arrays', async function () {
        const registry = new TypeRegistry(client);
        const stamp = { sec: 5, nanosec: 6 };
        const plain = await registry.stamper('test_msgs/msg/Arrays');
        const input = { header: { frame_id: 'map' }, count: 1 };
        assert.deepStrictEqual(plain(input, stamp), { header: { frame_id: 'map', stamp }, count: 1 });
        assert.deepStrictEqual(input, { header: { frame_id: 'map' }, count: 1 }); // untouched
        assert.deepStrictEqual(plain({ count: 1 }, stamp), { header: { stamp }, count: 1 });
        const own = { header: { stamp: { sec: 1, nanosec: 0 } } };
        assert.strictEqual(plain(own, stamp), own); // a set stamp is kept
        assert.deepStrictEqual(plain({ header: { stamp: { sec: 0, nanosec: 0 } } }, stamp), { header: { stamp } });

        const tf = await registry.stamper('tf2_msgs/msg/TFMessage');
        const out = tf({ transforms: [{ child_frame_id: 'a' }, { header: { stamp: { sec: 9, nanosec: 0 } } }] }, stamp);
        assert.deepStrictEqual(out.transforms[0], { child_frame_id: 'a', header: { stamp } });
        assert.deepStrictEqual(out.transforms[1].header.stamp, { sec: 9, nanosec: 0 });

        const request = await registry.stamper('test_msgs/srv/Plan', 'request');
        assert.deepStrictEqual(request({ start: { value: 1 } }, stamp), { start: { value: 1, header: { stamp } } });
        assert.deepStrictEqual(request({ tolerance: 1 }, stamp), { tolerance: 1 }); // absent sub-messages stay absent
        assert.strictEqual(await registry.stamper('geometry_msgs/msg/Twist'), null);
    });
});

describe('nodes: QoS, stamps, TF, action server', function () {
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

    const conn = () => ({ id: 'c1', type: 'ros2-connection', host: '127.0.0.1', port: mock.port, reconnectMin: 0.05, reconnectMax: 0.2 });

    async function load(flow) {
        await helper.load(ALL, [conn(), ...flow]);
        const c = helper.getNode('c1');
        await until(() => c.client.connected, 2000, 'connection');
        return c;
    }

    it('subscribe and publish pass their QoS to rosbridge', async function () {
        await load([
            { id: 'sub', type: 'ros2-subscribe', connection: 'c1', topic: '/chatter', qos: 'sensor', wires: [[]] },
            { id: 'pub', type: 'ros2-publish', connection: 'c1', topic: '/chatter', rosType: 'std_msgs/msg/String', qos: 'custom', qosReliability: 'reliable', qosDurability: 'transient_local', qosDepth: 7, wires: [[]] },
            { id: 'plain', type: 'ros2-subscribe', connection: 'c1', topic: '/turtle1/pose', wires: [[]] }
        ]);
        const sub = await mock.waitFor((f) => f.op === 'subscribe' && f.topic === '/chatter');
        assert.deepStrictEqual(sub.qos, { history: 'keep_last', reliability: 'best_effort', durability: 'volatile', depth: 5 });
        const adv = await mock.waitFor((f) => f.op === 'advertise' && f.topic === '/chatter');
        assert.deepStrictEqual(adv.qos, { history: 'keep_last', reliability: 'reliable', durability: 'transient_local', depth: 7 });
        const plain = await mock.waitFor((f) => f.op === 'subscribe' && f.topic === '/turtle1/pose');
        assert.strictEqual(plain.qos, undefined);
    });

    it('publish fills empty header stamps with the system time', async function () {
        await load([
            { id: 'pub', type: 'ros2-publish', connection: 'c1', topic: '/arrays', stamp: 'system', validation: 'strict', wires: [['out']] },
            { id: 'out', type: 'helper' }
        ]);
        const before = Date.now() / 1000;
        const out = nextInput(helper.getNode('out'));
        helper.getNode('pub').receive({ payload: { header: { frame_id: 'map' }, count: 1 } });
        const msg = await out;
        assert.deepStrictEqual(msg.payload, { header: { frame_id: 'map' }, count: 1 }); // msg.payload stays as sent
        const frame = await mock.waitFor((f) => f.op === 'publish' && f.topic === '/arrays');
        const { sec, nanosec } = frame.msg.header.stamp;
        assert.ok(Number.isInteger(sec) && Number.isInteger(nanosec) && nanosec >= 0 && nanosec < 1e9);
        assert.ok(Math.abs(sec + nanosec / 1e9 - before) < 5);
        assert.strictEqual(frame.msg.header.frame_id, 'map');
    });

    it('publish stamps with ROS time from /clock and falls back with a warning', async function () {
        await load([
            { id: 'pub', type: 'ros2-publish', connection: 'c1', topic: '/arrays', stamp: 'clock', wires: [['out']] },
            { id: 'out', type: 'helper' }
        ]);
        const pub = helper.getNode('pub');
        const warns = calls(pub, 'warn');
        await mock.waitFor((f) => f.op === 'subscribe' && f.topic === '/clock');
        let out = nextInput(helper.getNode('out'));
        pub.receive({ payload: { count: 1 } });
        await out;
        assert.match(warns[0], /no message on \/clock yet — stamping with the system time/);

        mock.publish('/clock', { clock: { sec: 42, nanosec: 7 } });
        await delay(50);
        mock.clearReceived();
        out = nextInput(helper.getNode('out'));
        pub.receive({ payload: { count: 2 } });
        await out;
        const frame = await mock.waitFor((f) => f.op === 'publish' && f.topic === '/arrays');
        assert.deepStrictEqual(frame.msg.header.stamp, { sec: 42, nanosec: 7 });
        assert.strictEqual(warns.length, 1);
    });

    it('service and action stamp requests and goals', async function () {
        await load([
            { id: 'svc', type: 'ros2-service', connection: 'c1', service: '/plan', stamp: 'system', wires: [['out']] },
            { id: 'out', type: 'helper' }
        ]);
        const out = nextInput(helper.getNode('out'));
        helper.getNode('svc').receive({ payload: { start: { value: 1 } } });
        await out;
        const frame = await mock.waitFor((f) => f.op === 'call_service' && f.service === '/plan');
        assert.ok(frame.args.start.header.stamp.sec > 0);
    });

    it('tf looks up transforms and lists frames', async function () {
        await load([
            { id: 'tf', type: 'ros2-tf', connection: 'c1', target: 'map', source: 'tool', wires: [['out']] },
            { id: 'out', type: 'helper' }
        ]);
        const tfNode = helper.getNode('tf');
        const statics = await mock.waitFor((f) => f.op === 'subscribe' && f.topic === '/tf_static');
        assert.strictEqual(statics.qos.durability, 'transient_local');
        await mock.waitFor((f) => f.op === 'subscribe' && f.topic === '/tf');

        const errors = calls(tfNode, 'error');
        tfNode.receive({});
        await until(() => errors.length > 0, 2000, 'error');
        assert.match(String(errors[0]), /no transforms received yet/);

        mock.publish('/tf_static', { transforms: [edge('base_link', 'tool', { x: 1 })] });
        mock.publish('/tf', { transforms: [edge('map', 'base_link', { x: 1, y: 2 }, yaw(Math.PI / 2))] });
        await delay(100);
        let out = nextInput(helper.getNode('out'));
        tfNode.receive({ target: 'ignored' });
        let msg = await out;
        assert.ok(close(msg.payload.transform.translation.x, 1) && close(msg.payload.transform.translation.y, 3));
        assert.strictEqual(msg.payload.header.frame_id, 'map');
        assert.strictEqual(msg.payload.child_frame_id, 'tool');
        assert.strictEqual(msg.payload.ageMs, undefined);
        assert.ok(close(msg.ros.rpy.yaw, Math.PI / 2));
        assert.strictEqual(msg.ros.static, false);
        assert.ok(msg.ros.ageMs >= 0);

        out = nextInput(helper.getNode('out'));
        tfNode.receive({ operation: 'frames' });
        msg = await out;
        assert.deepStrictEqual(msg.payload.map((f) => f.frame), ['base_link', 'map', 'tool']);

        await tfNode.close();
        await mock.waitFor((f) => f.op === 'unsubscribe' && f.topic === '/tf');
    });

    it('tf refuses transforms older than the limit', async function () {
        await load([
            { id: 'tf', type: 'ros2-tf', connection: 'c1', target: 'map', source: 'base_link', maxAge: 0.1, wires: [[]] }
        ]);
        const tfNode = helper.getNode('tf');
        await mock.waitFor((f) => f.op === 'subscribe' && f.topic === '/tf');
        mock.publish('/tf', { transforms: [edge('map', 'base_link', { x: 1 })] });
        await delay(300);
        const errors = calls(tfNode, 'error');
        tfNode.receive({});
        await until(() => errors.length > 0, 2000, 'error');
        assert.match(String(errors[0]), /the transform from base_link to map is 0\.\d s old \(limit 0\.1 s\)/);
    });

    describe('ros2-action-server', function () {
        const flow = (extra = {}) => [
            { id: 'srv', type: 'ros2-action-server', connection: 'c1', action: '/nr/rotate', rosType: 'turtlesim/action/RotateAbsolute', validation: 'strict', ...extra, wires: [['goal'], ['cancel']] },
            { id: 'goal', type: 'helper' },
            { id: 'cancel', type: 'helper' }
        ];

        it('emits goals and returns feedback and the result', async function () {
            await load(flow());
            await mock.waitFor((f) => f.op === 'advertise_action' && f.action === '/nr/rotate');
            const srv = helper.getNode('srv');
            const request = nextInput(helper.getNode('goal'));
            const goal = mock.sendClientGoal('/nr/rotate', { theta: 1.5 });
            const msg = await request;
            assert.deepStrictEqual(msg.payload, { theta: 1.5 });
            assert.deepStrictEqual(msg.ros, { action: '/nr/rotate', type: 'turtlesim/action/RotateAbsolute', goalId: goal.id });

            srv.receive({ ...msg, feedback: true, payload: { remaining: 0.5 } });
            await until(() => goal.feedback.length === 1, 1000, 'feedback');
            srv.receive({ ...msg, payload: { delta: 1.5 } });
            const result = await goal.result;
            assert.deepStrictEqual([result.status, result.result, result.values], [4, true, { delta: 1.5 }]);
            assert.deepStrictEqual(goal.feedback, [{ remaining: 0.5 }]);

            const errors = calls(srv, 'error');
            srv.receive({ ...msg, payload: { delta: 0 } });
            await until(() => errors.length > 0, 1000, 'error');
            assert.match(String(errors[0]), /has already ended or was lost/);
        });

        it('passes cancel requests on and ends the goal as canceled', async function () {
            await load(flow());
            await mock.waitFor((f) => f.op === 'advertise_action');
            const srv = helper.getNode('srv');
            const request = nextInput(helper.getNode('goal'));
            const goal = mock.sendClientGoal('/nr/rotate', { theta: 3 });
            await request;
            const cancel = nextInput(helper.getNode('cancel'));
            mock.cancelClientGoal('/nr/rotate', goal.id);
            const msg = await cancel;
            assert.deepStrictEqual(msg.payload, { goalId: goal.id });
            srv.receive({ ...msg, status: 'canceled', payload: { delta: 0.2 } });
            const result = await goal.result;
            assert.deepStrictEqual([result.status, result.result], [5, true]);
        });

        it('aborts on msg.error and on an invalid result', async function () {
            await load(flow());
            await mock.waitFor((f) => f.op === 'advertise_action');
            const srv = helper.getNode('srv');
            let request = nextInput(helper.getNode('goal'));
            let goal = mock.sendClientGoal('/nr/rotate', { theta: 1 });
            srv.receive({ ...(await request), error: 'motor fault' });
            assert.deepStrictEqual([(await goal.result).status, (await goal.result).result], [6, false]);

            request = nextInput(helper.getNode('goal'));
            goal = mock.sendClientGoal('/nr/rotate', { theta: 1 });
            const errors = calls(srv, 'error');
            srv.receive({ ...(await request), payload: { delta: 'far' } });
            assert.strictEqual((await goal.result).result, false);
            await until(() => errors.length > 0, 1000, 'error');
            assert.match(String(errors[0]), /invalid result for turtlesim\/RotateAbsolute: delta: expected float32/);
        });

        it('aborts running goals when the node is closed', async function () {
            await load(flow());
            await mock.waitFor((f) => f.op === 'advertise_action');
            const request = nextInput(helper.getNode('goal'));
            const goal = mock.sendClientGoal('/nr/rotate', { theta: 1 });
            await request;
            await helper.getNode('srv').close();
            assert.strictEqual((await goal.result).result, false);
            await mock.waitFor((f) => f.op === 'unadvertise_action');
        });

        it('needs a name and a type', async function () {
            // without a type the node never uses the connection, so nothing connects
            await helper.load(ALL, [conn(), { id: 'srv', type: 'ros2-action-server', connection: 'c1', action: '/nr/x', wires: [[], []] }]);
            assert.ok(helper.getNode('srv').status.calledWithMatch({ fill: 'red', text: 'the action type is required' }));
        });
    });

    it('throttles a slow subscribe node next to a fast one', async function () {
        await load([
            { id: 'fast', type: 'ros2-subscribe', connection: 'c1', topic: '/chatter', wires: [['f']] },
            { id: 'slow', type: 'ros2-subscribe', connection: 'c1', topic: '/chatter', throttle: 300, wires: [['s']] },
            { id: 'f', type: 'helper' },
            { id: 's', type: 'helper' }
        ]);
        await until(() => mock.subscriberCount('/chatter') === 2, 2000, 'both subscriptions');
        const fast = collect(helper.getNode('f'));
        const slow = collect(helper.getNode('s'));
        for (let i = 0; i < 10; i++) {
            mock.publish('/chatter', { data: String(i) });
            await delay(20);
        }
        await delay(50);
        assert.strictEqual(fast.length, 10);
        assert.strictEqual(slow.length, 1);
    });
});
