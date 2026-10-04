'use strict';

// End-to-end checks against a real rosbridge with turtlesim (docker/Dockerfile.ros2):
//   docker build -f docker/Dockerfile.ros2 --build-arg ROS_DISTRO=jazzy -t ros2-suite-ros2 docker
//   docker run --rm -d -p 9090:9090 ros2-suite-ros2
//   npm run test:integration            (ROSBRIDGE_URL overrides ws://localhost:9090)
// Nothing here names a distro: types are detected, as a flow would do it.

const assert = require('assert');
const helper = require('node-red-node-test-helper');
const { RosbridgeClient } = require('../../lib/rosbridge-client');
const { TypeRegistry } = require('../../lib/type-registry');

const URL = process.env.ROSBRIDGE_URL || 'ws://localhost:9090';
const STARTUP_TIMEOUT = Number(process.env.ROSBRIDGE_STARTUP_TIMEOUT || 120) * 1000;

const ALL = ['connection', 'subscribe', 'publish', 'service', 'action', 'action-server', 'browse', 'param', 'tf']
    .map((n) => require(`../../nodes/ros2-${n}.js`));

helper.init(require.resolve('node-red'));

const delay = (ms) => new Promise((r) => setTimeout(r, ms));

async function until(fn, timeout = 5000, what = 'condition') {
    const end = Date.now() + timeout;
    while (Date.now() < end) {
        if (await fn()) return;
        await delay(50);
    }
    throw new Error(`timed out waiting for ${what}`);
}

function collect(node) {
    const msgs = [];
    node.on('input', (msg) => msgs.push(msg));
    return msgs;
}

function nextInput(node, timeout = 10000) {
    return new Promise((resolve, reject) => {
        const t = setTimeout(() => reject(new Error(`no message on ${node.id}`)), timeout);
        node.once('input', (msg) => { clearTimeout(t); resolve(msg); });
    });
}

function calls(node, method) {
    const out = [];
    node.on(`call:${method}`, (call) => out.push(call.args[0]));
    return out;
}

const CONN = { id: 'c1', type: 'ros2-connection', url: URL, reconnectMin: 0.2, reconnectMax: 1, serviceTimeout: 10 };

describe(`turtlesim through rosbridge at ${URL}`, function () {
    let probe; // a second rosbridge client, standing in for "another ROS user"

    before(async function () {
        this.timeout(STARTUP_TIMEOUT + 5000);
        probe = new RosbridgeClient({ url: URL, reconnectMin: 500, reconnectMax: 2000 });
        probe.on('warning', () => {});
        probe.connect();
        const registry = new TypeRegistry(probe, { ttl: 0 });
        // rosbridge, rosapi and turtlesim come up one after the other
        await until(async () => {
            if (!probe.connected) return false;
            try {
                const topics = await registry.listTopics();
                const actions = await registry.listActions();
                return topics.some((t) => t.name === '/turtle1/pose') &&
                    actions.some((a) => a.name === '/turtle1/rotate_absolute' && a.type);
            } catch (_) {
                return false;
            }
        }, STARTUP_TIMEOUT, `rosbridge with turtlesim at ${URL}`);
    });

    after(async function () {
        if (probe) await probe.close();
    });

    beforeEach(function (done) {
        helper.startServer(done);
    });

    afterEach(async function () {
        await helper.unload();
        await new Promise((r) => helper.stopServer(r));
    });

    async function load(nodes) {
        await helper.load(ALL, [CONN, ...nodes]);
        const c = helper.getNode('c1');
        await until(() => c.client.connected, 5000, 'connection');
        return c;
    }

    async function teleport(theta = 0) {
        await probe.callService('/turtle1/teleport_absolute', { x: 5.5, y: 5.5, theta });
    }

    it('subscribes with the detected type and shows the rate', async function () {
        await load([
            { id: 'sub', type: 'ros2-subscribe', connection: 'c1', topic: '/turtle1/pose', wires: [['out']] },
            { id: 'out', type: 'helper' }
        ]);
        const msg = await nextInput(helper.getNode('out'));
        assert.strictEqual(typeof msg.payload.x, 'number');
        assert.match(msg.ros.type, /\/msg\/Pose$/);
        const sub = helper.getNode('sub');
        await until(() => sub.status.calledWithMatch({ fill: 'green', text: require('sinon').match(/Hz · \w+\/Pose$/) }), 5000, 'Hz status');
    });

    it('subscribes with CBOR', async function () {
        await load([
            { id: 'sub', type: 'ros2-subscribe', connection: 'c1', topic: '/turtle1/pose', compression: 'cbor', wires: [['out']] },
            { id: 'out', type: 'helper' }
        ]);
        const msg = await nextInput(helper.getNode('out'));
        for (const field of ['x', 'y', 'theta', 'linear_velocity', 'angular_velocity']) {
            assert.strictEqual(typeof msg.payload[field], 'number', field);
        }
        assert.ok(msg.payload.x > 0 && msg.payload.x < 12);
    });

    it('calls a service and publishes: the turtle is teleported, then drives', async function () {
        await load([
            { id: 'sub', type: 'ros2-subscribe', connection: 'c1', topic: '/turtle1/pose', wires: [['pose']] },
            { id: 'pose', type: 'helper' },
            { id: 'svc', type: 'ros2-service', connection: 'c1', service: '/turtle1/teleport_absolute', validation: 'strict', wires: [['res']] },
            { id: 'res', type: 'helper' },
            { id: 'pub', type: 'ros2-publish', connection: 'c1', topic: '/turtle1/cmd_vel', validation: 'strict', wires: [['sent']] },
            { id: 'sent', type: 'helper' }
        ]);
        const poses = collect(helper.getNode('pose'));
        const last = () => poses[poses.length - 1].payload;

        const res = nextInput(helper.getNode('res'));
        helper.getNode('svc').receive({ payload: { x: 3, y: 4, theta: 0 } });
        assert.match((await res).ros.type, /\/srv\/TeleportAbsolute$/);
        await until(() => poses.length && Math.abs(last().x - 3) < 0.01 && Math.abs(last().y - 4) < 0.01, 5000, 'teleported pose');

        const sent = nextInput(helper.getNode('sent'));
        helper.getNode('pub').receive({ payload: { linear: { x: 2 }, angular: { z: 0 } } });
        assert.match((await sent).ros.type, /geometry_msgs\/msg\/Twist$/);
        await until(() => last().x > 3.2, 5000, 'the turtle to drive');
    });

    it('blocks an invalid message in strict mode', async function () {
        await load([
            { id: 'pub', type: 'ros2-publish', connection: 'c1', topic: '/turtle1/cmd_vel', validation: 'strict', wires: [[]] }
        ]);
        const pub = helper.getNode('pub');
        const errors = calls(pub, 'error');
        pub.receive({ payload: { linear: { x: 'fast' } } });
        await until(() => errors.length > 0, 5000, 'error');
        assert.match(String(errors[0]), /linear\.x: expected float64 \(number\), got string "fast"/);
    });

    describe('actions', function () {
        const flow = [
            { id: 'act', type: 'ros2-action', connection: 'c1', action: '/turtle1/rotate_absolute', validation: 'strict', wires: [['fb'], ['res'], ['st']] },
            { id: 'fb', type: 'helper' },
            { id: 'res', type: 'helper' },
            { id: 'st', type: 'helper' }
        ];

        it('runs a goal with feedback, result and status events', async function () {
            await teleport(0);
            await load(flow);
            const feedback = collect(helper.getNode('fb'));
            const status = collect(helper.getNode('st'));
            const result = nextInput(helper.getNode('res'), 20000);
            helper.getNode('act').receive({ payload: { theta: 1.0 }, correlation: 'abc' });
            const res = await result;
            assert.strictEqual(res.ros.status, 'succeeded');
            assert.strictEqual(res.correlation, 'abc');
            assert.strictEqual(typeof res.payload.delta, 'number');
            assert.match(res.ros.type, /\/action\/RotateAbsolute$/);
            assert.ok(feedback.length > 0, 'feedback arrived');
            assert.strictEqual(typeof feedback[0].payload.remaining, 'number');
            await until(() => status.some((m) => m.payload.event === 'succeeded'), 2000, 'succeeded event');
            assert.deepStrictEqual(status.map((m) => m.payload.event), ['sent', 'executing', 'succeeded']);
        });

        it('cancels a running goal', async function () {
            await teleport(0);
            await load(flow);
            const act = helper.getNode('act');
            const status = collect(helper.getNode('st'));
            const result = nextInput(helper.getNode('res'), 20000);
            act.receive({ payload: { theta: 3.0 } });
            await until(() => status.some((m) => m.payload.event === 'executing'), 10000, 'executing event');
            act.receive({ cancel: true });
            assert.strictEqual((await result).ros.status, 'canceled');
            await until(() => status.some((m) => m.payload.event === 'canceled'), 2000, 'canceled event');
        });
    });

    it('provides a service that ROS can call', async function () {
        await load([
            { id: 'srv', type: 'ros2-service', connection: 'c1', mode: 'server', service: '/nodered/it_greet', rosType: 'std_srvs/srv/Trigger', wires: [['work']] },
            { id: 'work', type: 'helper' }
        ]);
        const srv = helper.getNode('srv');
        helper.getNode('work').on('input', (msg) => {
            msg.payload = { success: true, message: 'hello from Node-RED' };
            srv.receive(msg);
        });
        let response = null;
        // the new service needs a moment to be discovered
        await until(async () => {
            try {
                response = await probe.callService('/nodered/it_greet', {}, { type: 'std_srvs/srv/Trigger', timeout: 2000 });
                return true;
            } catch (_) {
                return false;
            }
        }, 15000, 'the service to answer');
        assert.deepStrictEqual(response, { success: true, message: 'hello from Node-RED' });
    });

    describe('parameters', function () {
        const flow = (extra) => [
            { id: 'par', type: 'ros2-param', connection: 'c1', node: '/turtlesim', ...extra, wires: [['out']] },
            { id: 'out', type: 'helper' }
        ];

        it('lists, gets and sets', async function () {
            await load(flow({ operation: 'get', param: 'background_r' }));
            const par = helper.getNode('par');
            const out = helper.getNode('out');

            let got = nextInput(out);
            par.receive({ operation: 'list' });
            const list = (await got).payload;
            for (const name of ['background_r', 'background_g', 'background_b']) {
                assert.deepStrictEqual(list.find((p) => p.name === name), { name, type: 'integer' });
            }

            got = nextInput(out);
            par.receive({});
            const before = (await got).payload;
            assert.ok(Number.isInteger(before));

            const next = before === 200 ? 100 : 200;
            got = nextInput(out);
            par.receive({ operation: 'set', payload: next });
            assert.strictEqual((await got).ros.type, 'integer');

            got = nextInput(out);
            par.receive({});
            assert.strictEqual((await got).payload, next);

            got = nextInput(out);
            par.receive({ operation: 'describe' });
            const descriptor = (await got).payload;
            assert.strictEqual(descriptor.name, 'background_r');
            assert.strictEqual(descriptor.typeName, 'integer');
        });

        it('reports a value the ROS node rejects', async function () {
            await load(flow({ operation: 'set', param: 'background_r', paramType: 'string' }));
            const par = helper.getNode('par');
            const errors = calls(par, 'error');
            par.receive({ payload: 'red' });
            await until(() => errors.length > 0, 10000, 'error');
            assert.match(String(errors[0]), /\/turtlesim rejected background_r/);
        });
    });

    it('browses the graph', async function () {
        await load([
            { id: 'brw', type: 'ros2-browse', connection: 'c1', what: 'all', wires: [['out']] },
            { id: 'out', type: 'helper' }
        ]);
        const out = nextInput(helper.getNode('out'), 20000);
        helper.getNode('brw').receive({});
        const { payload } = await out;
        assert.ok(payload.topics.some((t) => t.name === '/turtle1/pose'));
        assert.ok(payload.services.some((s) => s.name === '/turtle1/teleport_absolute' && /TeleportAbsolute$/.test(s.type)));
        assert.ok(payload.actions.some((a) => a.name === '/turtle1/rotate_absolute' && /\/action\/RotateAbsolute$/.test(a.type)));
        assert.ok(payload.nodes.includes('/turtlesim'));
    });

    describe('binary data', function () {
        const bytes = Buffer.from([0, 1, 2, 250, 255]);

        async function roundTrip(topic, subscribe) {
            await load([
                { id: 'pub', type: 'ros2-publish', connection: 'c1', topic, rosType: 'std_msgs/msg/UInt8MultiArray', validation: 'strict', wires: [[]] },
                { id: 'sub', type: 'ros2-subscribe', connection: 'c1', topic, rosType: 'std_msgs/msg/UInt8MultiArray', ...subscribe, wires: [['out']] },
                { id: 'out', type: 'helper' }
            ]);
            const got = collect(helper.getNode('out'));
            const pub = helper.getNode('pub');
            // publisher and subscriber have to match first; repeat until a message makes it
            await until(() => {
                pub.receive({ payload: { data: bytes } });
                return got.length > 0;
            }, 10000, `a message on ${topic}`);
            return got[0].payload.data;
        }

        it('sends a Buffer and receives base64 with JSON', async function () {
            assert.strictEqual(await roundTrip('/nodered/it_bytes_json', {}), bytes.toString('base64'));
        });

        it('sends a Buffer and receives a Buffer with JSON and the Buffer option', async function () {
            assert.deepStrictEqual(await roundTrip('/nodered/it_bytes_buffer', { buffers: true }), bytes);
        });

        it('sends a Buffer and receives a Buffer with CBOR', async function () {
            assert.deepStrictEqual(await roundTrip('/nodered/it_bytes_cbor', { compression: 'cbor' }), bytes);
        });
    });

    describe('QoS', function () {
        const latchedPublisher = (topic) => ({
            id: 'pub', type: 'ros2-publish', connection: 'c1', topic, rosType: 'std_msgs/msg/String', qos: 'latched', wires: [['sent']]
        });

        it('a latched publisher reaches a subscriber that joins later', async function () {
            const topic = '/nodered/it_latched';
            await load([latchedPublisher(topic), { id: 'sent', type: 'helper' }]);
            const sent = nextInput(helper.getNode('sent'));
            helper.getNode('pub').receive({ payload: 'kept' });
            await sent;
            await delay(1500); // well past the 1 s rosbridge keeps messages of unlatched publishers

            let got = null;
            const late = probe.subscribe(topic, (m) => { got = m; }, {
                type: 'std_msgs/msg/String',
                qos: { history: 'keep_last', depth: 1, reliability: 'reliable', durability: 'transient_local' }
            });
            try {
                await until(() => got !== null, 5000, 'the latched message');
                assert.deepStrictEqual(got, { data: 'kept' });
            } finally {
                late();
            }
        });

        it('a volatile subscription does not get the latched message', async function () {
            // left to itself rosbridge would pick TRANSIENT_LOCAL here, as the only publisher is latched
            const topic = '/nodered/it_latched_volatile';
            await load([latchedPublisher(topic), { id: 'sent', type: 'helper' }]);
            const sent = nextInput(helper.getNode('sent'));
            helper.getNode('pub').receive({ payload: 'old' });
            await sent;
            await delay(500);

            const got = [];
            const late = probe.subscribe(topic, (m) => got.push(m.data), {
                type: 'std_msgs/msg/String',
                qos: { history: 'keep_last', depth: 10, reliability: 'reliable', durability: 'volatile' }
            });
            try {
                await delay(1500);
                assert.deepStrictEqual(got, []);
                helper.getNode('pub').receive({ payload: 'live' });
                await until(() => got.length > 0, 5000, 'the live message');
                assert.deepStrictEqual(got, ['live']);
            } finally {
                late();
            }
        });
    });

    it('fills header stamps and looks up transforms', async function () {
        const tfType = 'tf2_msgs/msg/TFMessage';
        await load([
            { id: 'static', type: 'ros2-publish', connection: 'c1', topic: '/tf_static', rosType: tfType, qos: 'latched', stamp: 'system', validation: 'strict', wires: [[]] },
            { id: 'dynamic', type: 'ros2-publish', connection: 'c1', topic: '/tf', rosType: tfType, stamp: 'system', validation: 'strict', wires: [[]] },
            { id: 'tf', type: 'ros2-tf', connection: 'c1', target: 'it_world', source: 'it_tool', wires: [['out']] },
            { id: 'out', type: 'helper' }
        ]);
        const half = Math.SQRT1_2; // 90° about z
        helper.getNode('static').receive({
            payload: { transforms: [{ header: { frame_id: 'it_base' }, child_frame_id: 'it_tool', transform: { translation: { x: 1, y: 0, z: 0 }, rotation: { x: 0, y: 0, z: 0, w: 1 } } }] }
        });
        const tfNode = helper.getNode('tf');
        const out = collect(helper.getNode('out'));
        tfNode.on('call:error', () => {}); // lookups fail until both transforms have arrived
        await until(() => {
            helper.getNode('dynamic').receive({
                payload: { transforms: [{ header: { frame_id: 'it_world' }, child_frame_id: 'it_base', transform: { translation: { x: 1, y: 2, z: 0 }, rotation: { x: 0, y: 0, z: half, w: half } } }] }
            });
            tfNode.receive({});
            return out.length > 0;
        }, 10000, 'a transform from it_tool to it_world');
        const msg = out[0];
        assert.ok(Math.abs(msg.payload.transform.translation.x - 1) < 1e-6);
        assert.ok(Math.abs(msg.payload.transform.translation.y - 3) < 1e-6);
        assert.ok(Math.abs(msg.ros.rpy.yaw - Math.PI / 2) < 1e-6);
        // the stamp the publish node filled in came back through ROS
        assert.ok(Math.abs(msg.payload.header.stamp.sec - Date.now() / 1000) < 60);
        assert.strictEqual(msg.ros.static, false);
    });

    describe('action server', function () {
        let type;

        before(async function () {
            // the package of turtlesim's action differs between distros
            type = await new TypeRegistry(probe).actionType('/turtle1/rotate_absolute');
        });

        const flow = () => [
            { id: 'srv', type: 'ros2-action-server', connection: 'c1', action: '/nodered/it_rotate', rosType: type, validation: 'strict', wires: [['goal'], ['cancel']] },
            { id: 'goal', type: 'helper' },
            { id: 'cancel', type: 'helper' },
            { id: 'act', type: 'ros2-action', connection: 'c1', action: '/nodered/it_rotate', rosType: type, wires: [['fb'], ['res'], ['st']] },
            { id: 'fb', type: 'helper' },
            { id: 'res', type: 'helper' },
            { id: 'st', type: 'helper' }
        ];

        // the new action server needs a moment to be discovered: retry until a goal arrives
        async function sendUntilAccepted(act, goals, payload) {
            await until(() => {
                if (!goals.length) act.receive({ payload });
                return goals.length > 0;
            }, 20000, 'a goal at the action server');
        }

        it('serves a goal with feedback and a result', async function () {
            this.timeout(40000);
            await load(flow());
            const srv = helper.getNode('srv');
            const act = helper.getNode('act');
            act.on('call:error', () => {});
            const goals = collect(helper.getNode('goal'));
            const feedback = collect(helper.getNode('fb'));
            const results = collect(helper.getNode('res'));
            helper.getNode('goal').on('input', (msg) => {
                srv.receive({ ...msg, feedback: true, payload: { remaining: 0.5 } });
                setTimeout(() => srv.receive({ ...msg, payload: { delta: msg.payload.theta } }), 300);
            });
            await until(async () => {
                if (!goals.length) act.receive({ payload: { theta: 1.25 } });
                await delay(1000);
                return results.length > 0;
            }, 30000, 'a result from the action server');
            assert.strictEqual(results[0].ros.status, 'succeeded');
            assert.strictEqual(results[0].payload.delta, 1.25);
            assert.strictEqual(goals[0].payload.theta, 1.25);
            assert.ok(feedback.some((m) => m.payload.remaining === 0.5), 'feedback arrived');
        });

        it('passes a cancel on and ends the goal as canceled', async function () {
            this.timeout(40000);
            await load(flow());
            const srv = helper.getNode('srv');
            const act = helper.getNode('act');
            act.on('call:error', () => {});
            const goals = collect(helper.getNode('goal'));
            const results = collect(helper.getNode('res'));
            helper.getNode('cancel').on('input', (msg) => {
                srv.receive({ ...msg, status: 'canceled', payload: { delta: 0 } });
            });
            await sendUntilAccepted(act, goals, { theta: 2 });
            await delay(300);
            act.receive({ cancel: true });
            await until(() => results.length > 0, 10000, 'the canceled result');
            assert.strictEqual(results[0].ros.status, 'canceled');
        });
    });

    it('re-subscribes after the connection drops', async function () {
        const c = await load([
            { id: 'sub', type: 'ros2-subscribe', connection: 'c1', topic: '/turtle1/pose', wires: [['out']] },
            { id: 'out', type: 'helper' }
        ]);
        await nextInput(helper.getNode('out'));
        c.client._ws.terminate();
        await until(() => !c.client.connected, 2000, 'disconnect');
        await until(() => c.client.connected, 10000, 'reconnect');
        const msg = await nextInput(helper.getNode('out'));
        assert.strictEqual(typeof msg.payload.x, 'number');
    });
});
