'use strict';

// Binary data, CBOR, parameters and the clean-up paths of the nodes.

const assert = require('assert');
const helper = require('node-red-node-test-helper');
const { RosbridgeClient } = require('../lib/rosbridge-client');
const { TypeRegistry, encodeBuffers } = require('../lib/type-registry');
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

function calls(node, method) {
    const out = [];
    node.on(`call:${method}`, (call) => out.push(call.args[0]));
    return out;
}

describe('encodeBuffers', function () {
    it('turns Buffers into base64 without touching the input', function () {
        const input = { header: { frame_id: 'x' }, data: Buffer.from([1, 2, 3]), list: [{ data: new Uint8Array([255]) }], n: 1 };
        const out = encodeBuffers(input);
        assert.deepStrictEqual(out, { header: { frame_id: 'x' }, data: 'AQID', list: [{ data: '/w==' }], n: 1 });
        assert.ok(Buffer.isBuffer(input.data));
        assert.strictEqual(out.header, input.header); // unchanged parts are shared
        const plain = { a: [1, 2], b: { c: 'x' } };
        assert.strictEqual(encodeBuffers(plain), plain);
    });
});

describe('RosbridgeClient additions', function () {
    let mock;
    let client;

    beforeEach(async function () {
        mock = new MockRosbridge();
        await mock.start();
        client = new RosbridgeClient({ url: mock.url, reconnectMin: 50, reconnectMax: 200, serviceTimeout: 500 });
        const up = new Promise((resolve) => client.once('state', function on(s) {
            if (s === 'connected') resolve(); else client.once('state', on);
        }));
        client.connect();
        await up;
    });

    afterEach(async function () {
        await client.close();
        await mock.stop();
    });

    it('receives CBOR frames and replays the compression after a reconnect', async function () {
        const got = [];
        client.subscribe('/arrays', (m) => got.push(m), { type: 'test_msgs/msg/Arrays', compression: 'cbor' });
        const frame = await mock.waitFor((f) => f.op === 'subscribe' && f.topic === '/arrays');
        assert.strictEqual(frame.compression, 'cbor');
        mock.publish('/arrays', { data: Buffer.from([1, 2, 255]), fixed: [1.5, 2, 3], count: -3 });
        await until(() => got.length === 1, 1000, 'CBOR message');
        assert.deepStrictEqual(got[0], { data: Buffer.from([1, 2, 255]), fixed: [1.5, 2, 3], count: -3 });

        mock.clearReceived();
        mock.dropClients();
        const again = await mock.waitFor((f) => f.op === 'subscribe' && f.topic === '/arrays', 3000);
        assert.strictEqual(again.compression, 'cbor');
    });

    it('warns about binary frames it cannot decode', async function () {
        const warnings = [];
        client.on('warning', (w) => warnings.push(w));
        for (const ws of mock.sockets) ws.send(Buffer.from([0x9f, 0x01]), { binary: true });
        await until(() => warnings.length === 1, 1000, 'warning');
        assert.match(warnings[0], /ignoring undecodable binary frame/);
    });

    it('warns when a second advertisement asks for a different latch setting', async function () {
        const warnings = [];
        client.on('warning', (w) => warnings.push(w));
        client.advertise('/chatter', 'std_msgs/msg/String', { latch: true });
        client.advertise('/chatter', 'std_msgs/msg/String', { latch: true });
        assert.strictEqual(warnings.length, 0);
        client.advertise('/chatter', 'std_msgs/msg/String');
        assert.match(warnings[0], /\/chatter is already advertised with other latch \/ QoS settings by another node/);
    });

    it('abandons a goal: cancel is sent and no callback fires', async function () {
        mock.actions.get('/turtle1/rotate_absolute').hang = true;
        let fired = 0;
        const handle = client.sendGoal('/turtle1/rotate_absolute', 'turtlesim/action/RotateAbsolute', { theta: 1 }, {
            onResult: () => fired++, onError: () => fired++
        });
        await mock.waitFor((f) => f.op === 'send_action_goal');
        assert.strictEqual(handle.abandon(), true);
        await mock.waitFor((f) => f.op === 'cancel_action_goal' && f.id === handle.goalId);
        await delay(100);
        assert.strictEqual(fired, 0);
        assert.strictEqual(client.activeGoals, 0);
        assert.strictEqual(handle.abandon(), false);
    });

    it('reconnects at once when connect() is called while close() is in flight', async function () {
        const closing = client.close();
        client.connect();
        await closing;
        await until(() => client.connected, 1000, 'reconnect');
        assert.strictEqual(client.lastError, null);
    });

    it('builds a decoder for the binary fields of a type', async function () {
        const registry = new TypeRegistry(client);
        const decode = await registry.binaryDecoder('test_msgs/msg/Arrays');
        const input = { header: { frame_id: 'a' }, data: 'AQID', names: ['AQID'], points: [] };
        const msg = decode(input);
        assert.deepStrictEqual(msg.data, Buffer.from([1, 2, 3]));
        assert.strictEqual(input.data, 'AQID'); // other subscribers share the input
        assert.strictEqual(msg.header, input.header);
        assert.deepStrictEqual(msg.names, ['AQID']); // strings stay strings
        assert.strictEqual(await registry.binaryDecoder('geometry_msgs/msg/Twist'), null);
    });
});

describe('node features', function () {
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

    describe('binary data', function () {
        it('subscribe delivers uint8[] as Buffer when asked to', async function () {
            await load([
                conn(),
                { id: 'sub', type: 'ros2-subscribe', connection: 'c1', topic: '/arrays', buffers: true, wires: [['out']] },
                { id: 'out', type: 'helper' }
            ]);
            const frame = await mock.waitFor((f) => f.op === 'subscribe' && f.topic === '/arrays');
            assert.strictEqual(frame.compression, undefined);
            const got = nextInput(helper.getNode('out'));
            mock.publish('/arrays', { data: 'AQID', count: 1 });
            assert.deepStrictEqual((await got).payload, { data: Buffer.from([1, 2, 3]), count: 1 });
        });

        it('subscribe keeps base64 by default', async function () {
            await load([
                conn(),
                { id: 'sub', type: 'ros2-subscribe', connection: 'c1', topic: '/arrays', wires: [['out']] },
                { id: 'out', type: 'helper' }
            ]);
            await mock.waitFor((f) => f.op === 'subscribe' && f.topic === '/arrays');
            const got = nextInput(helper.getNode('out'));
            mock.publish('/arrays', { data: 'AQID' });
            assert.strictEqual((await got).payload.data, 'AQID');
        });

        it('subscribe asks for CBOR and delivers Buffers', async function () {
            await load([
                conn(),
                { id: 'sub', type: 'ros2-subscribe', connection: 'c1', topic: '/arrays', compression: 'cbor', wires: [['out']] },
                { id: 'out', type: 'helper' }
            ]);
            const frame = await mock.waitFor((f) => f.op === 'subscribe' && f.topic === '/arrays');
            assert.strictEqual(frame.compression, 'cbor');
            const got = nextInput(helper.getNode('out'));
            mock.publish('/arrays', { data: Buffer.from([9, 8]), fixed: [1, 2, 3.5] });
            assert.deepStrictEqual((await got).payload, { data: Buffer.from([9, 8]), fixed: [1, 2, 3.5] });
        });

        it('two subscribe nodes on one topic each get their own representation', async function () {
            await load([
                conn(),
                { id: 'a', type: 'ros2-subscribe', connection: 'c1', topic: '/arrays', buffers: true, wires: [['oa']] },
                { id: 'b', type: 'ros2-subscribe', connection: 'c1', topic: '/arrays', wires: [['ob']] },
                { id: 'oa', type: 'helper' },
                { id: 'ob', type: 'helper' }
            ]);
            await until(() => mock.subscriberCount('/arrays') === 2, 2000, 'both subscriptions');
            const a = nextInput(helper.getNode('oa'));
            const b = nextInput(helper.getNode('ob'));
            mock.publish('/arrays', { data: 'AQID', header: { frame_id: 'x' } });
            assert.deepStrictEqual((await a).payload.data, Buffer.from([1, 2, 3]));
            assert.strictEqual((await b).payload.data, 'AQID');
        });

        it('publish refuses a Buffer as the whole payload', async function () {
            await load([
                conn(),
                { id: 'pub', type: 'ros2-publish', connection: 'c1', topic: '/chatter', wires: [[]] }
            ]);
            const pub = helper.getNode('pub');
            const errors = calls(pub, 'error');
            pub.receive({ payload: Buffer.from('hello') });
            await until(() => errors.length > 0, 2000, 'error');
            assert.match(String(errors[0]), /msg.payload is a Buffer, but std_msgs\/msg\/String needs an object/);
            assert.ok(!mock.received.some((f) => f.op === 'publish'));
        });

        it('publish sends Buffers as base64 and leaves msg.payload alone', async function () {
            await load([
                conn(),
                { id: 'pub', type: 'ros2-publish', connection: 'c1', topic: '/arrays', validation: 'strict', wires: [['out']] },
                { id: 'out', type: 'helper' }
            ]);
            const out = nextInput(helper.getNode('out'));
            helper.getNode('pub').receive({ payload: { data: Buffer.from([1, 2, 3]), count: 2 } });
            const msg = await out;
            assert.ok(Buffer.isBuffer(msg.payload.data));
            const frame = await mock.waitFor((f) => f.op === 'publish' && f.topic === '/arrays');
            assert.deepStrictEqual(frame.msg, { data: 'AQID', count: 2 });
        });
    });

    describe('ros2-param', function () {
        function paramFlow(extra = {}) {
            return [
                conn(),
                { id: 'par', type: 'ros2-param', connection: 'c1', node: '/turtlesim', ...extra, wires: [['out']] },
                { id: 'out', type: 'helper' }
            ];
        }

        it('gets one parameter', async function () {
            await load(paramFlow({ operation: 'get', param: 'background_r' }));
            const out = nextInput(helper.getNode('out'));
            helper.getNode('par').receive({ payload: 'ignored', param: 'gain' });
            const msg = await out;
            assert.strictEqual(msg.payload, 69);
            assert.deepStrictEqual(msg.ros, { node: '/turtlesim', operation: 'get', param: 'background_r', type: 'integer' });
        });

        it('gets several parameters from msg.param', async function () {
            await load(paramFlow({ operation: 'get' }));
            const out = nextInput(helper.getNode('out'));
            helper.getNode('par').receive({ param: ['background_r', 'gain'] });
            const msg = await out;
            assert.deepStrictEqual(msg.payload, { background_r: 69, gain: 0.5 });
            assert.deepStrictEqual(msg.ros.types, { background_r: 'integer', gain: 'double' });
        });

        it('suggests names for an unknown parameter', async function () {
            await load(paramFlow({ operation: 'get', param: 'background_rr' }));
            const par = helper.getNode('par');
            const errors = calls(par, 'error');
            par.receive({});
            await until(() => errors.length > 0, 2000, 'error');
            assert.match(String(errors[0]), /\/turtlesim has no parameter background_rr — did you mean background_r/);
        });

        it('sets a whole number on a double parameter as double', async function () {
            await load(paramFlow({ operation: 'set', param: 'gain' }));
            const out = nextInput(helper.getNode('out'));
            helper.getNode('par').receive({ payload: 2 });
            const msg = await out;
            assert.strictEqual(msg.payload, 2);
            assert.strictEqual(msg.ros.type, 'double');
            const frame = await mock.waitFor((f) => f.service === '/turtlesim/set_parameters');
            assert.deepStrictEqual(frame.args.parameters, [{ name: 'gain', value: { type: 3, double_value: 2 } }]);
            mock.parameters.set('gain', { type: 3, double_value: 0.5 });
        });

        it('sets several parameters from an object and reports rejections', async function () {
            await load(paramFlow({ operation: 'set' }));
            const par = helper.getNode('par');
            const out = nextInput(helper.getNode('out'));
            par.receive({ payload: { background_r: 1, background_g: 2 } });
            await out;
            assert.strictEqual(mock.parameters.get('background_r').integer_value, 1);
            mock.parameters.set('background_r', { type: 2, integer_value: 69 });
            mock.parameters.set('background_g', { type: 2, integer_value: 86 });

            const errors = calls(par, 'error');
            par.receive({ payload: 'red', param: 'background_r', paramType: 'string' });
            await until(() => errors.length > 0, 2000, 'error');
            assert.match(String(errors[0]), /\/turtlesim rejected background_r \(wrong type for background_r\)/);
        });

        it('refuses a value that does not fit the parameter type', async function () {
            await load(paramFlow({ operation: 'set', param: 'background_r' }));
            const par = helper.getNode('par');
            const errors = calls(par, 'error');
            par.receive({ payload: 'red' });
            await until(() => errors.length > 0, 2000, 'error');
            assert.match(String(errors[0]), /background_r: expected integer, got string "red"/);
            assert.ok(!mock.received.some((f) => f.service === '/turtlesim/set_parameters'));
        });

        it('names only the unknown parameters of a mixed request', async function () {
            await load(paramFlow({ operation: 'get' }));
            const par = helper.getNode('par');
            const errors = calls(par, 'error');
            par.receive({ param: ['background_r', 'nope'] });
            await until(() => errors.length > 0, 2000, 'error');
            assert.match(String(errors[0]), /\/turtlesim has no parameter nope$/);
            par.receive({ operation: 'describe', param: 'nope' });
            await until(() => errors.length > 1, 2000, 'second error');
            assert.match(String(errors[1]), /\/turtlesim has no parameter nope$/);
        });

        it('sets a double next to a parameter that is not declared yet', async function () {
            await load(paramFlow({ operation: 'set' }));
            const out = nextInput(helper.getNode('out'));
            helper.getNode('par').receive({ payload: { gain: 3, brand_new: 1 } });
            const msg = await out;
            assert.deepStrictEqual(msg.ros.types, { gain: 'double', brand_new: 'integer' });
            mock.parameters.set('gain', { type: 3, double_value: 0.5 });
            mock.parameters.delete('brand_new');
        });

        it('lists and describes', async function () {
            await load(paramFlow({ operation: 'list' }));
            const par = helper.getNode('par');
            let out = nextInput(helper.getNode('out'));
            par.receive({});
            const list = (await out).payload;
            assert.ok(list.some((p) => p.name === 'use_sim_time' && p.type === 'bool'));
            out = nextInput(helper.getNode('out'));
            par.receive({ operation: 'describe', param: 'gain' });
            const msg = await out;
            assert.strictEqual(msg.payload.typeName, 'double');
            assert.strictEqual(msg.payload.description, 'about gain');
        });

        it('explains a ROS node that is not running', async function () {
            await load(paramFlow({ operation: 'list', node: '/turtlesin' }));
            const par = helper.getNode('par');
            const errors = calls(par, 'error');
            par.receive({});
            await until(() => errors.length > 0, 2000, 'error');
            assert.match(String(errors[0]), /ROS node \/turtlesin is not running — did you mean \/turtlesim/);
        });

        it('serves parameter names to the editor', async function () {
            await load(paramFlow());
            const res = await helper.request().get('/ros2-suite/c1/params?node=turtlesim').expect(200);
            assert.ok(res.body.some((p) => p.name === 'background_r' && p.type === 'integer'));
            await helper.request().get('/ros2-suite/c1/params').expect(503);
        });
    });

    describe('clean-up', function () {
        it('service server drops open requests when the connection is lost', async function () {
            await load([
                conn(),
                { id: 'srv', type: 'ros2-service', connection: 'c1', mode: 'server', service: '/nr/lost', rosType: 'std_srvs/srv/Trigger', timeout: 0, wires: [['work']] },
                { id: 'work', type: 'helper' }
            ]);
            await mock.waitFor((f) => f.op === 'advertise_service' && f.service === '/nr/lost');
            const srv = helper.getNode('srv');
            const warns = calls(srv, 'warn');
            const errors = calls(srv, 'error');
            const request = nextInput(helper.getNode('work'));
            mock.callClientService('/nr/lost', {}).catch(() => {});
            const msg = await request;
            mock.dropClients();
            await until(() => warns.some((w) => /1 open request was lost with the connection/.test(w)), 2000, 'warning');
            const c = helper.getNode('c1');
            await until(() => c.client.connected, 2000, 'reconnect');
            msg.payload = { success: true, message: '' };
            srv.receive(msg);
            await until(() => errors.length > 0, 2000, 'error');
            assert.match(String(errors[0]), /already answered, timed out or lost with the connection/);
        });

        it('action node cancels running goals and completes their messages on close', async function () {
            mock.actions.get('/turtle1/rotate_absolute').hang = true;
            try {
                const c = await load([
                    conn(),
                    { id: 'act', type: 'ros2-action', connection: 'c1', action: '/turtle1/rotate_absolute', wires: [[], [], ['st']] },
                    { id: 'st', type: 'helper' }
                ]);
                const act = helper.getNode('act');
                const sent = nextInput(helper.getNode('st'));
                let completed = 0;
                act.receive({ payload: { theta: 1 } });
                // the test helper has no done-tracking; watch the hook Node-RED calls
                act._complete = () => { completed++; };
                await sent;
                assert.strictEqual(c.client.activeGoals, 1);
                await act.close();
                assert.strictEqual(completed, 1);
                assert.strictEqual(c.client.activeGoals, 0);
                await mock.waitFor((f) => f.op === 'cancel_action_goal');
            } finally {
                mock.actions.get('/turtle1/rotate_absolute').hang = false;
            }
        });

        it('closes the socket of a connection nobody uses any more', async function () {
            this.timeout(9000);
            const c = await load([
                conn(),
                { id: 'brw', type: 'ros2-browse', connection: 'c1', wires: [[]] }
            ]);
            assert.strictEqual(c.users.size, 1);
            await helper.getNode('brw').close();
            assert.strictEqual(c.users.size, 0);
            assert.ok(c.client.connected); // grace period for redeploys
            await until(() => c.client.state === 'disconnected', 7000, 'idle close');
        });
    });

    describe('probe token', function () {
        it('reuses the deployed token only for the deployed URL', async function () {
            const other = new MockRosbridge();
            await other.start();
            try {
                await helper.load(ALL, [conn()], { c1: { token: 's3cret' } });
                await helper.request().post('/ros2-suite/probe/topics')
                    .send({ id: 'c1', host: '127.0.0.1', port: mock.port }).expect(200);
                assert.ok([...mock.sockets].some((ws) => ws.headers.authorization === 'Bearer s3cret'));
                await helper.request().post('/ros2-suite/probe/topics')
                    .send({ id: 'c1', host: '127.0.0.1', port: other.port }).expect(200);
                assert.strictEqual(other.sockets.size, 1);
                assert.strictEqual(other.lastHeaders.authorization, undefined);
            } finally {
                await other.stop();
            }
        });
    });
});
