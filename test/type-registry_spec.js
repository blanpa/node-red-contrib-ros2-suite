'use strict';

const assert = require('assert');
const { RosbridgeClient } = require('../lib/rosbridge-client');
const {
    TypeRegistry, normalizeType, fullType, sameType, parseFieldType, similarNames
} = require('../lib/type-registry');
const { MockRosbridge } = require('./mock-rosbridge');

describe('type helpers', function () {
    it('normalizes type names', function () {
        assert.strictEqual(normalizeType('std_msgs/msg/String'), 'std_msgs/String');
        assert.strictEqual(normalizeType('std_msgs/String'), 'std_msgs/String');
        assert.strictEqual(normalizeType('std_msgs::msg::String'), 'std_msgs/String');
        assert.ok(sameType('geometry_msgs/msg/Twist', 'geometry_msgs/Twist'));
        assert.strictEqual(fullType('turtlesim/RotateAbsolute', 'action'), 'turtlesim/action/RotateAbsolute');
        assert.strictEqual(fullType('turtlesim/srv/Spawn', 'srv'), 'turtlesim/srv/Spawn');
    });

    it('parses field types in rosapi and IDL spellings', function () {
        assert.deepStrictEqual(parseFieldType('float64', -1), { base: 'float64', arrayLen: -1, primitive: true });
        assert.deepStrictEqual(parseFieldType('double', 0), { base: 'float64', arrayLen: 0, primitive: true });
        assert.deepStrictEqual(parseFieldType('sequence<geometry_msgs/msg/Point>'), { base: 'geometry_msgs/Point', arrayLen: 0, primitive: false });
        assert.deepStrictEqual(parseFieldType('int32[3]'), { base: 'int32', arrayLen: 3, primitive: true });
        assert.deepStrictEqual(parseFieldType('string<=10'), { base: 'string', arrayLen: -1, primitive: true });
        assert.deepStrictEqual(parseFieldType('octet[]'), { base: 'byte', arrayLen: 0, primitive: true });
    });

    it('suggests similar names', function () {
        const topics = ['/turtle1/pose', '/turtle1/cmd_vel', '/turtle2/pose', '/rosout'];
        assert.deepStrictEqual(similarNames('/turtle1/poses', topics).slice(0, 2), ['/turtle1/pose', '/turtle2/pose']);
        assert.deepStrictEqual(similarNames('/completely/else', topics), []);
    });
});

describe('TypeRegistry', function () {
    let mock;
    let client;
    let registry;

    before(async function () {
        mock = new MockRosbridge();
        await mock.start();
        client = new RosbridgeClient({ url: mock.url });
        registry = new TypeRegistry(client);
        await new Promise((resolve) => {
            client.on('state', (s) => s === 'connected' && resolve());
            client.connect();
        });
    });

    after(async function () {
        await client.close();
        await mock.stop();
    });

    it('lists topics with types and caches them', async function () {
        const topics = await registry.listTopics();
        assert.deepStrictEqual(topics.find((t) => t.name === '/turtle1/pose'), { name: '/turtle1/pose', type: 'turtlesim/msg/Pose' });
        mock.clearReceived();
        await registry.listTopics();
        assert.strictEqual(mock.received.filter((f) => f.service === '/rosapi/topics').length, 0);
    });

    it('resolves topic, service and action types', async function () {
        assert.strictEqual(await registry.topicType('/turtle1/cmd_vel'), 'geometry_msgs/msg/Twist');
        assert.strictEqual(await registry.topicType('/nope'), null);
        assert.strictEqual(await registry.serviceType('/turtle1/teleport_absolute'), 'turtlesim/srv/TeleportAbsolute');
        assert.deepStrictEqual(await registry.actionTypeInfo('/turtle1/rotate_absolute'), { type: 'turtlesim/action/RotateAbsolute', guessed: true });
        assert.strictEqual(await registry.actionType('/robot/navigate_to_pose'), null);
    });

    it('reads the action type from the feedback topic when rosapi lists hidden topics', async function () {
        const reg = new TypeRegistry(client);
        mock.topics.push({ name: '/turtle1/rotate_absolute/_action/feedback', type: 'turtlesim/action/RotateAbsolute_FeedbackMessage' });
        try {
            assert.deepStrictEqual(await reg.actionTypeInfo('/turtle1/rotate_absolute'), { type: 'turtlesim/action/RotateAbsolute', guessed: false });
        } finally {
            mock.topics.pop();
        }
    });

    it('lists services, actions and nodes', async function () {
        const services = await registry.listServices();
        assert.ok(services.find((s) => s.name === '/turtle1/teleport_absolute' && s.type === 'turtlesim/srv/TeleportAbsolute'));
        assert.deepStrictEqual(await registry.listActions(), [{ name: '/turtle1/rotate_absolute', type: 'turtlesim/action/RotateAbsolute', guessed: true }]);
        assert.ok((await registry.listNodes()).includes('/turtlesim'));
    });

    it('builds a template for geometry_msgs/Twist', async function () {
        assert.deepStrictEqual(await registry.template('geometry_msgs/msg/Twist'), {
            linear: { x: 0, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: 0 }
        });
    });

    it('builds templates for arrays, services and actions', async function () {
        assert.deepStrictEqual(await registry.template('test_msgs/Arrays'), {
            header: { stamp: { sec: 0, nanosec: 0 }, frame_id: '' },
            data: [],
            fixed: [0, 0, 0],
            names: [],
            points: [],
            count: 0
        });
        assert.deepStrictEqual(await registry.template('turtlesim/srv/TeleportAbsolute', 'request'), { x: 0, y: 0, theta: 0 });
        assert.deepStrictEqual(await registry.template('turtlesim/action/RotateAbsolute', 'goal'), { theta: 0 });
    });

    it('explains unknown types', async function () {
        await assert.rejects(registry.template('foo_msgs/msg/Nope'), /rosapi knows no type foo_msgs\/msg\/Nope/);
    });

    it('accepts a valid message', async function () {
        const r = await registry.validate('geometry_msgs/Twist', { linear: { x: 1 }, angular: { z: 0.5 } });
        assert.deepStrictEqual(r, { errors: [], warnings: [] });
    });

    it('reports wrong types and unknown fields', async function () {
        const r = await registry.validate('geometry_msgs/msg/Twist', { linear: { x: '1', q: 2 }, angualr: {} });
        assert.strictEqual(r.errors.length, 3);
        assert.ok(r.errors.some((e) => /^angualr: unknown field in geometry_msgs\/Twist — did you mean "angular"\?/.test(e)), r.errors.join('\n'));
        assert.ok(r.errors.includes('linear.x: expected float64 (number), got string "1"'), r.errors.join('\n'));
        assert.ok(r.errors.some((e) => e.startsWith('linear.q: unknown field in geometry_msgs/Vector3')));
    });

    it('checks integer ranges', async function () {
        const r = await registry.validate('test_msgs/Arrays', { count: 200, header: { stamp: { sec: 1.5, nanosec: -1 } } });
        assert.deepStrictEqual(r.errors, [
            'header.stamp.sec: expected integer int32, got 1.5',
            'header.stamp.nanosec: -1 is out of range for uint32 (0..4294967295)',
            'count: 200 is out of range for int8 (-128..127)'
        ]);
    });

    it('checks array lengths and item types', async function () {
        const r = await registry.validate('test_msgs/Arrays', {
            fixed: [1, 2],
            names: ['a', 3],
            points: [{ x: 1 }, 'nope'],
            data: [1, 256]
        });
        assert.deepStrictEqual(r.errors, [
            'data[1]: 256 is out of range for uint8 (0..255)',
            'fixed: expected exactly 3 items (float64[3]), got 2',
            'names[1]: expected string, got number 3',
            'points[1]: expected geometry_msgs/Vector3 (object), got string "nope"'
        ]);
    });

    it('allows uint8[] as base64', async function () {
        assert.deepStrictEqual((await registry.validate('test_msgs/Arrays', { data: 'AAEC' })).errors, []);
        assert.deepStrictEqual((await registry.validate('test_msgs/Arrays', { data: 'not base64!' })).errors,
            ['data: uint8[] given as string must be base64']);
    });

    it('rejects non-object messages', async function () {
        const r = await registry.validate('std_msgs/String', 'hello');
        assert.deepStrictEqual(r.errors, ['message must be an object with the fields of std_msgs/String, got string "hello"']);
    });

    it('validates service requests', async function () {
        const r = await registry.validate('turtlesim/srv/TeleportAbsolute', { x: 1, y: true }, 'request');
        assert.deepStrictEqual(r.errors, ['y: expected float32 (number), got boolean']);
    });

    it('explains a missing rosapi', async function () {
        const reg = new TypeRegistry(client, { timeout: 100 });
        mock.rosapiEnabled = false;
        try {
            await assert.rejects(reg.listNodes(), /is rosapi running next to rosbridge\?/);
        } finally {
            mock.rosapiEnabled = true;
        }
    });
});
