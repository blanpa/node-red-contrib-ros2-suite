'use strict';

const assert = require('assert');
const params = require('../lib/params');
const { RosbridgeClient } = require('../lib/rosbridge-client');
const { MockRosbridge } = require('./mock-rosbridge');

describe('params', function () {
    it('converts JavaScript values to ParameterValue', function () {
        assert.deepStrictEqual(params.toParameterValue(true), { type: 1, bool_value: true });
        assert.deepStrictEqual(params.toParameterValue(3), { type: 2, integer_value: 3 });
        assert.deepStrictEqual(params.toParameterValue(3.5), { type: 3, double_value: 3.5 });
        assert.deepStrictEqual(params.toParameterValue('x'), { type: 4, string_value: 'x' });
        assert.deepStrictEqual(params.toParameterValue(Buffer.from([1, 2])), { type: 5, byte_array_value: [1, 2] });
        assert.deepStrictEqual(params.toParameterValue([true, false]), { type: 6, bool_array_value: [true, false] });
        assert.deepStrictEqual(params.toParameterValue([1, 2]), { type: 7, integer_array_value: [1, 2] });
        assert.deepStrictEqual(params.toParameterValue([1, 2.5]), { type: 8, double_array_value: [1, 2.5] });
        assert.deepStrictEqual(params.toParameterValue(['a']), { type: 9, string_array_value: ['a'] });
    });

    it('lets the type hint decide between integer and double', function () {
        assert.deepStrictEqual(params.toParameterValue(3, 'double'), { type: 3, double_value: 3 });
        assert.deepStrictEqual(params.toParameterValue([1, 2], 8), { type: 8, double_array_value: [1, 2] });
        assert.deepStrictEqual(params.toParameterValue([], 'string_array'), { type: 9, string_array_value: [] });
        assert.deepStrictEqual(params.toParameterValue(3, 'not_set'), { type: 2, integer_value: 3 });
    });

    it('rejects values that do not fit', function () {
        assert.throws(() => params.toParameterValue(3.5, 'integer'), /expected integer, got number 3.5/);
        assert.throws(() => params.toParameterValue('1', 'bool'), /expected bool, got string "1"/);
        assert.throws(() => params.toParameterValue([]), /empty array/);
        assert.throws(() => params.toParameterValue([1, 'a']), /only booleans, only numbers or only strings/);
        assert.throws(() => params.toParameterValue({ a: 1 }), /must be a boolean, number, string/);
        assert.throws(() => params.toParameterValue(1, 'float'), /unknown parameter type "float"/);
    });

    it('reads ParameterValue', function () {
        assert.strictEqual(params.fromParameterValue({ type: 2, integer_value: 7, double_value: 0 }), 7);
        assert.strictEqual(params.fromParameterValue({ type: 0 }), undefined);
        assert.deepStrictEqual(params.fromParameterValue({ type: 9, string_array_value: ['a'] }), ['a']);
        assert.deepStrictEqual(params.fromParameterValue({ type: 5, byte_array_value: 'AQI=' }), Buffer.from([1, 2]));
    });

    it('normalizes node names', function () {
        assert.strictEqual(params.normalizeNode('turtlesim'), '/turtlesim');
        assert.strictEqual(params.normalizeNode(' /ns/node/ '), '/ns/node');
        assert.strictEqual(params.normalizeNode(''), '');
    });

    describe('services', function () {
        let mock;
        let client;

        before(async function () {
            mock = new MockRosbridge();
            await mock.start();
            client = new RosbridgeClient({ url: mock.url, serviceTimeout: 500 });
            const up = new Promise((resolve) => client.on('state', (s) => s === 'connected' && resolve()));
            client.connect();
            await up;
        });

        after(async function () {
            await client.close();
            await mock.stop();
        });

        it('gets, lists, sets and describes parameters', async function () {
            assert.deepStrictEqual(await params.getParameters(client, 'turtlesim', ['background_r', 'nope']), [
                { name: 'background_r', type: 'integer', value: 69 },
                { name: 'nope', type: 'not_set', value: undefined }
            ]);
            const list = await params.listParameters(client, '/turtlesim');
            assert.deepStrictEqual(list.find((p) => p.name === 'gain'), { name: 'gain', type: 'double' });
            assert.deepStrictEqual(list.map((p) => p.name), list.map((p) => p.name).slice().sort());
            const results = await params.setParameters(client, '/turtlesim', [
                { name: 'background_g', value: params.toParameterValue(10) },
                { name: 'gain', value: params.toParameterValue(2) }
            ]);
            assert.deepStrictEqual(results, [
                { name: 'background_g', successful: true, reason: '' },
                { name: 'gain', successful: false, reason: 'wrong type for gain' }
            ]);
            const [d] = await params.describeParameters(client, '/turtlesim', ['gain']);
            assert.strictEqual(d.typeName, 'double');
            // one unknown name must not hide the known ones
            const mixed = await params.describeParameters(client, '/turtlesim', ['nope', 'gain']);
            assert.strictEqual(mixed[0], null);
            assert.strictEqual(mixed[1].name, 'gain');
            assert.deepStrictEqual(await params.getParameterTypes(client, '/turtlesim', ['gain', 'nope']), ['double', 'not_set']);
            assert.deepStrictEqual(await params.describeParameters(client, '/turtlesim', ['nope']), [null]);
            const frame = await mock.waitFor((f) => f.service === '/turtlesim/get_parameters');
            assert.strictEqual(frame.type, 'rcl_interfaces/srv/GetParameters');
        });
    });
});
