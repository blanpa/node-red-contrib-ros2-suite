'use strict';

const assert = require('assert');
const cbor = require('../lib/cbor');
const { MockRosbridge } = require('./mock-rosbridge');

const hex = (s) => Buffer.from(s.replace(/\s+/g, ''), 'hex');

describe('cbor', function () {
    it('decodes the RFC 8949 examples', function () {
        assert.strictEqual(cbor.decode(hex('00')), 0);
        assert.strictEqual(cbor.decode(hex('1903e8')), 1000);
        assert.strictEqual(cbor.decode(hex('1a000f4240')), 1000000);
        assert.strictEqual(cbor.decode(hex('1b000000e8d4a51000')), 1000000000000);
        assert.strictEqual(cbor.decode(hex('3863')), -100);
        assert.strictEqual(cbor.decode(hex('f93c00')), 1.0);
        assert.strictEqual(cbor.decode(hex('f9c400')), -4.0);
        assert.strictEqual(cbor.decode(hex('fa47c35000')), 100000.0);
        assert.strictEqual(cbor.decode(hex('fb3ff199999999999a')), 1.1);
        assert.strictEqual(cbor.decode(hex('f97c00')), Infinity);
        assert.ok(Number.isNaN(cbor.decode(hex('f97e00'))));
        assert.strictEqual(cbor.decode(hex('f4')), false);
        assert.strictEqual(cbor.decode(hex('f6')), null);
        assert.strictEqual(cbor.decode(hex('6449455446')), 'IETF');
        assert.strictEqual(cbor.decode(hex('62c3bc')), 'ü');
        assert.deepStrictEqual(cbor.decode(hex('4401020304')), Buffer.from([1, 2, 3, 4]));
        assert.deepStrictEqual(cbor.decode(hex('8301820203820405')), [1, [2, 3], [4, 5]]);
        assert.deepStrictEqual(cbor.decode(hex('a26161016162820203')), { a: 1, b: [2, 3] });
    });

    it('decodes typed arrays (RFC 8746) into plain arrays', function () {
        // tag 69: uint16 little endian
        assert.deepStrictEqual(cbor.decode(hex('d845 44 0100 0002')), [1, 512]);
        // tag 78: int32 little endian
        assert.deepStrictEqual(cbor.decode(hex('d84e 48 ffffffff 02000000')), [-1, 2]);
        // tag 85: float32 little endian
        assert.deepStrictEqual(cbor.decode(hex('d855 48 0000803f 000000c0')), [1, -2]);
        // tag 86: float64 little endian
        assert.deepStrictEqual(cbor.decode(hex('d856 48 000000000000f83f')), [1.5]);
        // tag 79: int64 little endian
        assert.deepStrictEqual(cbor.decode(hex('d84f 48 feffffffffffffff')), [-2]);
        // tag 72: int8
        assert.deepStrictEqual(cbor.decode(hex('d848 42 ff01')), [-1, 1]);
        // tag 64: uint8 stays binary
        assert.deepStrictEqual(cbor.decode(hex('d840 42 ff01')), Buffer.from([255, 1]));
    });

    it('round-trips a publish frame from the mock encoder', function () {
        const frame = { op: 'publish', topic: '/scan', msg: { ranges: new Float32Array([0.5, 1.5]), data: Buffer.from('hi'), n: -70000, ok: true, x: 2.25 } };
        assert.deepStrictEqual(cbor.decode(MockRosbridge.cborEncode(frame)), {
            op: 'publish', topic: '/scan', msg: { ranges: [0.5, 1.5], data: Buffer.from('hi'), n: -70000, ok: true, x: 2.25 }
        });
    });

    it('rejects truncated and trailing input', function () {
        assert.throws(() => cbor.decode(hex('1903')), /truncated/);
        assert.throws(() => cbor.decode(hex('6449')), /truncated/);
        assert.throws(() => cbor.decode(hex('0000')), /trailing/);
        assert.throws(() => cbor.decode(hex('9f01ff')), /unsupported additional information/);
    });
});
