'use strict';

// CBOR decoder (RFC 8949) for the frames rosbridge sends with
// `compression: "cbor"`: definite-length items plus the typed-array tags of
// RFC 8746, which rosbridge uses for numeric arrays.

const TYPED_ARRAY_FIRST = 64;
const TYPED_ARRAY_LAST = 87;

function halfToNumber(h) {
    const exp = (h >> 10) & 0x1f;
    const frac = h & 0x3ff;
    const sign = h & 0x8000 ? -1 : 1;
    if (exp === 0) return sign * frac * 2 ** -24;
    if (exp === 31) return frac ? NaN : sign * Infinity;
    return sign * (frac + 1024) * 2 ** (exp - 25);
}

/**
 * RFC 8746 tag layout: 0b010_f_s_e_ll (float, signed, little endian, size).
 * uint8 stays a Buffer, like an untagged byte string; everything else becomes
 * a plain array so the message has the same shape as with JSON.
 */
function typedArray(tag, bytes) {
    const float = (tag & 0x10) !== 0;
    const signed = (tag & 0x08) !== 0;
    const little = (tag & 0x04) !== 0;
    const size = float ? 2 << (tag & 0x03) : 1 << (tag & 0x03);
    if (!float && !signed && size === 1) return bytes; // uint8, uint8 clamped
    if (float && size > 8) throw new Error(`CBOR: unsupported typed array tag ${tag}`);
    if (bytes.length % size) throw new Error(`CBOR: typed array tag ${tag} with ${bytes.length} bytes`);
    const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    const out = new Array(bytes.length / size);
    for (let i = 0, off = 0; i < out.length; i++, off += size) {
        if (float) {
            out[i] = size === 2 ? halfToNumber(view.getUint16(off, little))
                : size === 4 ? view.getFloat32(off, little)
                    : view.getFloat64(off, little);
        } else if (size === 1) {
            out[i] = view.getInt8(off);
        } else if (size === 2) {
            out[i] = signed ? view.getInt16(off, little) : view.getUint16(off, little);
        } else if (size === 4) {
            out[i] = signed ? view.getInt32(off, little) : view.getUint32(off, little);
        } else {
            // like JSON: 64-bit integers become numbers, exact up to 2^53
            out[i] = Number(signed ? view.getBigInt64(off, little) : view.getBigUint64(off, little));
        }
    }
    return out;
}

/** Decode one CBOR item from a Buffer. Byte strings come back as Buffers. */
function decode(buf) {
    const view = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
    let pos = 0;

    function need(n) {
        if (pos + n > buf.length) throw new Error('CBOR: truncated input');
    }

    function argument(info) {
        if (info < 24) return info;
        let v;
        switch (info) {
            case 24: need(1); v = view.getUint8(pos); pos += 1; return v;
            case 25: need(2); v = view.getUint16(pos); pos += 2; return v;
            case 26: need(4); v = view.getUint32(pos); pos += 4; return v;
            case 27: need(8); v = Number(view.getBigUint64(pos)); pos += 8; return v;
            default: throw new Error(`CBOR: unsupported additional information ${info}`);
        }
    }

    function item(depth) {
        if (depth > 128) throw new Error('CBOR: nesting too deep');
        need(1);
        const initial = view.getUint8(pos++);
        const major = initial >> 5;
        const info = initial & 0x1f;
        if (major === 7) {
            switch (info) {
                case 20: return false;
                case 21: return true;
                case 22: return null;
                case 23: return undefined;
                case 25: { need(2); const v = halfToNumber(view.getUint16(pos)); pos += 2; return v; }
                case 26: { need(4); const v = view.getFloat32(pos); pos += 4; return v; }
                case 27: { need(8); const v = view.getFloat64(pos); pos += 8; return v; }
                default: throw new Error(`CBOR: unsupported simple value ${info}`);
            }
        }
        const n = argument(info);
        switch (major) {
            case 0: return n;
            case 1: return -1 - n;
            case 2: { need(n); const v = buf.subarray(pos, pos + n); pos += n; return v; }
            case 3: { need(n); const v = buf.toString('utf8', pos, pos + n); pos += n; return v; }
            case 4: {
                const out = new Array(n);
                for (let i = 0; i < n; i++) out[i] = item(depth + 1);
                return out;
            }
            case 5: {
                const out = {};
                for (let i = 0; i < n; i++) {
                    const key = item(depth + 1);
                    out[String(key)] = item(depth + 1);
                }
                return out;
            }
            default: { // 6: tag
                const value = item(depth + 1);
                if (n >= TYPED_ARRAY_FIRST && n <= TYPED_ARRAY_LAST && Buffer.isBuffer(value)) return typedArray(n, value);
                return value;
            }
        }
    }

    const value = item(0);
    if (pos !== buf.length) throw new Error('CBOR: trailing bytes after the item');
    return value;
}

module.exports = { decode };
