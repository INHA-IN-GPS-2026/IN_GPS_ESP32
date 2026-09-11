import binascii
from dataclasses import replace
import struct
import unittest
import zlib
from protocol import Batch, Info, INFO, RECORD, decode_record, select, finish_command

def record(i=0, temps=(-50, 2500), rms=(0, 32768, 65535), flags=7, samples=100):
    raw = struct.pack('<HHhhHHHHH', i, i+1, *temps, *rms, samples, flags)
    return raw + struct.pack('<H', binascii.crc_hqx(raw, 0xffff))

class ProtocolTests(unittest.TestCase):
    def test_signed_temperature_and_full_uint16_rms(self):
        row = decode_record(record())
        self.assertEqual((row['th1_C'], row['th2_C']), (-0.5, 25.0))
        self.assertEqual((row['rms_x_mg'], row['rms_y_mg'], row['rms_z_mg']), (0, 32768, 65535))

    def test_sensor_error_does_not_become_zero(self):
        row = decode_record(record(temps=(-32768, -32768), flags=0x188, samples=0))
        self.assertIsNone(row['th1_C'])
        self.assertIsNone(row['rms_x_mg'])

    def test_damaged_record(self):
        raw = bytearray(record())
        raw[4] ^= 1
        with self.assertRaisesRegex(ValueError, 'CRC16'):
            decode_record(raw)

    def test_truncation_and_invalid_time(self):
        with self.assertRaises(ValueError):
            decode_record(record()[:-1])
        raw = bytearray(record()); raw[2:4] = b'\x02\x00'
        raw[-2:] = struct.pack('<H', binascii.crc_hqx(raw[:18], 0xffff))
        with self.assertRaisesRegex(ValueError, 'index/time'):
            decode_record(raw)

    def test_bad_validity(self):
        with self.assertRaisesRegex(ValueError, 'validity'):
            decode_record(record(flags=4|16))

    def test_full_batch_reordered_duplicates_and_missing(self):
        records = [record(i) for i in range(1800)]
        raw = b''.join(records)
        info = Info.parse(INFO.pack(0x314c4749, 1, 20, 1800, 1, 1800, 42, zlib.crc32(raw)))
        batch = Batch(info)
        for r in reversed(records[1:]):
            batch.ingest(r)
        with self.assertRaisesRegex(ValueError, 'Incomplete'):
            batch.verified_bytes()
        batch.ingest(records[0]); batch.ingest(records[0])
        self.assertEqual(batch.verified_bytes(), raw)
        with self.assertRaisesRegex(ValueError, 'Conflicting'):
            batch.ingest(record(0, temps=(123,456)))
        batch.info = replace(info, crc32=info.crc32 ^ 1)
        with self.assertRaisesRegex(ValueError, 'CRC32'):
            batch.verified_bytes()

    def test_metadata_and_commands(self):
        info = Info.parse(INFO.pack(0x314c4749, 1, 20, 1800, 1, 1800, 0x12345678, 0x87654321))
        self.assertEqual(select(1799), b'\x02\x07\x07')
        self.assertEqual(finish_command(info), bytes.fromhex('037856341221436587'))
        with self.assertRaises(ValueError):
            select(1800)
        with self.assertRaises(ValueError):
            Info.parse(INFO.pack(0x314c4749, 2, 20, 1800, 1, 1800, 1, 2))

if __name__ == '__main__':
    unittest.main()
