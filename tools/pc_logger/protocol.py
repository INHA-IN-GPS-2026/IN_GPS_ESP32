"""IN-GPS logger v1 wire codec; no Bluetooth dependency."""
from dataclasses import dataclass
import binascii
import struct
import zlib

SERVICE_UUID = '8c5e0001-7a6b-4d21-9c35-494e47505331'
INFO_UUID = '8c5e0002-7a6b-4d21-9c35-494e47505331'
DATA_UUID = '8c5e0003-7a6b-4d21-9c35-494e47505331'
CONTROL_UUID = '8c5e0004-7a6b-4d21-9c35-494e47505331'
INFO = struct.Struct('<IBBHHHII')
RECORD = struct.Struct('<HHhhHHHHHH')

@dataclass(frozen=True)
class Info:
    magic: int
    version: int
    record_size: int
    count: int
    interval_s: int
    duration_s: int
    session: int
    crc32: int

    @classmethod
    def parse(cls, raw):
        if len(raw) != INFO.size:
            raise ValueError(f'Info length {len(raw)}, expected 20')
        value = cls(*INFO.unpack(raw))
        if (value.magic, value.version, value.record_size, value.count,
                value.interval_s, value.duration_s) != (0x314c4749, 1, 20, 1800, 1, 1800):
            raise ValueError(f'Unsupported logger format: {value}')
        return value

def decode_record(raw, count=1800):
    if len(raw) != RECORD.size:
        raise ValueError(f'Record length {len(raw)}, expected 20')
    index, end, t1, t2, x, y, z, samples, flags, crc = RECORD.unpack(raw)
    if binascii.crc_hqx(raw[:18], 0xffff) != crc:
        raise ValueError('Record CRC16 mismatch')
    if not 0 <= index < count or end != index + 1:
        raise ValueError(f'Invalid record index/time: {index}/{end}')
    if flags & 4 and (not samples or flags & (16 | 32 | 128 | 256)):
        raise ValueError('Invalid RMS validity flags')
    return {
        'index': index, 'end_s': end,
        'th1_C': t1 / 100 if flags & 1 and t1 != -32768 else None,
        'th2_C': t2 / 100 if flags & 2 and t2 != -32768 else None,
        'rms_x_mg': x if flags & 4 else None,
        'rms_y_mg': y if flags & 4 else None,
        'rms_z_mg': z if flags & 4 else None,
        'samples': samples, 'flags': f'0x{flags:04x}',
    }

class Batch:
    def __init__(self, info):
        self.info = info
        self.records = {}

    def ingest(self, data):
        data = bytes(data)
        index = decode_record(data, self.info.count)['index']
        if index in self.records and self.records[index] != data:
            raise ValueError(f'Conflicting duplicate at {index}')
        self.records[index] = data
        return index

    def verified_bytes(self):
        if len(self.records) != self.info.count:
            raise ValueError(f'Incomplete batch: {len(self.records)}/{self.info.count}')
        raw = b''.join(self.records[i] for i in range(self.info.count))
        if zlib.crc32(raw) != self.info.crc32:
            raise ValueError('Whole batch CRC32 mismatch')
        return raw

def select(index):
    if not 0 <= index < 1800:
        raise ValueError('Index outside 0..1799')
    return struct.pack('<BH', 2, index)

def finish_command(info, next_batch=False):
    return struct.pack('<BII', 4 if next_batch else 3, info.session, info.crc32)
