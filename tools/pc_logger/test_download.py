"""Exercise the real downloader with an in-process GATT peer (no radio)."""
import argparse
import asyncio
import contextlib
import io
import json
from pathlib import Path
import struct
import sys
import tempfile
import types
import unittest
from unittest.mock import patch
import zlib
import download
from protocol import INFO, INFO_UUID, DATA_UUID, SERVICE_UUID
from test_protocol import record

class DownloadTests(unittest.TestCase):
    def make_peer(self, root, fail_after=None, count=1800):
        data = [record(i) for i in range(count)]
        wire_info = INFO.pack(0x314c4749,1,20,count,1,count,42,zlib.crc32(b''.join(data)))
        state = {'reads': 0, 'actions': [], 'fail_after': fail_after}
        dev = types.SimpleNamespace(address='AA:BB:CC:DD:EE:FF', name='IN_GPS_LOG_TEST')
        adv = types.SimpleNamespace(service_uuids=[SERVICE_UUID], local_name=dev.name, rssi=-50)
        class Scanner:
            @staticmethod
            async def discover(**kwargs):
                return {dev.address: (dev, adv)}
        class Client:
            def __init__(self, *args, **kwargs):
                self.selected = 0; self.is_connected = True
            async def __aenter__(self): return self
            async def __aexit__(self, *args): self.is_connected = False
            async def read_gatt_char(self, uuid):
                if uuid == INFO_UUID: return wire_info
                assert uuid == DATA_UUID
                state['reads'] += 1
                if state['fail_after'] and state['reads'] == state['fail_after']:
                    raise ConnectionError('simulated link drop')
                return data[self.selected]
            async def write_gatt_char(self, uuid, command, response):
                assert response is True
                if command[0] == 2:
                    self.selected = struct.unpack('<H', command[1:])[0]
                else:
                    # FINISH/NEXT must happen only after verified durable artifacts.
                    assert len(list(root.glob('*.csv'))) == 1
                    assert len(list(root.glob('*.bin'))) == 1
                    assert next(root.glob('*.bin')).read_bytes() == b''.join(data)
                    state['actions'].append(command)
        module = types.SimpleNamespace(BleakScanner=Scanner, BleakClient=Client)
        args = argparse.Namespace(scan=False, address=None, scan_seconds=1, out=root,
                                  stream=False, keep_awake=False, next_batch=False)
        return module, args, state, data

    def test_disconnect_resume_and_shutdown_after_save(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            module, args, state, _ = self.make_peer(root, fail_after=35)
            with patch.dict(sys.modules, bleak=module), contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaises(ConnectionError): asyncio.run(download.run(args))
                saved = json.loads(next(root.glob('*.partial.json')).read_text())
                self.assertEqual(len(saved['records']), 34)
                self.assertEqual(state['actions'], [])
                state['fail_after'] = None; state['reads'] = 0
                asyncio.run(download.run(args))
            self.assertEqual(state['reads'], 1766)
            self.assertEqual(state['actions'][0][0], 3)
            self.assertEqual(len(next(root.glob('*.csv')).read_text(encoding='utf-8-sig').splitlines()), 1801)

    def test_five_minute_download_and_next_batch(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            module, args, state, _ = self.make_peer(root, count=300)
            args.next_batch = True
            with patch.dict(sys.modules, bleak=module), contextlib.redirect_stdout(io.StringIO()):
                asyncio.run(download.run(args))
            self.assertEqual(state['reads'], 300)
            self.assertEqual(state['actions'][0][0], 4)
            self.assertEqual(len(next(root.glob('*.bin')).read_bytes()), 6000)
            self.assertEqual(len(next(root.glob('*.csv')).read_text(encoding='utf-8-sig').splitlines()), 301)

    def test_corruption_never_acknowledged(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            module, args, state, data = self.make_peer(root)
            bad = bytearray(data[10]); bad[8] ^= 1; data[10] = bytes(bad)
            with patch.dict(sys.modules, bleak=module), contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaisesRegex(ValueError, 'CRC16'): asyncio.run(download.run(args))
            self.assertEqual(state['actions'], [])
            self.assertFalse(list(root.glob('*.csv')))

if __name__ == '__main__': unittest.main()
