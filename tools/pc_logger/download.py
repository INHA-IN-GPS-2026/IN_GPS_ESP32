"""Scan and download the IN-GPS direct BLE batch logger (Windows/macOS/Linux)."""
import argparse
import asyncio
import csv
from dataclasses import asdict
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys

from protocol import (Batch, Info, SERVICE_UUID, INFO_UUID, DATA_UUID, CONTROL_UUID,
                      decode_record, select, finish_command)

def atomic_write(path, data):
    temporary = path.with_suffix(path.suffix + '.tmp')
    with temporary.open('wb') as f:
        f.write(data)
        f.flush()
        os.fsync(f.fileno())
    temporary.replace(path)

def checkpoint(path, batch):
    data = {'info': asdict(batch.info),
            'records': [batch.records[i].hex() for i in sorted(batch.records)]}
    atomic_write(path, json.dumps(data, indent=2).encode())

def restore(path, batch):
    if not path.exists():
        return
    saved = json.loads(path.read_text(encoding='utf-8'))
    if saved['info'] != asdict(batch.info):
        raise ValueError('Saved session metadata differs; choose another --out directory')
    for raw in saved['records']:
        batch.ingest(bytes.fromhex(raw))

async def run(args):
    try:
        from bleak import BleakClient, BleakScanner
    except ImportError as e:
        raise RuntimeError('Install dependencies: python -m pip install -r requirements.txt') from e
    print('Scanning; the sensor becomes visible only after the 30-minute collection.')
    found = await BleakScanner.discover(timeout=args.scan_seconds, return_adv=True)
    matches = []
    for dev, adv in found.values():
        if SERVICE_UUID in [u.lower() for u in adv.service_uuids] or (adv.local_name or '').startswith('IN_GPS_LOG_'):
            print(f'{dev.address}  {adv.local_name or dev.name}  RSSI={adv.rssi}')
            matches.append(dev)
    if args.scan:
        return
    if args.address:
        chosen = next((d for d in matches if d.address.casefold() == args.address.casefold()), None)
    else:
        if len(matches) > 1:
            raise ValueError('Multiple loggers found. Select one with --address.')
        chosen = matches[0] if matches else None
    if chosen is None:
        raise ValueError('Logger not found. Wait until collection ends, then scan again.')
    async with BleakClient(chosen, timeout=30) as client:
        info = Info.parse(bytes(await client.read_gatt_char(INFO_UUID)))
        batch = Batch(info)
        args.out.mkdir(parents=True, exist_ok=True)
        stem = args.out / f'ingps_{chosen.address.replace(":", "").replace("-", "")}_{info.session:08x}'
        partial = stem.with_suffix('.partial.json')
        restore(partial, batch)
        print(f'Session {info.session:08x}, already saved {len(batch.records)}/{info.count}')
        try:
            if args.stream and len(batch.records) < info.count:
                changed = asyncio.Event()
                errors = []
                def on_data(_, data):
                    try:
                        batch.ingest(data)
                    except ValueError as e:
                        errors.append(str(e))
                    changed.set()
                await client.start_notify(DATA_UUID, on_data)
                await client.write_gatt_char(CONTROL_UUID, b'\x01', response=True)
                while len(batch.records) < info.count and client.is_connected:
                    changed.clear()
                    try:
                        await asyncio.wait_for(changed.wait(), timeout=10)
                    except asyncio.TimeoutError:
                        break  # repair missing records with indexed reads
                    if len(batch.records) % 30 == 0:
                        checkpoint(partial, batch)
                await client.write_gatt_char(CONTROL_UUID, b'\x05', response=True)
                await asyncio.sleep(0.5)  # allow final indication confirmation
                await client.stop_notify(DATA_UUID)
                if errors:
                    print(f'{len(errors)} malformed indications; repairing by indexed reads')
            for i in range(info.count):
                if i in batch.records:
                    continue
                await client.write_gatt_char(CONTROL_UUID, select(i), response=True)
                raw = bytes(await client.read_gatt_char(DATA_UUID))
                actual = batch.ingest(raw)
                if actual != i:
                    raise ValueError(f'Selected {i}, received {actual}')
                if len(batch.records) % 30 == 0:
                    checkpoint(partial, batch)
                    print(f'\rReceived {len(batch.records)}/{info.count}', end='', flush=True)
            raw = batch.verified_bytes()
        finally:
            checkpoint(partial, batch)  # retain partial on interruption or disconnect
        atomic_write(stem.with_suffix('.bin'), raw)
        csv_path = stem.with_suffix('.csv')
        csv_tmp = csv_path.with_suffix('.csv.tmp')
        rows = [decode_record(batch.records[i]) for i in range(info.count)]
        with csv_tmp.open('w', encoding='utf-8-sig', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
            f.flush(); os.fsync(f.fileno())
        csv_tmp.replace(csv_path)
        metadata = {'device': chosen.address, 'info': asdict(info),
                    'downloaded_utc': datetime.now(timezone.utc).isoformat(),
                    'time_basis': 'seconds since acquisition start, not UTC',
                    'records_with_quality_flags': sum(bool(int(r['flags'], 16) & ~7) for r in rows)}
        atomic_write(stem.with_suffix('.json'), json.dumps(metadata, indent=2).encode())
        print(f'\nCRC verified. Saved {csv_path}')
        if metadata['records_with_quality_flags']:
            print(f"Quality flags in {metadata['records_with_quality_flags']} records; inspect CSV flags.")
        if not args.keep_awake:
            try:
                await client.write_gatt_char(CONTROL_UUID, finish_command(info, args.next_batch), response=True)
                print('Next 30-minute acquisition requested.' if args.next_batch else 'BLE shutdown requested; flash retained.')
            except Exception as e:
                raise RuntimeError(f'Files saved and verified, but shutdown/start acknowledgement failed: {e}') from e

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scan', action='store_true')
    parser.add_argument('--address', help='Bluetooth address printed by --scan')
    parser.add_argument('--scan-seconds', type=float, default=15)
    parser.add_argument('--out', type=Path, default=Path(__file__).resolve().parent / 'downloads')
    parser.add_argument('--stream', action='store_true', help='Use indications, then repair missing indices')
    actions = parser.add_mutually_exclusive_group()
    actions.add_argument('--keep-awake', action='store_true', help='Keep BLE available after verified download')
    actions.add_argument('--next-batch', action='store_true', help='After saving, erase batch and start another 30 minutes')
    args = parser.parse_args()
    if args.scan_seconds <= 0:
        parser.error('--scan-seconds must be positive')
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        print('\nInterrupted. Run the same command to resume.', file=sys.stderr)
        return 130
    except Exception as e:
        print(f'Error: {e}', file=sys.stderr)
        return 1
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
