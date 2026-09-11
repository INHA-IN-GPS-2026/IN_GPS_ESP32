"""Copy only required logger settings from sdkconfig.defaults, with a backup.

Run before `idf.py build` when an existing sdkconfig still selects legacy values.
Other settings are preserved; ESP-IDF normalizes derived values on the next build.
"""
from datetime import datetime
from pathlib import Path
import re

REQUIRED = (
    'CONFIG_INGPS_LOGGER_DURATION_S',
    'CONFIG_INGPS_LOGGER_DIAGNOSTICS',
    'CONFIG_INGPS_BATCH_LOGGER', 'CONFIG_ULP_COPROC_ENABLED',
    'CONFIG_ULP_COPROC_TYPE_RISCV', 'CONFIG_ULP_COPROC_RESERVE_MEM',
    'CONFIG_BT_ENABLED', 'CONFIG_BT_NIMBLE_ENABLED',
    'CONFIG_BT_NIMBLE_ROLE_PERIPHERAL', 'CONFIG_BT_NIMBLE_GATT_SERVER',
    'CONFIG_BT_NIMBLE_MAX_CONNECTIONS', 'CONFIG_PARTITION_TABLE_CUSTOM',
    'CONFIG_PARTITION_TABLE_CUSTOM_FILENAME',
)
CONFLICTING = (
    'CONFIG_ULP_COPROC_TYPE_FSM', 'CONFIG_BT_BLUEDROID_ENABLED',
    'CONFIG_PARTITION_TABLE_SINGLE_APP', 'CONFIG_PARTITION_TABLE_SINGLE_APP_LARGE',
    'CONFIG_PARTITION_TABLE_TWO_OTA', 'CONFIG_PARTITION_TABLE_TWO_OTA_LARGE',
)
SETTING = re.compile(r'(?:# )?(CONFIG_[A-Z0-9_]+)(?:=(.*)| is not set)$')

def settings(text):
    result = {}
    for line in text.splitlines():
        match = SETTING.fullmatch(line)
        if match:
            result[match[1]] = match[2] if match[2] is not None else 'n'
    return result

def synchronize(current, defaults):
    original, baseline = settings(current), settings(defaults)
    if original.get('CONFIG_IDF_TARGET') != '"esp32s3"':
        raise ValueError('Expected an ESP32-S3 sdkconfig; no changes made')
    desired = {key: baseline[key] for key in REQUIRED}
    desired.update({key: 'n' for key in CONFLICTING})
    if all(original.get(key, 'n') == value for key, value in desired.items()):
        return current
    newline = '\r\n' if '\r\n' in current else '\n'
    seen = set()
    output = []
    for line in current.splitlines(keepends=True):
        match = SETTING.fullmatch(line.rstrip('\r\n'))
        key = match[1] if match else None
        if key == 'CONFIG_PARTITION_TABLE_FILENAME':
            continue  # derived by Kconfig from the selected custom table
        if key in desired:
            if key in seen:
                continue
            seen.add(key)
            value = desired[key]
            line = (f'# {key} is not set' if value == 'n' else f'{key}={value}') + newline
        output.append(line)
    for key, value in desired.items():
        if key not in seen:
            output.append(newline + (f'# {key} is not set' if value == 'n' else f'{key}={value}') + newline)
    return ''.join(output)

def main():
    root = Path(__file__).resolve().parents[1]
    config = root / 'sdkconfig'
    before = config.read_bytes()
    after = synchronize(before.decode('utf-8'), (root/'sdkconfig.defaults').read_text(encoding='utf-8')).encode('utf-8')
    if before == after:
        print('Required logger settings already match sdkconfig.defaults.')
        return
    backup = root/'build'/('sdkconfig.before_logger_sync_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.bak')
    backup.parent.mkdir(exist_ok=True)
    backup.write_bytes(before)
    config.write_bytes(after)
    print(f'Updated required logger settings. Backup: {backup}')
    print('Next, run idf.py build in the ESP-IDF terminal. No flash was performed.')

if __name__ == '__main__':
    main()
