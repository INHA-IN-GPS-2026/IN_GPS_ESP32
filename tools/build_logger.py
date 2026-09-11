"""Build only, using the installed IDF toolchain and an isolated configuration."""
import argparse
import os
from pathlib import Path
import subprocess
import sys

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--idf', type=Path, default=Path('C:/Espressif/frameworks/esp-idf-v5.5.2'))
    parser.add_argument('--tools', type=Path, default=Path('C:/Espressif'))
    parser.add_argument('--python', type=Path, default=Path('C:/Espressif/python_env/idf5.5_py3.13_env/Scripts/python.exe'))
    parser.add_argument('--fresh-config', action='store_true', help='Regenerate only build-logger/sdkconfig from defaults')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    build = root / 'build-logger'
    env = os.environ.copy()
    env.update(PYTHONUTF8='1', IDF_PATH=str(args.idf), IDF_TOOLS_PATH=str(args.tools),
               IDF_PYTHON_ENV_PATH=str(args.python.parent.parent))
    exports = subprocess.check_output([str(args.python), str(args.idf/'tools/idf_tools.py'),
                                      'export', '--format', 'key-value'], env=env, text=True)
    for line in exports.splitlines():
        if '=' in line:
            name, value = line.split('=', 1)
            env[name] = value.replace('%PATH%', env.get('PATH', '')).replace('$PATH', env.get('PATH', ''))
    if args.fresh_config:
        (build / 'sdkconfig').unlink(missing_ok=True)
    result = subprocess.run([str(args.python), str(args.idf/'tools/idf.py'), '-B', str(build),
                             '-D', f'SDKCONFIG={build / "sdkconfig"}', 'build'], cwd=root, env=env)
    if result.returncode:
        return result.returncode
    # ELF memory check includes BSS and reserves an explicit 768-byte ULP stack.
    from elftools.elf.elffile import ELFFile
    elf_path = build/'esp-idf/main/ulp_logger/ulp_logger.elf'
    with elf_path.open('rb') as f:
        elf = ELFFile(f)
        end = max(s['sh_addr'] + s['sh_size'] for s in elf.iter_sections()
                  if s['sh_flags'] & 2 and s['sh_size'])
    config = (build/'sdkconfig').read_text(encoding='utf-8')
    reserve = int(next(line.split('=', 1)[1] for line in config.splitlines()
                       if line.startswith('CONFIG_ULP_COPROC_RESERVE_MEM=')))
    print(f'ULP allocated end: {end} bytes; stack/headroom: {reserve-end} bytes')
    if end + 768 > reserve:
        print('ULP memory budget failed (requires >=768-byte stack/headroom).', file=sys.stderr)
        return 1
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
