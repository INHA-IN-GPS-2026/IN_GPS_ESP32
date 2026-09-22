#!/usr/bin/env python3
"""
battflash.py — 배터리 전원 그대로 ESP32-S3 플래싱 (어떤 펌웨어든 공용)

원리
  - 플래싱은 ROM 부트로더 + esptool stub이 하므로, 올리는 펌웨어 내용과 무관하다.
  - 64KB씩 쓰고, 사이사이 칩을 리셋(EN=LOW) 상태로 붙잡아 C6(0.47F)를 재충전한다.
  - 시리얼 포트를 처음부터 끝까지 한 번만 열어두므로, 조각 사이에 앱이 부팅하지 않는다.

사용 (ESP-IDF 프로젝트 루트에서, idf.py build 후)
    python battflash.py -p COM5              # build/flash_args 전체(부트로더+파티션+앱)
    python battflash.py -p COM5 --app-only   # 앱만 (빠름)
    python battflash.py -p COM5 0x10000 some.bin   # 임의 bin 직접 지정

필요: pip install "esptool>=5" pyserial
"""
import argparse
import os
import sys
import time

import serial
from esptool.cmds import attach_flash, detect_chip, run_stub, write_flash
from esptool.loader import ESPLoader

# 포트를 열어둔 채 넘기면 esptool이 VID/PID 조회에 실패해 매번 경고를 찍는다.
# 결과는 어차피 '표준 리셋'(Connector PCB DTR/RTS 회로에 맞는 방식)이므로 조회를 생략.
ESPLoader.get_usb_vid_pid = lambda self: (None, None)

SECTOR = 4096


def set_lines(port: serial.Serial, dtr: bool, rts: bool) -> None:
    """pyserial True = 라인 active = 핀 LOW (esptool과 같은 규약)."""
    port.dtr = dtr
    port.rts = rts
    port.dtr = dtr  # Windows 드라이버에서 RTS 변경을 확실히 반영시키는 esptool과 같은 우회


def hold_reset(port: serial.Serial) -> None:
    """esptool ClassicReset의 리셋 구간과 동일: IO0=HIGH, EN=LOW → 칩 정지, 소모전류 최소.

    esptool은 접속 실패 시 포트를 닫아버린다. 닫힌 상태에서는 DTR/RTS가 풀려
    칩이 부팅(→ 미완성 펌웨어면 부팅 반복)하면서 C6를 소모하므로, 즉시 다시 연다.
    """
    if not port.is_open:
        port.dtr = False
        port.rts = True       # 열리는 순간부터 리셋 유지
        port.baudrate = 115200
        port.open()
    set_lines(port, dtr=False, rts=True)


def load_flash_args(build: str, app_only: bool):
    fname = "flash_app_args" if app_only else "flash_args"
    path = os.path.join(build, fname)
    if not os.path.exists(path):
        sys.exit(f"{path} 없음 — 프로젝트 루트에서 idf.py build 후 실행하거나 --build 로 경로 지정")
    pairs = []
    with open(path) as f:
        for line in f:
            tok = line.split()
            if not tok or tok[0].startswith("--"):
                continue  # --flash_mode 등: 이미지 헤더에 이미 반영돼 있어 keep 사용
            pairs.append((int(tok[0], 0), os.path.join(build, tok[1])))
    return pairs


def build_jobs(pairs, chunk):
    jobs = []
    for addr, path in sorted(pairs):
        if addr % SECTOR:
            sys.exit(f"{hex(addr)} 가 4KB 정렬이 아님")
        with open(path, "rb") as f:
            data = f.read()
        for off in range(0, len(data), chunk):
            jobs.append((addr + off, data[off:off + chunk], os.path.basename(path)))
    return jobs


def main() -> None:
    ap = argparse.ArgumentParser(description="배터리 전원용 조각 플래싱 (펌웨어 무관)")
    ap.add_argument("-p", "--port", required=True)
    ap.add_argument("-b", "--baud", type=int, default=921600)
    ap.add_argument("--build", default="build", help="ESP-IDF build 폴더 (기본: ./build)")
    ap.add_argument("--app-only", action="store_true", help="앱 파티션만 쓰기")
    ap.add_argument("--chunk-kb", type=int, default=64, help="조각 크기 KB (4의 배수)")
    ap.add_argument("--rest", type=float, default=5.0, help="조각 사이 리셋 유지(초)")
    ap.add_argument("--retries", type=int, default=4, help="조각당 재시도 횟수")
    ap.add_argument("--recover", type=float, default=15.0,
                    help="실패 후 재시도 전 리셋 유지(초) — C6 회복용")
    ap.add_argument("--precharge", type=float, default=15.0, help="시작 전 리셋 유지(초)")
    ap.add_argument("pairs", nargs="*", help="직접 지정 시: addr file [addr file ...]")
    a = ap.parse_args()

    chunk = a.chunk_kb * 1024
    if chunk <= 0 or chunk % SECTOR:
        ap.error("--chunk-kb는 4의 배수여야 함")

    if a.pairs:
        if len(a.pairs) % 2:
            ap.error("addr/file 쌍으로 입력해야 함")
        pairs = [(int(a.pairs[i], 0), a.pairs[i + 1]) for i in range(0, len(a.pairs), 2)]
    else:
        pairs = load_flash_args(a.build, a.app_only)

    jobs = build_jobs(pairs, chunk)
    total = sum(len(j[1]) for j in jobs)
    print(f"{len(jobs)}개 조각, {total/1024:.0f} KB — 시작 전 {a.precharge}s 리셋 유지(C6 충전)")

    port = serial.Serial()
    port.port = a.port
    port.baudrate = 115200
    port.dtr = False
    port.rts = True          # 열리는 순간부터 리셋 유지
    port.open()
    try:
        hold_reset(port)
        time.sleep(a.precharge)

        for n, (addr, blob, name) in enumerate(jobs, 1):
            for attempt in range(1, a.retries + 2):
                tag = "" if attempt == 1 else f" (재시도 {attempt - 1}/{a.retries})"
                print(f"\n[{n}/{len(jobs)}] {name} @ {hex(addr)} ({len(blob)} B){tag}")
                try:
                    hold_reset(port)                 # 포트가 닫혀 있으면 다시 열기
                    port.baudrate = 115200
                    esp = detect_chip(port, baud=115200, connect_mode="default-reset")
                    esp = run_stub(esp)
                    if a.baud != 115200:
                        esp.change_baud(a.baud)
                    attach_flash(esp)
                    write_flash(esp, [(addr, blob)],
                                flash_mode="keep", flash_freq="keep", flash_size="keep")
                    break
                except Exception as e:
                    hold_reset(port)
                    if attempt > a.retries:
                        raise
                    print(f"  ! 조각 실패({e.__class__.__name__}) — {a.recover:.0f}s 리셋 유지 후 재시도")
                    time.sleep(a.recover)
            hold_reset(port)                     # 다음 조각 전까지 칩 정지 + C6 재충전
            if n < len(jobs):
                time.sleep(a.rest)

        time.sleep(0.1)
        set_lines(port, dtr=False, rts=False)    # 리셋 해제 → 새 펌웨어로 정상 부팅
        print("\n완료 — 정상 부팅")
    except Exception as e:
        try:
            hold_reset(port)
        except Exception:
            pass
        print(f"\n실패: {e}\n→ 다시 실행: python tools\\battflash.py -p {a.port} --rest 10 --chunk-kb 32")
        sys.exit(1)
    finally:
        port.close()


if __name__ == "__main__":
    main()
