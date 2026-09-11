"""Reject FENCE instructions in the S3 ULP executable (including inline asm).

The assembler accepts these even though the observed S3 ULP traps on FENCE.
Run with the ESP-IDF Python environment, which provides pyelftools.
"""
from pathlib import Path
import sys
from elftools.elf.elffile import ELFFile


def check(path):
    checked = 0
    with Path(path).open('rb') as source:
        elf = ELFFile(source)
        if elf['e_machine'] != 'EM_RISCV':
            raise ValueError('Expected a RISC-V ULP ELF')
        for section in elf.iter_sections():
            if not section['sh_flags'] & 4:  # SHF_EXECINSTR; exclude data words
                continue
            code = section.data()
            offset = 0
            while offset + 2 <= len(code):
                half = int.from_bytes(code[offset:offset + 2], 'little')
                size = 4 if half & 3 == 3 else 2  # RV32IMC
                if offset + size > len(code):
                    raise ValueError('Truncated ULP instruction')
                if size == 4:
                    word = int.from_bytes(code[offset:offset + 4], 'little')
                    if word & 0x7f == 0x0f:  # MISC-MEM: FENCE / FENCE.I
                        address = section['sh_addr'] + offset
                        raise ValueError(f'ULP FENCE at 0x{address:x}: 0x{word:08x}; use compiler ordering and volatile RTC accesses')
                offset += size
                checked += 1
    if not checked:
        raise ValueError('No executable ULP instructions found')
    print(f'ULP instruction check passed: {checked} instructions, no FENCE')


if __name__ == '__main__':
    try:
        check(sys.argv[1])
    except (ValueError, IndexError) as error:
        print(f'ULP instruction check failed: {error}', file=sys.stderr)
        raise SystemExit(1)
