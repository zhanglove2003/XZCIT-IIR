#!/usr/bin/env python3
"""C50C STM32F407VE serial flashing with whole-flash backup and readback."""
import argparse
import datetime
import hashlib
import os
import pathlib
import re
import subprocess
import sys
import tempfile

BASE = 0x08000000
FLASH_SIZE = 0x80000
PARAM_OFFSET = 0x60000  # sector 7: HARDWARE/stmflash.c FLASH_SAVE_ADDR
SECTOR_ENDS = (0x4000, 0x8000, 0xC000, 0x10000, 0x20000, 0x40000, 0x60000)
ENTRY = 'dtr,-rts,rts,-dtr,dtr,-rts'


def validate_image(image):
    if not (8 <= len(image) <= PARAM_OFFSET):
        raise ValueError('image size overlaps parameter sector or is too small')
    sp = int.from_bytes(image[:4], 'little')
    reset = int.from_bytes(image[4:8], 'little')
    if not (0x20000000 < sp <= 0x20020000 and sp % 4 == 0):
        raise ValueError('invalid initial stack pointer')
    if not (reset & 1 and BASE <= (reset & ~1) < BASE + len(image)):
        raise ValueError('invalid reset vector')


def prepare_image(image, backup):
    validate_image(image)
    if len(backup) != FLASH_SIZE:
        raise ValueError('whole-flash backup must be exactly 512 KiB')
    end = next(x for x in SECTOR_ENDS if x >= len(image))
    sectors = list(range(SECTOR_ENDS.index(end) + 1))
    payload = image + backup[len(image):end]
    return {'sectors': sectors, 'payload': payload}


def prepare_from_region(image, region):
    validate_image(image)
    end = next(x for x in SECTOR_ENDS if x >= len(image))
    if len(region) != end:
        raise ValueError('quick baseline length must match the erased sector span')
    return {'sectors': list(range(SECTOR_ENDS.index(end) + 1)),
            'payload': image + region[len(image):]}


def invoke(executable, device, *args, timeout=240, baud=115200):
    cmd = [str(executable), '-b', str(baud), '-i', ENTRY, *args, str(device)]
    print(' '.join(cmd), flush=True)
    res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         text=True, timeout=timeout)
    print(res.stdout, flush=True)
    if res.returncode or not re.search(r'Device ID\s*:\s*0x0413\b', res.stdout, re.I):
        raise RuntimeError('stm32flash failed or did not identify device 0x0413')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--image', required=True)
    p.add_argument('--device', required=True)
    p.add_argument('--backup-dir', required=True)
    p.add_argument('--stm32flash', default='/home/orangepi/c50c-tools/bin/stm32flash')
    p.add_argument('--baud', type=int, choices=(57600, 115200, 230400), default=115200, help='serial baud rate (default: 115200)')
    p.add_argument('--quick-baseline', help='use a prior verified sector readback; compare device before erasing')
    p.add_argument('--execute', action='store_true', help='actually erase and program')
    a = p.parse_args()
    image = pathlib.Path(a.image).read_bytes()
    validate_image(image)
    print('image bytes:', len(image), 'sha256:', hashlib.sha256(image).hexdigest())
    print('maximum firmware end:', hex(BASE + PARAM_OFFSET), '(sector 7 preserved)')
    print('baud:', a.baud)
    print('backup mode:', 'quick with verified baseline' if a.quick_baseline else 'full 512 KiB')
    if a.quick_baseline:
        print('quick baseline:', a.quick_baseline)
    if a.baud > 115200:
        print('experimental baud above ST AN3155 highest tested rate')
    if not a.execute:
        print('Preview only: no serial device opened, no data written.')
        return 0
    directory = pathlib.Path(a.backup_dir)
    directory.mkdir(parents=True, exist_ok=True)
    stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    backup_path = directory / ('flash-before-' + stamp + '.bin')
    payload_path = directory / ('flash-payload-' + stamp + '.bin')
    readback_path = directory / ('flash-readback-' + stamp + '.bin')
    try:
        # Keep the application stopped between the comparison/backup and update.
        if a.quick_baseline:
            baseline_path = pathlib.Path(a.quick_baseline)
            baseline = baseline_path.read_bytes()
            plan = prepare_from_region(image, baseline)
            print('quick baseline sha256:', hashlib.sha256(baseline).hexdigest(), flush=True)
            with tempfile.TemporaryDirectory(prefix='c50c-precheck-') as tmp:
                precheck_path = pathlib.Path(tmp) / 'current.bin'
                invoke(a.stm32flash, a.device, '-r', str(precheck_path),
                       '-S', f'0x08000000:{len(baseline)}', baud=a.baud)
                if precheck_path.read_bytes() != baseline:
                    raise RuntimeError('current flash differs from quick baseline; use full backup')
            print('current erased-sector region matches baseline; full backup skipped', flush=True)
        else:
            invoke(a.stm32flash, a.device, '-r', str(backup_path), '-S', '0x08000000:0x80000', baud=a.baud)
            backup = backup_path.read_bytes()
            plan = prepare_image(image, backup)
            print('backup:', backup_path, 'sha256:', hashlib.sha256(backup).hexdigest(), flush=True)
        with payload_path.open('xb') as f:
            f.write(plan['payload'])
            f.flush()
            os.fsync(f.fileno())
        print('erase sectors:', plan['sectors'], 'sector payload bytes:', len(plan['payload']), flush=True)
        invoke(a.stm32flash, a.device, '-w', str(payload_path), '-v',
               '-e', str(len(plan['sectors'])), '-n', '0', timeout=480, baud=a.baud)
        invoke(a.stm32flash, a.device, '-r', str(readback_path),
               '-S', f'0x08000000:{len(plan["payload"])}', baud=a.baud)
        readback = readback_path.read_bytes()
        if readback != plan['payload']:
            raise RuntimeError('independent readback mismatch')
        print('full erased-sector readback matches', flush=True)
        invoke(a.stm32flash, a.device, '-g', '0', baud=a.baud)
        print('GO acknowledged; application operation must be checked separately', flush=True)
    except Exception as exc:
        print('FAILED:', type(exc).__name__, str(exc), file=sys.stderr, flush=True)
        print('Do not assume application is intact. Preserve recovery baseline:',
              a.quick_baseline if a.quick_baseline else backup_path,
              file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
