#!/usr/bin/env python3
"""관측된 CAN 주소 중 DBC에 정의되지 않은 것(미식별)을 버스별로 뽑는다.
공조 '명령' 메시지 후보를 좁히는 용도."""
import sys
from pathlib import Path
import glob
from collections import defaultdict

# Resolve the repo root from this file, so the tool works wherever the fork is installed
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from openpilot.tools.lib.logreader import LogReader
from opendbc.can import CANDefine
try:
    from opendbc.can.parser import dbc_lookup  # noqa
except Exception:
    dbc_lookup = None

ROUTE = sys.argv[1] if len(sys.argv) > 1 else '00000030--455ec8c01f'
SEG = sys.argv[2] if len(sys.argv) > 2 else '5'

# DBC 정의 주소 수집
defined = {}
try:
    from opendbc.can import CANParser
    import opendbc
    import os
    dbc_path = None
    for root, _, files in os.walk(os.path.dirname(opendbc.__file__)):
        if 'vw_meb.dbc' in files:
            dbc_path = os.path.join(root, 'vw_meb.dbc')
            break
    if dbc_path:
        with open(dbc_path, encoding='utf-8', errors='replace') as f:
            for line in f:
                if line.startswith('BO_ '):
                    parts = line.split()
                    addr = int(parts[1])
                    name = parts[2].rstrip(':')
                    defined[addr] = name
    print(f"DBC 정의 메시지: {len(defined)}개  ({dbc_path})")
except Exception as e:
    print("DBC 로드 실패:", e)

path = f'/data/media/0/realdata/{ROUTE}--{SEG}'
files = glob.glob(f'{path}/rlog*')
if not files:
    print("no rlog", path); sys.exit(1)

seen = defaultdict(lambda: defaultdict(int))
lens = {}
for m in LogReader(files[0]):
    if m.which() != 'can':
        continue
    for f in m.can:
        if f.src > 2:      # TX 에코 제외
            continue
        seen[int(f.src)][int(f.address)] += 1
        lens[(int(f.src), int(f.address))] = len(f.dat)

for bus in sorted(seen):
    unknown = [(a, c) for a, c in sorted(seen[bus].items()) if a not in defined and a < 0x800]
    known = [a for a in seen[bus] if a in defined]
    print(f"\n===== bus {bus}: 총 {len(seen[bus])}, DBC정의 {len(known)}, 미식별 {len(unknown)} =====")
    for a, c in unknown:
        print(f"  0x{a:03X} ({a:4d})  len={lens[(bus,a)]:2d}  frames={c}")
