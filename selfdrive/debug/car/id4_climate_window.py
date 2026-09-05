#!/usr/bin/env python3
"""지정한 시각 창(window) 안에서 바뀐 CAN 바이트만 보여준다.
공조 조작 순간에 함께 변한 메시지 = 명령 후보.

사용: python3 climate_window.py <route> <seg> <t1> <t2> [t3 t4 ...]
      (창은 쌍으로: 시작,끝)
"""
import sys
from pathlib import Path
import glob
from collections import defaultdict

# Resolve the repo root from this file, so the tool works wherever the fork is installed
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from openpilot.tools.lib.logreader import LogReader

NAMES = {}
try:
    import opendbc, os
    for root, _, files in os.walk(os.path.dirname(opendbc.__file__)):
        if 'vw_meb.dbc' in files:
            for line in open(os.path.join(root, 'vw_meb.dbc'), encoding='utf-8', errors='replace'):
                if line.startswith('BO_ '):
                    p = line.split(); NAMES[int(p[1])] = p[2].rstrip(':')
            break
except Exception:
    pass

route, seg = sys.argv[1], sys.argv[2]
bounds = [float(x) for x in sys.argv[3:]]
windows = list(zip(bounds[0::2], bounds[1::2]))

files = glob.glob(f'/data/media/0/realdata/{route}--{seg}/rlog*')
if not files:
    print("no rlog"); sys.exit(1)

last = {}
changes = defaultdict(list)   # (bus,addr,byte) -> [(t, old, new)]
t0 = None
for m in LogReader(files[0]):
    if m.which() != 'can':
        continue
    t = m.logMonoTime / 1e9
    if t0 is None:
        t0 = t
    rel = t - t0
    for f in m.can:
        if f.src > 2:
            continue
        key = (int(f.src), int(f.address))
        dat = bytes(f.dat)
        prev = last.get(key)
        if prev is not None and len(prev) == len(dat):
            for i, (a, b) in enumerate(zip(prev, dat)):
                if a != b:
                    changes[(key[0], key[1], i)].append((rel, a, b))
        last[key] = dat

for w0, w1 in windows:
    print(f"\n===== 창 {w0:.1f}s ~ {w1:.1f}s =====")
    rows = []
    for (bus, addr, idx), evs in changes.items():
        inwin = [e for e in evs if w0 <= e[0] <= w1]
        if not inwin:
            continue
        # 창 밖에서도 자주 변하는 바이트는 노이즈 -> 제외
        outside = len(evs) - len(inwin)
        if outside > 6:
            continue
        rows.append((bus, addr, idx, inwin, outside))
    rows.sort(key=lambda r: (r[4], r[0], r[1]))
    if not rows:
        print("  (해당 없음)")
    for bus, addr, idx, inwin, outside in rows[:30]:
        name = NAMES.get(addr, "★미식별★")
        times = ", ".join(f"{t:.1f}s({a:02X}->{b:02X})" for t, a, b in inwin[:6])
        print(f"  bus{bus} 0x{addr:03X} [{name}] byte{idx}: {times}  (창밖 {outside}회)")
