#!/usr/bin/env python3
"""공조 조작 시점에 변하는 CAN 바이트를 찾아낸다 (변화점 탐지).

사용법:
  1) 차 시동 켜고 openpilot이 rlog를 기록하는 상태에서
  2) 아래 순서로 공조를 조작 (각 동작 사이 10초 정지)
       ① 10초 대기(아무것도 안 함)  ② 에어컨 OFF  ③ 10초  ④ 에어컨 ON
       ⑤ 10초  ⑥ 송풍 최대  ⑦ 10초  ⑧ 송풍 최소
       ⑨ 10초  ⑩ 운전석 시트히터 ON  ⑪ 10초  ⑫ 시트히터 OFF
  3) 해당 세그먼트 번호로 이 스크립트 실행

  python3 climate_diff.py <route> <seg> [--max-changes 40]

출력: "가끔만 바뀌는" 바이트(=버튼/상태 후보)와 변화 시각.
      속도처럼 계속 변하는 바이트는 자동 제외한다.
"""
import argparse
import glob
import sys
from pathlib import Path
from collections import defaultdict

# Resolve the repo root from this file, so the tool works wherever the fork is installed
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from openpilot.tools.lib.logreader import LogReader

KNOWN_NAMES = {}
try:
    import opendbc, os
    for root, _, files in os.walk(os.path.dirname(opendbc.__file__)):
        if 'vw_meb.dbc' in files:
            with open(os.path.join(root, 'vw_meb.dbc'), encoding='utf-8', errors='replace') as f:
                for line in f:
                    if line.startswith('BO_ '):
                        p = line.split()
                        KNOWN_NAMES[int(p[1])] = p[2].rstrip(':')
            break
except Exception:
    pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("route")
    ap.add_argument("seg")
    ap.add_argument("--buses", default="0,1,2")
    ap.add_argument("--max-changes", type=int, default=40,
                    help="이보다 자주 바뀌는 바이트는 제외 (기본 40)")
    ap.add_argument("--min-changes", type=int, default=1)
    args = ap.parse_args()

    buses = {int(b) for b in args.buses.split(",")}
    path = f'/data/media/0/realdata/{args.route}--{args.seg}'
    files = glob.glob(f'{path}/rlog*')
    if not files:
        print("no rlog at", path)
        return 1

    t0 = None
    last = {}                       # (bus,addr) -> bytes
    changes = defaultdict(list)     # (bus,addr,byte_idx) -> [(t, old, new)]
    counts = defaultdict(int)

    for m in LogReader(files[0]):
        if m.which() != 'can':
            continue
        t = m.logMonoTime / 1e9
        if t0 is None:
            t0 = t
        rel = t - t0
        for f in m.can:
            if f.src not in buses:
                continue
            key = (int(f.src), int(f.address))
            dat = bytes(f.dat)
            counts[key] += 1
            prev = last.get(key)
            if prev is not None and len(prev) == len(dat):
                for i, (a, b) in enumerate(zip(prev, dat)):
                    if a != b:
                        changes[(key[0], key[1], i)].append((rel, a, b))
            last[key] = dat

    print(f"세그먼트 길이 ~{rel:.0f}초, 메시지 {len(counts)}종\n")
    print(f"=== 가끔만 바뀌는 바이트 (변화 {args.min_changes}~{args.max_changes}회) ===")
    rows = []
    for (bus, addr, idx), evs in changes.items():
        if args.min_changes <= len(evs) <= args.max_changes:
            rows.append((bus, addr, idx, evs))
    rows.sort(key=lambda r: (len(r[3]), r[0], r[1]))

    for bus, addr, idx, evs in rows:
        name = KNOWN_NAMES.get(addr, "미식별")
        times = ", ".join(f"{t:.1f}s({a:02X}->{b:02X})" for t, a, b in evs[:8])
        more = "" if len(evs) <= 8 else f" ... +{len(evs)-8}"
        print(f"bus{bus} 0x{addr:03X} [{name}] byte{idx}: {len(evs)}회  {times}{more}")

    if not rows:
        print("(해당 없음 - --max-changes 를 늘려보세요)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
