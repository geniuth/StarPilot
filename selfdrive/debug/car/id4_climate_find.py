#!/usr/bin/env python3
"""공조 '조작' 흔적만 골라낸다: 에어컨 스위치 토글 / 시트히터 단수 / 송풍 급변.
오토 공조의 완만한 변화(0.1V씩)는 무시하고 사람이 누른 흔적만 보여준다.

사용: python3 climate_find.py <route> [끝세그]
"""
import sys
from pathlib import Path
import glob

# Resolve the repo root from this file, so the tool works wherever the fork is installed
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from openpilot.tools.lib.logreader import LogReader
from opendbc.can import CANParser

ROUTE = sys.argv[1]
SMAX = int(sys.argv[2]) if len(sys.argv) > 2 else 80
BLOWER_JUMP = 0.8   # 이 이상 급변하면 수동 조작으로 본다

MSGS = [("Klima_11", 5), ("Klima_12", 5)]
hits = 0

for seg in range(0, SMAX + 1):
    files = glob.glob(f'/data/media/0/realdata/{ROUTE}--{seg}/rlog*')
    if not files:
        continue
    parsers = {b: CANParser("vw_meb", MSGS, b) for b in (0, 1)}
    last = {}
    events = []
    t0 = None
    try:
        for m in LogReader(files[0]):
            if m.which() != 'can':
                continue
            t = m.logMonoTime / 1e9
            if t0 is None:
                t0 = t
            frames = [(m.logMonoTime, [(f.address, f.dat, f.src) for f in m.can])]
            for bus, cp in parsers.items():
                try:
                    cp.update(frames)
                except Exception:
                    continue
                # 에어컨 스위치
                if cp.vl_all["Klima_11"].get("KL_AC_Schalter"):
                    v = cp.vl["Klima_11"]["KL_AC_Schalter"]
                    k = (bus, "AC")
                    if k in last and last[k] != v:
                        events.append((t - t0, f"에어컨 스위치 {int(last[k])} -> {int(v)}", bus))
                    last[k] = v
                # 시트히터 좌/우
                for sig, label in (("KL_SIH_Soll_li", "시트히터(좌)"), ("KL_SIH_Soll_re", "시트히터(우)")):
                    if cp.vl_all["Klima_12"].get(sig):
                        v = cp.vl["Klima_12"][sig]
                        k = (bus, sig)
                        if k in last and last[k] != v:
                            events.append((t - t0, f"{label} {int(last[k])} -> {int(v)}단", bus))
                        last[k] = v
                # 송풍 급변
                if cp.vl_all["Klima_12"].get("KL_Geblspng_Soll"):
                    v = cp.vl["Klima_12"]["KL_Geblspng_Soll"]
                    k = (bus, "BLW")
                    if k in last and abs(last[k] - v) >= BLOWER_JUMP:
                        events.append((t - t0, f"송풍 급변 {last[k]:.2f}V -> {v:.2f}V", bus))
                    last[k] = v
    except Exception as e:
        print(f"seg {seg}: {e}")
        continue

    if events:
        hits += 1
        print(f"\n===== seg {seg} =====")
        for rel, desc, bus in events[:20]:
            print(f"  {rel:6.1f}s  bus{bus}  {desc}")
        if len(events) > 20:
            print(f"  ... +{len(events)-20}건")

if hits == 0:
    print("조작 흔적 없음")
