#!/usr/bin/env python3
"""VW MEB rlog 리플레이 검증기 — 실차 rlog의 CAN을 carstate 파서에 먹여 파싱 상태를 판정한다.

용도: MK1/MK2 설치 차량에서 "CAN 오류"가 보고됐을 때, 기기 없이 rlog만으로
  - canValid 분포 (파서 타임아웃/CRC 문제 여부)
  - 기어 소스 값 (MK2: Gateway_73.GE_Fahrstufe / MK1: Getriebe_11)
  - 주소별 실측 수신 주기 (파서 기대 주기와 대조용)
를 확인한다. ID.4 MK2 최초 규명(기어 소스 이동 + 메시지 주기 불일치)에 사용했던 도구.
자세한 배경: docs/id4_mk2_gen2.md

사용법:
  python3 selfdrive/debug/vw_meb_mk2_replay.py [--mk1] <rlog.zst> [rlog2.zst ...]
"""
import sys
import types
from collections import Counter
from pathlib import Path

import capnp
import zstandard

REPO_ROOT = Path(__file__).resolve().parents[2]

# params_pyx는 기기 빌드 산출물이라 PC에서 import가 안 됨 -> 스텁 주입
stub = types.ModuleType("openpilot.common.params_pyx")
class _P:
  def __init__(self, *a): pass
  def get_int(self, k, d=0): return 0
  def get(self, k, *a, **kw): return None
  def get_bool(self, k, *a): return False
stub.Params = _P
stub.ParamKeyFlag = stub.ParamKeyType = stub.UnknownKeyName = Exception
sys.modules["openpilot.common.params_pyx"] = stub

capnp.remove_import_hook()
log_capnp = capnp.load(str(REPO_ROOT / "cereal/log.capnp"),
                       imports=[str(REPO_ROOT / "opendbc_repo/opendbc/car")])

sys.path.insert(0, str(REPO_ROOT / "opendbc_repo"))
sys.path.insert(0, str(REPO_ROOT))

from opendbc.car import Bus                              # noqa: E402
from opendbc.car.volkswagen.carstate import CarState     # noqa: E402
from opendbc.car.volkswagen.interface import CarInterface  # noqa: E402
from opendbc.car.volkswagen.values import CAR            # noqa: E402


def read_events(path):
  with open(path, "rb") as f:
    raw = zstandard.ZstdDecompressor().stream_reader(f).read()
  try:
    yield from log_capnp.Event.read_multiple_bytes(raw)
  except Exception:
    # 세그먼트 꼬리가 잘린 rlog는 흔함 - 읽은 데까지만 사용
    return


def main():
  args = [a for a in sys.argv[1:] if not a.startswith("--")]
  mk1 = "--mk1" in sys.argv
  if not args:
    print(__doc__)
    sys.exit(1)

  car = CAR.VOLKSWAGEN_ID4_MK1 if mk1 else CAR.VOLKSWAGEN_ID4_MK2
  gear_msg, gear_sig = ("Getriebe_11", "GE_Fahrstufe") if mk1 else ("Gateway_73", "GE_Fahrstufe")

  # 실제 rlog에서 버스별 주소를 스캔해 핑거프린트를 구성 (하네스 오검출 방지).
  # bus1의 게이트웨이 지표(0x86/0xFD/0x520/0x13D)로 게이트웨이/카메라 하네스가 정확히 판정돼야
  # MEB_ACC_01/MEB_Side_Assist_01이 올바른 버스(pt vs cam) 파서에 등록된다.
  fingerprint: dict[int, dict[int, int]] = {0: {}, 1: {}, 2: {}}
  for path in args:
    for ev in read_events(path):
      if ev.which() != "can":
        continue
      for m in ev.can:
        if m.src in (0, 1, 2):
          fingerprint[m.src][m.address] = len(m.dat)
    if fingerprint[1]:  # 게이트웨이 판정에 충분하면 조기 종료
      break

  cp = CarInterface.get_params(car, fingerprint, [], True, False, True, None)
  cp.enableBsm = True
  print(f"networkLocation={cp.networkLocation}")
  cs = CarState(cp)
  parsers = cs.get_can_parsers(cp)
  pt, cam = parsers[Bus.pt], parsers[Bus.cam]
  print(f"car={car} | pt msgs={len(pt.vl)} cam msgs={len(cam.vl)}")

  valid_hist = Counter()
  addr_count = Counter()
  gear_vals = set()
  n_updates = 0
  t_first = t_last = None

  for path in args:
    for ev in read_events(path):
      if ev.which() != "can":
        continue
      t = ev.logMonoTime
      t_first = t if t_first is None else t_first
      t_last = t
      by_bus = {0: [], 2: []}
      for m in ev.can:
        if m.src in (0, 2):
          by_bus[m.src].append([m.address, bytes(m.dat), m.src])
          addr_count[(m.address, m.src)] += 1
      if by_bus[0]:
        pt.update([(t, by_bus[0])])
      if by_bus[2]:
        cam.update([(t, by_bus[2])])
      n_updates += 1
      if n_updates % 100 == 0:
        valid_hist[(pt.can_valid, cam.can_valid)] += 1
        try:
          gear_vals.add(int(pt.vl[gear_msg][gear_sig]))
        except (KeyError, TypeError):
          pass

  print(f"updates: {n_updates}")
  print(f"(pt_valid, cam_valid) 분포: {dict(valid_hist)}")
  print(f"기어 값들 ({gear_msg}.{gear_sig}): {sorted(gear_vals)}")

  if t_first is not None and t_last is not None and t_last > t_first:
    dur = (t_last - t_first) * 1e-9
    print(f"\n주소별 실측 주기 (구간 {dur:.1f}s, 상위 40개):")
    for (addr, src), cnt in sorted(addr_count.items(), key=lambda kv: -kv[1])[:40]:
      print(f"  0x{addr:03X} src{src}: {cnt / dur:6.1f} Hz")


if __name__ == "__main__":
  main()
