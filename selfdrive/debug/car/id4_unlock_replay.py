#!/usr/bin/env python3
"""ID.4(MEB) 원격 언락 재현 — 저장된 rlog 에서 언락 순간 분석 (수신 전용).

id4_unlock_scan.py 의 라이브 버전과 짝이다. 차 앞에서 SSH 붙어 있을 필요 없이,
주차장에서 스마트키로 락/언락을 몇 번 하고 오면 그 프레임이 rlog 에 남는다.
(콤마가 주차 중에도 깨어 있을 때. 절전 들어가기 전에 조작해야 함)

이 스크립트가 하는 일:
  - rlog 에서 Blinkmodi_02(0x366) 의 BM_ZV_auf(언락)/BM_ZV_zu(락) 변화를 찾는다
  - 0->1 로 바뀌는 '언락 순간'을 자동 검출하고, 그때의 정확한 8바이트를 뽑는다
  - 그 페이로드가 재생 검증(1-b, id4_unlock_replay_test.py --payload)의 입력이 된다
  - (A)이벤트/(B)상태 판별도 자동으로 낸다

*** 로그를 읽기만 한다. 송신(TX)은 없다. ***

사용:
  python3 id4_unlock_replay.py <route>            # 세그먼트 0~80
  python3 id4_unlock_replay.py <route> 20         # 0~20
  python3 id4_unlock_replay.py <route> 10 30      # 10~30
  python3 id4_unlock_replay.py <route> 0 80 --all # 0x366 프레임 전부 덤프

실행은 openpilot venv 로:
  cd <repo> && export PYTHONPATH=<repo>
  /usr/local/venv/bin/python3 selfdrive/debug/car/id4_unlock_replay.py <route>
"""
import argparse
import glob
import sys
from pathlib import Path

# Resolve the repo root from this file, so the tool works wherever the fork is installed
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from openpilot.tools.lib.logreader import LogReader

REALDATA = "/data/media/0/realdata"

BLINKMODI_02 = 0x366
GATEWAY_72 = 0x3DB

# DBC "start|len@1+" 를 byte/bit 로 환산(little-endian). unlock_scan 과 동일.
#   BM_ZV_auf : 12|1 -> byte 1, bit 4
#   BM_ZV_zu  : 13|1 -> byte 1, bit 5
RAW_BITS = {
    "BM_ZV_auf": (1, 4),
    "BM_ZV_zu": (1, 5),
}


def bit(payload: bytes, byte_idx: int, bit_idx: int) -> int:
    if byte_idx >= len(payload):
        return -1
    return (payload[byte_idx] >> bit_idx) & 1


def hexs(payload: bytes) -> str:
    return payload.hex(" ").upper()


def analyze(route: str, seg_from: int, seg_to: int, dump_all: bool) -> bool:
    """rlog 를 순회하며 0x366 의 언락/락 변화를 찾는다. 언락 검출되면 True."""
    # (bus, signal) -> 마지막 값
    last: dict[tuple[int, str], int] = {}
    # bus -> 마지막 payload (변화 순간 페이로드 추출용)
    changes: list[tuple[float, int, str, int, int, bytes]] = []  # (t, bus, sig, prev, now, payload)
    seen_bus: dict[int, int] = {}
    all_frames: list[tuple[float, int, bytes]] = []
    t0 = None
    done_segments = 0

    for seg in range(seg_from, seg_to + 1):
        files = glob.glob(f"{REALDATA}/{route}--{seg}/rlog*")
        if not files:
            continue
        try:
            for m in LogReader(files[0]):
                if m.which() != "can":
                    continue
                t = m.logMonoTime / 1e9
                if t0 is None:
                    t0 = t
                rel = t - t0
                for f in m.can:
                    if int(f.address) != BLINKMODI_02:
                        continue
                    bus = int(f.src)
                    payload = bytes(f.dat)
                    seen_bus[bus] = seen_bus.get(bus, 0) + 1
                    if dump_all:
                        all_frames.append((rel, bus, payload))
                    for sig, (bi, bit_i) in RAW_BITS.items():
                        v = bit(payload, bi, bit_i)
                        key = (bus, sig)
                        prev = last.get(key)
                        if prev is not None and prev != v:
                            changes.append((rel, bus, sig, prev, v, payload))
                        last[key] = v
        except Exception as exc:
            print(f"  seg {seg}: 읽기 실패 ({exc})")
            continue
        done_segments += 1

    if done_segments == 0:
        print(f"\n[중단] rlog 를 못 찾았다: {REALDATA}/{route}--*")
        print("  루트 목록:  ls " + REALDATA + " | sed 's/--[0-9]*$//' | sort -u | tail")
        return False

    print(f"세그먼트 {done_segments}개 스캔")
    print("\n  [0x366 이 보인 bus]")
    if not seen_bus:
        print("    0x366(Blinkmodi_02) 프레임이 이 구간에 없다.")
        print("    · 콤마가 조작 시점에 잠들어 있었거나")
        print("    · 이 세그먼트에 락/언락 조작이 안 들어갔다.")
        return False
    for bus, n in sorted(seen_bus.items()):
        tag = " (판다 에코, 무시)" if bus >= 128 else ""
        print(f"    bus{bus}: {n}회{tag}")

    print("\n  [BM_ZV_auf / BM_ZV_zu 변화 이력]")
    if not changes:
        print("    변화 없음 — 락/언락 조작이 이 구간에 안 담겼다.")
    else:
        for rel, bus, sig, prev, now, payload in changes:
            star = "  ★언락!" if (sig == "BM_ZV_auf" and now == 1) else ""
            print(f"    [{rel:7.1f}s] bus{bus} {sig:10} {prev}->{now}  {hexs(payload)}{star}")

    # 언락 순간(0->1) 페이로드 추출 — 재생 검증 입력
    unlocks = [(rel, bus, payload) for rel, bus, sig, prev, now, payload in changes
               if sig == "BM_ZV_auf" and now == 1]
    print("\n" + "=" * 74)
    print("재생용 언락 페이로드")
    print("=" * 74)
    if unlocks:
        # 실제 버스(< 128)를 우선한다
        real = [(rel, bus, p) for rel, bus, p in unlocks if bus < 128]
        pick = real[0] if real else unlocks[0]
        rel, bus, payload = pick
        print(f"  언락 순간 {len(unlocks)}회 검출. 대표 페이로드:")
        print(f"    bus{bus}  {hexs(payload)}   (@ {rel:.1f}s)")
        print(f"\n  → 재생 검증(1-b)에 이대로 넣으면 된다:")
        print(f"    python3 id4_unlock_replay_test.py --bus {bus} "
              f"--payload {payload.hex()} --confirm")
    else:
        print("  BM_ZV_auf 0->1 을 못 찾았다. 재생할 언락 페이로드를 확정할 수 없다.")
        print("  스마트키로 '잠금 -> 해제'를 확실히 하며 다시 로그를 남길 것.")

    # (A)/(B) 판별
    print("\n" + "=" * 74)
    print("판별")
    print("=" * 74)
    auf = [c for c in changes if c[2] == "BM_ZV_auf"]
    ones = sum(1 for c in auf if c[4] == 1)
    zeros = sum(1 for c in auf if c[4] == 0)
    print(f"  BM_ZV_auf: 0->1 {ones}회, 1->0 {zeros}회")
    if ones and zeros:
        print("  => (A) 언락 순간 잠깐 1로 튀는 이벤트성 신호. 재생하면 언락될 가능성 높음.")
    elif ones and not zeros:
        print("  => 1로 올라간 뒤 유지. (B) 상태 신호일 수 있음(재생 무반응 가능).")
    elif not auf:
        print("  => 변화 없음. 조작이 안 담겼거나 신호가 상수.")
    else:
        print("  => 패턴 애매. 위 변화 이력을 조작 시각과 대조할 것.")

    if dump_all and all_frames:
        print("\n" + "=" * 74)
        print(f"0x366 전체 덤프 ({len(all_frames)}프레임)")
        print("=" * 74)
        for rel, bus, payload in all_frames:
            print(f"  [{rel:7.1f}s] bus{bus}  {hexs(payload)}")

    return bool(unlocks)


def main() -> int:
    p = argparse.ArgumentParser(description="ID.4 언락 신호 rlog 분석 (수신 전용)")
    p.add_argument("route", help="루트 이름 (예: 0000006c--0540efccb2)")
    p.add_argument("seg_from", nargs="?", type=int, default=0)
    p.add_argument("seg_to", nargs="?", type=int, default=None)
    p.add_argument("--all", action="store_true", help="0x366 프레임 전부 덤프")
    args = p.parse_args()

    if args.seg_to is None:
        seg_from, seg_to = (0, args.seg_from) if args.seg_from else (0, 80)
    else:
        seg_from, seg_to = args.seg_from, args.seg_to

    print("=" * 74)
    print("ID.4 언락 신호 rlog 분석 — 수신 전용 (TX 없음)")
    print("=" * 74)
    print(f"루트    : {args.route}")
    print(f"세그먼트: {seg_from} ~ {seg_to}\n")

    analyze(args.route, seg_from, seg_to, args.all)
    print("\n끝 — 로그를 읽기만 했다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
