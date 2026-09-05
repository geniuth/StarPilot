#!/usr/bin/env python3
"""ID.4(MEB) 원격 언락 재현 — 1-a단계: 잠금/해제 신호 정찰 (수신 전용).

목표: 스마트키로 차량을 잠그고/풀 때 Blinkmodi_02(0x366)의 BM_ZV_auf(언락),
BM_ZV_zu(락) 비트가 어떻게 움직이는지 관측한다.

판별해야 할 것:
  (A) 언락 순간에만 1로 튀는 '명령성/이벤트' 신호인가?
      -> 재생하면 실제 언락이 일어날 가능성이 있다.
  (B) 계속 유지되는 '상태' 신호인가?
      -> 재생해도 도어 컨트롤러는 이미 그 상태라 무반응일 수 있다.
  그리고 이 프레임이 어느 bus 에 보이는가? (재생은 그 bus 로 쏴야 한다)

*** 이 스크립트는 송신(TX)을 하지 않는다. 순수 수신/관측만이다. ***

주의: BM = Blinkmodi(깜빡이 모드), ZV = Zentralverriegelung(중앙잠금).
BM_ZV_auf 는 원래 '잠금해제에 수반되는 깜빡이(웰컴 라이팅)' 신호일 수 있다.
그렇다면 이건 언락의 '결과'이지 '명령'이 아니다. (A)/(B) 판별이 그래서 중요하다.

사용:
  python3 id4_unlock_scan.py                 # 120초, 값 표시
  python3 id4_unlock_scan.py --watch         # 값이 바뀌는 순간만 (키 조작 대조용)
  python3 id4_unlock_scan.py --duration 180
  python3 id4_unlock_scan.py --raw           # CANParser 대신 raw 파싱 폴백 사용
"""
import argparse
import sys
from pathlib import Path
import time

# Resolve the repo root from this file, so the tool works wherever the fork is installed
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from openpilot.cereal import messaging

# --- 대상 정의 -----------------------------------------------------------
# Blinkmodi_02 = 870 = 0x366, Gateway_72 = 987 = 0x3DB (둘 다 11bit 표준 ID)
BLINKMODI_02 = 0x366
GATEWAY_72 = 0x3DB

# CANParser 로 읽을 신호들
BLINK_SIGNALS = ["BM_ZV_auf", "BM_ZV_zu", "BM_DWA_ein", "BM_Warnblinken"]
DOOR_SIGNALS = ["ZV_FT_offen", "ZV_BT_offen", "ZV_HFS_offen", "ZV_HBFS_offen", "ZV_HD_offen"]

# raw 폴백용 비트 위치 (DBC "start|len@1+" 를 byte/bit 로 환산, little-endian)
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


def open_sock():
    # sample_vehicle_can 과 같은 패턴: 열고 곧바로 읽으면 구독 성립 전이라 빈다.
    sock = messaging.sub_sock("can", timeout=200)
    time.sleep(0.5)                 # 구독 성립 대기
    messaging.drain_sock(sock)      # 워밍업 중 쌓인 것 버림
    return sock


def scan_canparser(duration: float, watch: bool) -> None:
    """CANParser 로 Blinkmodi_02 / Gateway_72 를 bus 0/1/2 에서 관측."""
    try:
        from opendbc.can import CANParser
    except Exception as exc:
        print(f"[경고] CANParser 사용 불가({exc}) → raw 폴백으로 전환")
        scan_raw(duration, watch)
        return

    msgs = [("Blinkmodi_02", 0), ("Gateway_72", 0)]  # freq 0 = 주기 검사 안 함
    try:
        parsers = {b: CANParser("vw_meb", msgs, b) for b in (0, 1, 2)}
    except Exception as exc:
        print(f"[경고] CANParser 생성 실패({exc}) → raw 폴백으로 전환")
        scan_raw(duration, watch)
        return

    sock = open_sock()
    started = time.monotonic()
    deadline = started + duration
    # (bus, signal) -> 마지막 값 / 프레임 관측 여부
    last: dict[tuple[int, str], int] = {}
    seen_frame: dict[tuple[int, str], int] = {}   # (bus, msg) -> 관측 횟수
    change_log: list[str] = []
    next_tick = started + 10.0

    print(f"관측 시작 — {duration:.0f}초  ({'변화 순간만' if watch else '값 표시'})\n")

    while time.monotonic() < deadline:
        pkts = messaging.drain_sock(sock)
        if not pkts:
            continue
        frames = [(m.logMonoTime, [(f.address, f.dat, f.src) for f in m.can]) for m in pkts]
        rel = time.monotonic() - started
        for bus, cp in parsers.items():
            try:
                cp.update(frames)
            except Exception:
                continue
            for msg_name, sig_list in (("Blinkmodi_02", BLINK_SIGNALS),
                                       ("Gateway_72", DOOR_SIGNALS)):
                # vl_all = 이번 update 에서 실제 수신된 값들. 비어 있으면 미수신.
                got = cp.vl_all[msg_name]
                any_val = next((v for v in got.values() if v), None) if got else None
                if not got or all(len(v) == 0 for v in got.values()):
                    continue
                seen_frame[(bus, msg_name)] = seen_frame.get((bus, msg_name), 0) + 1
                for sig in sig_list:
                    seq = got.get(sig)
                    if not seq:
                        continue
                    v = int(seq[-1])
                    key = (bus, sig)
                    prev = last.get(key)
                    if prev is None:
                        last[key] = v
                        if not watch:
                            print(f"  [{rel:6.1f}s] bus{bus} {sig:16} = {v}  (최초)")
                    elif prev != v:
                        line = f"  [{rel:6.1f}s] bus{bus} {sig:16} {prev} -> {v}  ★변화"
                        print(line, flush=True)
                        change_log.append(line)
                        last[key] = v

        if not watch and time.monotonic() >= next_tick:
            print(f"  ... {time.monotonic()-started:4.0f}s 경과", flush=True)
            next_tick += 10.0

    report(seen_frame, last, change_log)


def scan_raw(duration: float, watch: bool) -> None:
    """CANParser 없이 raw can 소켓에서 0x366 / 0x3DB 를 직접 파싱하는 폴백."""
    sock = open_sock()
    started = time.monotonic()
    deadline = started + duration
    last: dict[tuple[int, str], int] = {}
    seen_frame: dict[tuple[int, int], int] = {}   # (bus, address)
    change_log: list[str] = []
    last_hex: dict[tuple[int, int], str] = {}

    print(f"관측 시작 (raw) — {duration:.0f}초\n")

    while time.monotonic() < deadline:
        pkts = messaging.drain_sock(sock)
        if not pkts:
            continue
        rel = time.monotonic() - started
        for m in pkts:
            for f in m.can:
                address = int(f.address)
                if address not in (BLINKMODI_02, GATEWAY_72):
                    continue
                bus = int(f.src)
                payload = bytes(f.dat)
                seen_frame[(bus, address)] = seen_frame.get((bus, address), 0) + 1

                if address == BLINKMODI_02:
                    for sig, (bi, bit_i) in RAW_BITS.items():
                        v = bit(payload, bi, bit_i)
                        key = (bus, sig)
                        prev = last.get(key)
                        if prev is None:
                            last[key] = v
                            if not watch:
                                print(f"  [{rel:6.1f}s] bus{bus} {sig:12} = {v}  (최초) {hexs(payload)}")
                        elif prev != v:
                            line = (f"  [{rel:6.1f}s] bus{bus} {sig:12} {prev} -> {v}  "
                                    f"★변화  {hexs(payload)}")
                            print(line, flush=True)
                            change_log.append(line)
                            last[key] = v
                    # 전체 페이로드 변화도 감시(우리가 모르는 비트가 함께 움직이는지)
                    hk = (bus, address)
                    hx = hexs(payload)
                    if last_hex.get(hk) not in (None, hx) and not watch:
                        print(f"  [{rel:6.1f}s] bus{bus} 0x366 payload -> {hx}")
                    last_hex[hk] = hx

    print("\n=== raw 관측 요약 ===")
    for (bus, addr), n in sorted(seen_frame.items()):
        print(f"  0x{addr:03X} bus{bus}: {n}회")
    _print_changes(change_log)


def report(seen_frame, last, change_log) -> None:
    print("\n" + "=" * 74)
    print("관측 요약")
    print("=" * 74)
    if not seen_frame:
        print("\n  Blinkmodi_02 / Gateway_72 를 어느 bus 에서도 못 받았다.")
        print("  · 차량이 잠들어 CAN 이 조용하거나")
        print("  · 게이트웨이 하네스가 해당 버스를 넘기지 않는다.")
        print("  --raw 로 다시 시도하거나, 차 시동/도어 조작 중에 재실행할 것.")
        return

    print("\n  [프레임이 보인 bus]")
    for (bus, msg), n in sorted(seen_frame.items()):
        print(f"    {msg:14} bus{bus}: {n}회 수신")

    print("\n  [마지막 값]")
    for (bus, sig), v in sorted(last.items()):
        print(f"    bus{bus} {sig:16} = {v}")

    _print_changes(change_log)
    _print_verdict(change_log)


def _print_changes(change_log) -> None:
    print("\n  [변화 이력]")
    if not change_log:
        print("    (관측 중 변화 없음 — 키 조작을 안 했거나 신호가 상수)")
    else:
        for line in change_log:
            print("   " + line.strip())


def _print_verdict(change_log) -> None:
    print("\n" + "=" * 74)
    print("판별 힌트")
    print("=" * 74)
    auf_changes = [l for l in change_log if "BM_ZV_auf" in l]
    if not auf_changes:
        print("  BM_ZV_auf 변화가 안 잡혔다. 재생 대상 신호를 확정할 수 없다.")
        print("  키로 확실히 '잠금 -> 해제'를 하며 다시 관측할 것.")
        return
    # 0->1->0 처럼 잠깐 튀면 (A) 명령/이벤트, 1로 계속 있으면 (B) 상태
    ones = sum(1 for l in auf_changes if "-> 1" in l)
    zeros = sum(1 for l in auf_changes if "-> 0" in l)
    print(f"  BM_ZV_auf: 0->1 {ones}회, 1->0 {zeros}회 관측")
    if ones and zeros:
        print("  => (A) 언락 순간 잠깐 1로 튀는 이벤트성 신호로 보인다.")
        print("        재생하면 언락이 일어날 가능성이 있다 (1-b 로 검증).")
    elif ones and not zeros:
        print("  => 1로 올라간 뒤 유지. (B) 상태 신호일 수 있다.")
        print("        재생해도 이미 그 상태라 무반응일 가능성. 그래도 1-b 로 확인 가치 있음.")
    else:
        print("  => 패턴이 애매하다. 변화 이력을 키 조작 시점과 대조해 직접 판단할 것.")


def main() -> int:
    p = argparse.ArgumentParser(description="ID.4 언락 신호 정찰 (수신 전용)")
    p.add_argument("--duration", type=float, default=120.0, help="관측 시간(초), 기본 120")
    p.add_argument("--watch", action="store_true", help="값이 바뀌는 순간만 출력")
    p.add_argument("--raw", action="store_true", help="CANParser 대신 raw 파싱 폴백 사용")
    args = p.parse_args()

    print("=" * 74)
    print("ID.4 언락 신호 정찰 — 수신 전용 (TX 없음)")
    print("=" * 74)
    print("스마트키로 '잠금 -> 해제'를 몇 번 반복하며 관측하세요.\n")

    try:
        if args.raw:
            scan_raw(args.duration, args.watch)
        else:
            scan_canparser(args.duration, args.watch)
    except KeyboardInterrupt:
        print("\n중단됨")
        return 130

    print("\n끝 — 어떤 프레임도 송신하지 않았다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
