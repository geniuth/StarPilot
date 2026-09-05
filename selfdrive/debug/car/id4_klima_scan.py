#!/usr/bin/env python3
"""ID.4(MEB) 공조 제어 프레임 정찰 — 라이브 CAN 수신 전용.

jagheterfredrik/meb-preheat 가 리버싱한 확장 ID 프레임들이 우리 콤마의
어느 버스에 보이는지 확인한다. 이 ID들은 vw_meb.dbc 에 없어서 CANParser 로는
안 잡히므로 raw 로 훑는다.

  0x16A954FB  공조 온도/제어 명령 (30단계: LO, 16.0~29.5, HI)
  0x16A95493  공조 가동 상태 (byte0: 0x01=ON, 0x00=OFF)
  0x1B000010  CAN 슬립 모드 제어 (버스 웨이크용)

*** 이 스크립트는 송신(TX)을 하지 않는다. 순수 수신/관측만이다. ***

사용:
  python3 id4_klima_scan.py                    # 60초 스캔
  python3 id4_klima_scan.py --duration 120
  python3 id4_klima_scan.py --all              # 확장 ID 전체 덤프
  python3 id4_klima_scan.py --watch 0x16A954FB # 특정 ID 실시간 (수동 조작 대조용)
"""
import argparse
import sys
from pathlib import Path
import time
from collections import defaultdict

# Resolve the repo root from this file, so the tool works wherever the fork is installed
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from openpilot.cereal import messaging

# --- 타겟 ---------------------------------------------------------------
TARGETS = {
    0x16A954FB: "공조 온도/제어 명령 (30단계: LO, 16.0~29.5, HI)",
    0x16A95493: "공조 가동 상태 (byte0: 0x01=ON, 0x00=OFF)",
    0x1B000010: "CAN 슬립 모드 제어 (버스 웨이크용)",
}
# 같은 패밀리도 함께 본다. 0x16A954AD(MEB_AWV_01)는 이미 DBC/판다 TX 화이트리스트에 있다.
FAMILY_MASKS = (
    (0xFFFFFF00, 0x16A95400, "0x16A954xx"),
    (0xFFFFFF00, 0x1B000000, "0x1B0000xx"),
)
EXT_ID_MIN = 0x800  # 이보다 크면 확장 ID로 본다


class Track:
    """(address, bus) 하나의 관측 이력."""

    __slots__ = ("first_payload", "count", "first_t", "last_t", "changed_bytes",
                 "last_payload", "length")

    def __init__(self, payload: bytes, t: float):
        self.first_payload = payload
        self.last_payload = payload
        self.length = len(payload)
        self.count = 1
        self.first_t = t
        self.last_t = t
        self.changed_bytes: set[int] = set()

    def add(self, payload: bytes, t: float) -> None:
        self.count += 1
        self.last_t = t
        # 길이가 바뀌면 비교가 무의미하니 갱신만 한다
        if len(payload) == len(self.last_payload):
            for i, (a, b) in enumerate(zip(self.last_payload, payload)):
                if a != b:
                    self.changed_bytes.add(i)
        self.last_payload = payload

    @property
    def hz(self) -> float:
        span = self.last_t - self.first_t
        return (self.count - 1) / span if span > 0.2 and self.count > 1 else 0.0


def family_of(address: int) -> str | None:
    for mask, value, label in FAMILY_MASKS:
        if (address & mask) == value:
            return label
    return None


def hexs(payload: bytes) -> str:
    return payload.hex(" ").upper()


def scan(duration: float, watch: int | None) -> dict:
    """지정 시간 동안 raw CAN 을 훑는다. 반환: {(address, bus): Track}"""
    # wayon_vehicle_telemetry.sample_vehicle_can 과 같은 패턴.
    # 소켓을 열고 곧바로 읽으면 구독이 성립되기 전이라 아무것도 못 받는다.
    sock = messaging.sub_sock("can", timeout=200)
    time.sleep(0.5)                 # 구독 성립 대기
    messaging.drain_sock(sock)      # 워밍업 중 쌓인 것 버림

    tracks: dict[tuple[int, int], Track] = {}
    started = time.monotonic()
    deadline = started + duration
    total_frames = 0
    next_tick = started + 5.0

    while time.monotonic() < deadline:
        msgs = messaging.drain_sock(sock)
        if not msgs:
            continue
        now = time.monotonic()
        for m in msgs:
            for f in m.can:
                address = int(f.address)
                bus = int(f.src)
                payload = bytes(f.dat)
                total_frames += 1

                if watch is not None and address == watch:
                    rel = now - started
                    print(f"  [{rel:7.2f}s] bus{bus}  {address:#010x}  "
                          f"len={len(payload)}  {hexs(payload)}", flush=True)

                # 확장 ID 만 수집한다(표준 ID 는 이미 DBC 로 보고 있다)
                if address < EXT_ID_MIN:
                    continue
                key = (address, bus)
                track = tracks.get(key)
                if track is None:
                    tracks[key] = Track(payload, now)
                    if address in TARGETS or family_of(address):
                        print(f"  [발견] {address:#010x} bus{bus} "
                              f"len={len(payload)} {hexs(payload)}", flush=True)
                else:
                    track.add(payload, now)

        if watch is None and time.monotonic() >= next_tick:
            elapsed = time.monotonic() - started
            print(f"  ... {elapsed:4.0f}s / {duration:.0f}s  "
                  f"프레임 {total_frames:,}개, 확장 ID {len(tracks)}종", flush=True)
            next_tick += 5.0

    print(f"\n총 {total_frames:,} 프레임 수신, 확장 ID {len(tracks)}종 관측")
    return tracks


def row(address: int, bus: int, track: Track) -> str:
    changed = ""
    if track.changed_bytes:
        changed = "byte " + ",".join(str(i) for i in sorted(track.changed_bytes))
    return (f"  {address:#010x}  bus{bus}  len={track.length}  "
            f"{track.count:6d}회  {track.hz:6.1f}Hz  "
            f"{hexs(track.first_payload):24}  {changed}")


def report(tracks: dict, dump_all: bool) -> None:
    print("\n" + "=" * 78)
    print("타겟 ID")
    print("=" * 78)
    found_targets = False
    for target, desc in TARGETS.items():
        hits = [(a, b, t) for (a, b), t in tracks.items() if a == target]
        if hits:
            found_targets = True
            print(f"\n  ✓ {target:#010x}  {desc}")
            for a, b, t in sorted(hits, key=lambda x: x[1]):
                print(row(a, b, t))
        else:
            print(f"\n  ✗ {target:#010x}  {desc}")
            print("      관측되지 않음")

    print("\n" + "=" * 78)
    print("패밀리")
    print("=" * 78)
    for mask, value, label in FAMILY_MASKS:
        hits = sorted(((a, b, t) for (a, b), t in tracks.items() if (a & mask) == value),
                      key=lambda x: (x[0], x[1]))
        print(f"\n  [{label}] {len(hits)}건")
        for a, b, t in hits:
            mark = " <-- 타겟" if a in TARGETS else ""
            print(row(a, b, t) + mark)
        if not hits:
            print("      없음")

    others = sorted(((a, b, t) for (a, b), t in tracks.items()
                     if family_of(a) is None), key=lambda x: (x[0], x[1]))
    print("\n" + "=" * 78)
    print(f"그 외 확장 ID  ({len(others)}건)")
    print("=" * 78)
    if not others:
        print("  없음")
    elif dump_all:
        for a, b, t in others:
            print(row(a, b, t))
    else:
        for a, b, t in others[:20]:
            print(row(a, b, t))
        if len(others) > 20:
            print(f"  ... +{len(others) - 20}건 (--all 로 전체 출력)")

    if not found_targets:
        print("\n" + "!" * 78)
        print("타겟 ID 가 하나도 잡히지 않았다.")
        print("!" * 78)
        if not tracks:
            print("  확장 ID 자체가 전혀 안 보인다. 아래를 의심할 것:")
            print("   · 차량이 잠들어 CAN 이 조용한 상태 (시동/충전 중에 다시 시도)")
            print("   · 게이트웨이 하네스가 해당 버스를 넘겨주지 않음")
        else:
            print(f"  확장 ID {len(tracks)}종은 보이므로 수신 경로 자체는 살아 있다.")
            print("  타겟이 다른 버스(컴포트 CAN)에만 존재할 가능성이 크다.")
            print("  위 '그 외 확장 ID' 목록에서 근접한 ID 가 있는지 확인할 것.")


def main() -> int:
    parser = argparse.ArgumentParser(description="ID.4 공조 제어 프레임 정찰 (수신 전용)")
    parser.add_argument("--duration", type=float, default=60.0, help="스캔 시간(초), 기본 60")
    parser.add_argument("--all", action="store_true", help="확장 ID 전체 덤프")
    parser.add_argument("--watch", type=lambda s: int(s, 0), default=None,
                        help="특정 ID 를 실시간으로 계속 출력 (예: 0x16A954FB)")
    args = parser.parse_args()

    print("=" * 78)
    print("ID.4 공조 프레임 정찰 — 수신 전용 (TX 없음)")
    print("=" * 78)
    if args.watch is not None:
        print(f"감시 대상: {args.watch:#010x}   (수동으로 공조를 조작하며 대조할 것)")
    print(f"스캔 시간: {args.duration:.0f}초\n")

    try:
        tracks = scan(args.duration, args.watch)
    except KeyboardInterrupt:
        print("\n중단됨")
        return 130

    report(tracks, args.all)
    print("\n끝 — 어떤 프레임도 송신하지 않았다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
