#!/usr/bin/env python3
"""ID.4(MEB) 공조 제어 프레임 정찰 — 저장된 rlog 분석용.

id4_klima_scan.py 와 같은 분석을 이미 녹화된 주행 로그에서 한다.
차를 만지지 않고도 타겟 ID 가 존재하는지 먼저 답을 얻을 수 있어서,
라이브 스캔보다 이쪽을 먼저 돌려보는 게 빠르다.

*** 로그를 읽기만 한다. 송신(TX)은 없다. ***

사용:
  python3 id4_klima_replay.py <route>              # 세그먼트 0~80
  python3 id4_klima_replay.py <route> 20           # 0~20
  python3 id4_klima_replay.py <route> 10 30        # 10~30
  python3 id4_klima_replay.py <route> 0 80 --all   # 확장 ID 전체 덤프
"""
import argparse
import glob
import sys
from pathlib import Path
from collections import defaultdict

# Resolve the repo root from this file, so the tool works wherever the fork is installed
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from openpilot.tools.lib.logreader import LogReader

REALDATA = "/data/media/0/realdata"

# id4_klima_scan.py 와 동일한 타겟/패밀리 정의를 쓴다.
TARGETS = {
    0x16A954FB: "공조 온도/제어 명령 (30단계: LO, 16.0~29.5, HI)",
    0x16A95493: "공조 가동 상태 (byte0: 0x01=ON, 0x00=OFF)",
    0x1B000010: "CAN 슬립 모드 제어 (버스 웨이크용)",
}
FAMILY_MASKS = (
    (0xFFFFFF00, 0x16A95400, "0x16A954xx"),
    (0xFFFFFF00, 0x1B000000, "0x1B0000xx"),
)
EXT_ID_MIN = 0x800


class Track:
    """(address, bus) 하나의 관측 이력. 시간은 로그의 monotime[s] 을 쓴다."""

    __slots__ = ("first_payload", "last_payload", "length", "count",
                 "first_t", "last_t", "changed_bytes", "segments")

    def __init__(self, payload: bytes, t: float, seg: int):
        self.first_payload = payload
        self.last_payload = payload
        self.length = len(payload)
        self.count = 1
        self.first_t = t
        self.last_t = t
        self.changed_bytes: set[int] = set()
        self.segments: set[int] = {seg}

    def add(self, payload: bytes, t: float, seg: int) -> None:
        self.count += 1
        self.last_t = t
        self.segments.add(seg)
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


def row(address: int, bus: int, track: Track) -> str:
    changed = ""
    if track.changed_bytes:
        changed = "byte " + ",".join(str(i) for i in sorted(track.changed_bytes))
    segs = sorted(track.segments)
    seg_text = f"seg {segs[0]}" if len(segs) == 1 else f"seg {segs[0]}~{segs[-1]}({len(segs)}개)"
    return (f"  {address:#010x}  bus{bus}  len={track.length}  "
            f"{track.count:7d}회  {track.hz:6.1f}Hz  "
            f"{hexs(track.first_payload):24}  {seg_text}  {changed}")


def scan_route(route: str, seg_from: int, seg_to: int) -> tuple[dict, int, int]:
    """rlog 를 순회하며 확장 ID 를 모은다. 반환: (tracks, 처리세그수, 총프레임)"""
    tracks: dict[tuple[int, int], Track] = {}
    done_segments = 0
    total_frames = 0

    for seg in range(seg_from, seg_to + 1):
        files = glob.glob(f"{REALDATA}/{route}--{seg}/rlog*")
        if not files:
            continue
        try:
            for m in LogReader(files[0]):
                if m.which() != "can":
                    continue
                t = m.logMonoTime / 1e9
                for f in m.can:
                    address = int(f.address)
                    total_frames += 1
                    if address < EXT_ID_MIN:
                        continue
                    bus = int(f.src)
                    payload = bytes(f.dat)
                    key = (address, bus)
                    track = tracks.get(key)
                    if track is None:
                        tracks[key] = Track(payload, t, seg)
                    else:
                        track.add(payload, t, seg)
        except Exception as exc:
            print(f"  seg {seg}: 읽기 실패 ({exc})")
            continue
        done_segments += 1
        print(f"  seg {seg:3d}  누적 확장 ID {len(tracks)}종", flush=True)

    return tracks, done_segments, total_frames


def report(tracks: dict, dump_all: bool) -> bool:
    print("\n" + "=" * 78)
    print("타겟 ID")
    print("=" * 78)
    found = False
    for target, desc in TARGETS.items():
        hits = [(a, b, t) for (a, b), t in tracks.items() if a == target]
        if hits:
            found = True
            print(f"\n  ✓ {target:#010x}  {desc}")
            for a, b, t in sorted(hits, key=lambda x: x[1]):
                print(row(a, b, t))
        else:
            print(f"\n  ✗ {target:#010x}  {desc}")
            print("      로그에 없음")

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
    return found


def main() -> int:
    parser = argparse.ArgumentParser(description="ID.4 공조 프레임 정찰 (rlog 분석)")
    parser.add_argument("route", help="루트 이름 (예: a1b2c3d4e5f6|2026-08-09--06-00-00)")
    parser.add_argument("seg_from", nargs="?", type=int, default=0, help="시작 세그먼트")
    parser.add_argument("seg_to", nargs="?", type=int, default=None, help="끝 세그먼트")
    parser.add_argument("--all", action="store_true", help="확장 ID 전체 덤프")
    args = parser.parse_args()

    # 인자 1개면 그게 끝 세그먼트(기존 id4_climate_find.py 와 같은 감각)
    if args.seg_to is None:
        seg_from, seg_to = (0, args.seg_from) if args.seg_from else (0, 80)
    else:
        seg_from, seg_to = args.seg_from, args.seg_to

    print("=" * 78)
    print("ID.4 공조 프레임 정찰 — rlog 분석 (TX 없음)")
    print("=" * 78)
    print(f"루트    : {args.route}")
    print(f"세그먼트: {seg_from} ~ {seg_to}\n")

    tracks, done, total_frames = scan_route(args.route, seg_from, seg_to)

    if done == 0:
        print(f"\n[중단] 해당 루트의 rlog 를 찾지 못했다: {REALDATA}/{args.route}--*")
        print("  사용 가능한 루트를 보려면:")
        print(f"    ls {REALDATA} | sed 's/--[0-9]*$//' | sort -u | tail")
        return 1

    print(f"\n세그먼트 {done}개, 총 {total_frames:,} 프레임, 확장 ID {len(tracks)}종")
    found = report(tracks, args.all)

    if not found:
        print("\n" + "!" * 78)
        print("타겟 ID 가 이 로그에 없다.")
        print("!" * 78)
        if not tracks:
            print("  확장 ID 자체가 없다. 이 로그에는 해당 버스가 안 담겼을 수 있다.")
        else:
            print(f"  확장 ID {len(tracks)}종은 있으므로 로그 자체는 정상이다.")
            print("  타겟이 컴포트 CAN 등 콤마가 보지 못하는 버스에만 있을 가능성이 크다.")
            print("  주차 중 공조를 수동 조작한 구간의 로그로 다시 확인해볼 것.")

    print("\n끝 — 로그를 읽기만 했다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
