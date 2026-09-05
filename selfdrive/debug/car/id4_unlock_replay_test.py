#!/usr/bin/env python3
"""ID.4(MEB) 원격 언락 재현 — 1-b단계: 언락 프레임 재생 검증 (수동, 위험).

정찰(id4_unlock_scan.py)에서 확인한 Blinkmodi_02(0x366)의 언락 페이로드를
그대로 재생해서, 실제로 차량 언락(→ 독립 공조 가동)이 일어나는지 확인한다.
여기서 공조가 돌면 프로젝트 성공이 확정된다.

*** 매우 위험. 반드시 --confirm 이 있어야만 실제 송신한다. ***
없으면 dry-run(무엇을 쏠지 출력만).

=== 왜 이렇게 해야 하나 (판다/safety 제약) ===
Blinkmodi_02(0x366)는 판다 safety 의 MEB TX 화이트리스트에 없다
(safety_volkswagen_meb.h 확인). 따라서 정식 safety 모드에서는 판다가 이
프레임을 '차단'한다. 검증 단계에서는 판다를 allOutput(unsafe) 모드로 두고
쏘는 수밖에 없다. allOutput 은 모든 TX 를 허용하는 무방비 모드다.

그리고 allOutput 으로 바꾸려면 판다를 이 스크립트가 직접 잡아야 하는데,
openpilot 의 pandad 가 이미 USB 로 판다를 점유하고 있으면 열 수 없다.
그래서 실행 전에 pandad(openpilot)를 멈춰야 한다. openpilot 이 멈추면
cereal can 스트림도 끊기므로, 송신 후 상태 폴링은 Panda.can_recv() 로
직접 읽는다.

정식 배포 단계(다음 단계)에서는 allOutput 대신 safety_volkswagen_meb.h 의
TX 화이트리스트에 {MSG_BLINKMODI_02, <bus>, 8} 을 추가하고 재빌드해야 한다.
이번 단계는 그 패치가 값어치 있는지부터 확인하는 것이다.

=== 실패할 수 있는 이유 (미리 명시) ===
- 인증/immobilizer: 도어 컨트롤러가 스마트키의 롤링코드 인증을 요구하면,
  게이트웨이발 브로드캐스트를 흉내 내도 언락이 거부될 수 있다.
- 상태 신호였던 경우: BM_ZV_auf 가 '언락됨'을 알리는 결과 신호라면,
  재생해도 도어 컨트롤러는 무반응이다. (정찰의 (A)/(B) 판별 참고)
- sender 충돌: 이 프레임의 원 송신자는 게이트웨이다. 게이트웨이가 같은
  주기로 계속 쏘는 와중에 우리가 끼어들면 무시되거나 충돌할 수 있다.
이 셋 중 어디서 막혔는지 로그로 구분되게 했다.

사용:
  # 1) dry-run (아무것도 안 쏨, 무엇을 쏠지 확인)
  python3 id4_unlock_replay_test.py --bus 0

  # 2) 실제 재생 (openpilot 중지 후)
  sudo systemctl stop comma      # 또는 pkill -f manager
  python3 id4_unlock_replay_test.py --bus 0 --payload 0010004000000000 --confirm

  옵션:
    --bus N         재생할 버스 (정찰에서 확인한 값, 필수 판단)
    --payload HEX   정찰에서 캡처한 8바이트 언락 페이로드 (hex 16자리)
                    생략하면 CANPacker 로 BM_ZV_auf=1 프레임을 합성
    --count N       송신 횟수 (기본 5)
    --interval S    송신 간격 초 (기본 0.1)
    --confirm       실제 송신 (없으면 dry-run)
"""
import argparse
import sys
from pathlib import Path
import time
from subprocess import check_output, CalledProcessError

# Resolve the repo root from this file, so the tool works wherever the fork is installed
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

BLINKMODI_02 = 0x366
KLIMA_11 = 0x3B5          # 949, KL_AC_Schalter = byte0 bit2 (공조 반영 확인용)
AC_BYTE, AC_BIT = 0, 2


def pandad_running() -> bool:
    try:
        check_output(["pidof", "pandad"])
        return True
    except CalledProcessError as e:
        if e.returncode == 1:   # 1 = 프로세스 없음
            return False
        raise


def build_payload(args) -> bytes:
    """재생할 8바이트를 만든다. --payload 우선, 없으면 CANPacker 합성."""
    if args.payload:
        raw = bytes.fromhex(args.payload)
        if len(raw) != 8:
            raise ValueError(f"--payload 는 8바이트(hex 16자리)여야 한다: {len(raw)}바이트 받음")
        return raw

    # CANPacker 로 BM_ZV_auf=1 만 세운 프레임 합성.
    # Blinkmodi_02 는 카운터/체크섬이 없어 이 방식이 유효하다.
    from opendbc.can import CANPacker
    packer = CANPacker("vw_meb")
    addr, dat, _ = packer.make_can_msg("Blinkmodi_02", args.bus, {"BM_ZV_auf": 1})
    if not dat:
        raise RuntimeError("CANPacker 가 Blinkmodi_02 를 만들지 못했다")
    return bytes(dat)


def poll_ac(panda, seconds: float) -> None:
    """송신 후 Klima_11(KL_AC_Schalter)을 raw 로 몇 초간 읽어 공조 반영 확인.
    openpilot 이 멈춘 상태라 cereal 대신 Panda.can_recv() 로 직접 읽는다."""
    print(f"\n  공조 상태 {seconds:.0f}초 폴링 (Klima_11 0x{KLIMA_11:03X}) ...")
    deadline = time.monotonic() + seconds
    last_ac = None
    seen = 0
    while time.monotonic() < deadline:
        try:
            msgs = panda.can_recv()
        except Exception as exc:
            print(f"    can_recv 실패: {exc}")
            break
        for address, data, bus in msgs:
            if int(address) != KLIMA_11:
                continue
            seen += 1
            data = bytes(data)
            ac = (data[AC_BYTE] >> AC_BIT) & 1 if len(data) > AC_BYTE else -1
            if ac != last_ac:
                rel = seconds - (deadline - time.monotonic())
                print(f"    [{rel:4.1f}s] bus{bus} KL_AC_Schalter = {ac}  ({data.hex(' ').upper()})")
                last_ac = ac
        time.sleep(0.02)
    if seen == 0:
        print("    Klima_11 프레임을 못 받았다 (버스에 안 올라오거나 차가 잠든 상태).")
    elif last_ac == 1:
        print("    => 공조 ON 감지. 언락 재생이 먹혔을 가능성이 크다.")
    else:
        print("    => 공조 ON 신호를 못 봤다. (인증 거부/상태신호/sender충돌 중 하나)")


def main() -> int:
    p = argparse.ArgumentParser(description="ID.4 언락 프레임 재생 검증 (위험, --confirm 필요)")
    p.add_argument("--bus", type=int, required=True, help="재생할 버스 (정찰에서 확인)")
    p.add_argument("--payload", type=str, default=None, help="캡처한 8바이트 언락 페이로드 hex")
    p.add_argument("--count", type=int, default=5, help="송신 횟수 (기본 5)")
    p.add_argument("--interval", type=float, default=0.1, help="송신 간격 초 (기본 0.1)")
    p.add_argument("--poll", type=float, default=8.0, help="송신 후 공조 폴링 시간 초")
    p.add_argument("--confirm", action="store_true", help="실제 송신 (없으면 dry-run)")
    args = p.parse_args()

    print("=" * 74)
    print("ID.4 언락 프레임 재생 검증")
    print("=" * 74)

    try:
        payload = build_payload(args)
    except Exception as exc:
        print(f"[중단] 페이로드 준비 실패: {exc}")
        return 1

    src = "캡처본(--payload)" if args.payload else "CANPacker 합성(BM_ZV_auf=1)"
    print(f"  대상    : Blinkmodi_02 (0x{BLINKMODI_02:03X})  bus{args.bus}")
    print(f"  페이로드: {payload.hex(' ').upper()}   [{src}]")
    print(f"  송신    : {args.count}회 × {args.interval}s 간격")

    if not args.confirm:
        print("\n[DRY-RUN] --confirm 이 없어 실제로 쏘지 않았다.")
        print("  실제 재생하려면 openpilot 을 멈추고 --confirm 을 붙일 것:")
        print("    sudo systemctl stop comma")
        print(f"    python3 {sys.argv[0].split('/')[-1]} --bus {args.bus} "
              f"--payload {payload.hex()} --confirm")
        return 0

    # --- 여기부터 실제 송신 경로 ---
    if pandad_running():
        print("\n[중단] pandad(openpilot)가 실행 중이다. 판다를 열 수 없다.")
        print("  먼저 openpilot 을 멈출 것:  sudo systemctl stop comma")
        return 1

    try:
        from panda import Panda
        from opendbc.car.structs import CarParams
    except Exception as exc:
        print(f"[중단] panda 임포트 실패: {exc}")
        return 1

    print("\n  판다 여는 중 ...")
    try:
        panda = Panda()
    except Exception as exc:
        print(f"[중단] 판다 열기 실패: {exc}")
        return 1

    # allOutput = 모든 TX 허용(무방비). 화이트리스트에 없는 0x366 을 쏘기 위함.
    print("  safety = allOutput (unsafe) 로 설정 ...")
    panda.set_safety_mode(CarParams.SafetyModel.allOutput)
    time.sleep(0.2)

    print(f"\n  ★ 송신 시작 — Blinkmodi_02 bus{args.bus} × {args.count}")
    try:
        for i in range(args.count):
            panda.can_send(BLINKMODI_02, payload, args.bus)
            print(f"    [{i+1}/{args.count}] 송신: {payload.hex(' ').upper()}", flush=True)
            time.sleep(args.interval)
    except Exception as exc:
        print(f"  송신 중 오류: {exc}")

    poll_ac(panda, args.poll)

    # 판다를 안전 모드로 되돌린다. openpilot 재시작 시 어차피 다시 잡지만 명시적으로.
    try:
        panda.set_safety_mode(CarParams.SafetyModel.silent)
        print("\n  safety = silent 로 복귀.")
    except Exception:
        pass

    print("\n끝. openpilot 을 다시 켜려면:  sudo systemctl start comma")
    return 0


if __name__ == "__main__":
    sys.exit(main())
