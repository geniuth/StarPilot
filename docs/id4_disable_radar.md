# DISABLE_RADAR — 카메라 하네스 롱컨 (미검증)

카메라 하네스로도 openpilot 롱컨(가감속)을 가능하게 하는 기능. 순정 레이더를
프로그래밍(플래시) 세션에 가둬 송신을 멈추고, openpilot이 레이더 대체 메시지를
직접 보내 EPS/ACC ECU가 폴트나지 않게 한다. infiniteCable2 실코드 이식.

**⚠️ 이 기능은 실차 검증 안 됨.** 카메라 하네스 + ID.4 테스터가 필요하다.
켜면 **순정 AEB / FCW / 긴급조향(EA)을 잃는다.** 문제 시 복구는 **시동 재점화**.

## 켜는 법

- 카메라 하네스로 설치된 ID.4에서, 설정 → TEST 탭 → "카메라하네스 롱컨/레이더무력화" = 1
- 재부팅 후 카메라 하네스에서 롱컨(가감속)이 가능해짐
- 게이트웨이 하네스에서는 이 옵션이 필요 없음 (순정 레이더가 살아있어 롱컨이 이미 됨).
  게이트웨이에서는 파라미터를 켜도 DISABLE_RADAR 플래그가 붙지 않음

## 동작 원리

부팅 시(`interface.init`, card.py가 controls ready 후 호출):
1. `_is_engine_state_allowed_meb` — Motor_54(0x14C)의 Engine_On으로 프로그래밍 세션
   요청 가능 여부 판정. 엔진 On이면 세션이 거부되므로 시도하지 않고 실패 표시.
2. `_radar_communication_control` — 레이더(0x757)에 Tester Present → 확장 진단 세션
   → 프로그래밍 세션 요청. 성공하면 레이더가 송신을 멈춤. 실패 시 RADAR_DISABLE_STATE.

주행 중(`carcontroller`, DISABLE_RADAR + long + not radarDisableFailed):
- 0x700로 Tester Present (1Hz) — 프로그래밍 세션 유지
- AWV_03 대체본 (1Hz) — 비활성 AEB 제어
- MEB_AWV_01 대체본 (5Hz) — AEB HUD (비활성 아이콘)
- Strukturen_01 빈 오브젝트 (25Hz) — 레이더 오브젝트 자리 채움
- 부팅 후 몇 초간 AEB 비활성 경고 HUD 표시 후 숨김

## 구현 파일

- `values.py`: `VolkswagenFlags.DISABLE_RADAR`, `VolkswagenSafetyFlags.DISABLE_RADAR`,
  `RADAR_DISABLE_STATE`
- `interface.py`: 파라미터→플래그(카메라 하네스만), 롱컨 게이트, `init`/`_radar_communication_control`/`_is_engine_state_allowed_meb`
- `carstate.py`: `radarDisableFailed`
- `carcontroller.py`: 레이더 대체 메시지 tx 블록
- `mebcan.py`: `create_aeb_control`/`create_aeb_hud`/`create_radar_objects`
- `vw_meb.dbc` / `vw_meb_2024.dbc`: AWV_03 전체 시그널 + MEB_AWV_01
- `safety_volkswagen_meb.h`: DISABLE_RADAR tx 허용목록(AWV_03/MEB_AWV_01/Strukturen_01/0x700)
- `car.capnp`: `radarDisableFailed @85`

## 검증 상태 (이 환경에서 가능한 것만)

- ✅ opendbc 파이썬 전체 컴파일
- ✅ 판다 safety C 컴파일 (`-Wall -Werror`)
- ✅ 게이트: 카메라 하네스+파라미터 on → DISABLE_RADAR 플래그 + 롱컨 + safetyParam(LONG|DISABLE_RADAR=5)
- ✅ 게이트: 파라미터 off → 플래그 없음, 롱컨 없음
- ✅ 게이트: 게이트웨이 하네스 → DISABLE_RADAR 안 붙고 기존 롱컨 유지
- ✅ AEB 대체 메시지 패킹 왕복 (AWV_03 0xDB/48B, MEB_AWV_01 0x16A954AD/8B, Strukturen_01 0x24F/64B)
- ✅ 엔진 상태 판정 (on→거부, off→허용)
- ✅ MK1/MK2 rlog 리플레이 파서 회귀 없음 (DBC 변경 후 canValid 100%)
- ❌ **실차 동작 — 불가 (차·카메라 하네스·레이더 없음).** 레이더가 실제로 세션에
  갇히는지, 대체 메시지가 ECU에 수락되는지, 크루즈 폴트 없이 도는지는 테스터 필요.

## if2 대비 충실도 (재검증)

- UDS 무력화 루틴(0x757 프로그래밍 세션, 0x700 TP, 재시도 3/타임아웃 0.5, 무대기
  프로그래밍 세션): if2와 동일
- AEB 대체 메시지 값(AWV_03/MEB_AWV_01/Strukturen_01): if2와 동일 (MEB에서 안 타는
  MQB_EVO 분기만 생략)
- carcontroller tx 타이밍(AEB_CONTROL_STEP/AEB_HUD_STEP/frame%4), 엔진 비트
  오프셋(0x14C dat[9]>>5): if2와 동일
- **무력화 실패 시 안전 처리**: if2와 동일하게 (1) ACC 가감속/HUD tx도 중단해
  살아있는 순정 레이더와 충돌 방지, (2) `radarDisableFailed` 이벤트로 인게이지
  차단(NO_ENTRY)·즉시 해제(IMMEDIATE_DISABLE)·"시동 재점화" 안내
- 차이: if2의 `pre_init` 대시캠 강등은 carrot에 pre_init 훅이 없어 미이식. 대신
  위 (1)(2)가 동일한 안전 속성(무력화 실패 시 openpilot이 종제어 안 함)을 보장

## 테스터에게

- 반드시 카메라 하네스 + 이 브랜치(test)에서만.
- 첫 시험은 안전한 곳에서. 켠 직후 계기판에 AEB 비활성 경고가 몇 초 뜨는 게 정상.
- 크루즈가 폴트나거나 계기판에 레이더/ACC 오류가 뜨면 즉시 파라미터 0으로 되돌리고
  **시동을 껐다 켜서** 레이더를 소생시킬 것.
- 로그(rlog)를 남겨주면 대체 메시지 수락 여부/폴트 원인을 분석할 수 있음.
