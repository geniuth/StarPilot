# ID.4 원격 예열/예냉 — 설계

## 목표

앱에서 버튼 하나로 주차 중인 ID.4의 공조를 미리 돌린다(예열/예냉).

차량에 **"잠금 해제 시 독립 공조 작동"** 옵션이 켜져 있어, 스마트키 언락 시
공조가 도는 것을 실차 확인했다. 따라서 문제는 **"원격으로 언락 신호를 재현"**
하나로 좁혀졌다. 공조 프레임 직접 주입(`0x16A954FB`)은 컴포트 CAN 전용이라
콤마에 안 보여서 실패 확정. 반면 언락 신호(`Blinkmodi_02.BM_ZV_auf`)는
**bus1에서 콤마에 수신됨**(정찰로 확인). 재생 경로가 물리적으로 열려 있다.

## 전체 흐름

```
[앱 예열 버튼]
   │  POST /api/command {type:"preheat"}   (Bearer: LIVE_TOKEN)
   ▼
[Worker]  KV에 명령 저장 (deviceId별 1건)
   │
   │  기기가 오프로드일 때 짧은 주기로 폴링
   ▼  GET /api/command  (Bearer: UPLOAD_TOKEN)
[기기: wayon_preheat.py]
   │  명령 수신 → 언락 프레임 재생
   │  messaging.pub_sock("sendcan") 로 Blinkmodi_02(BM_ZV_auf=1) publish
   ▼
[pandad]  safety hook 통과(화이트리스트에 있어야) → 판다 → CAN bus1
   │
   ▼  언락 → 차량 독립 공조 가동
   │
   │  기기가 Klima_11(KL_AC_Schalter) 로 공조 ON 확인
   ▼  POST /api/command/ack {id, result:"ac_on"}
[Worker]  결과 저장
   ▲
   │  GET /api/command/status  (앱이 폴링)
[앱]  버튼에 "예열 시작됨 / 실패" 피드백
```

## 컴포넌트별 설계

### 1. 앱 (MyID4)
- 제어 탭(첫 화면)에 **예열 버튼**. 누르면 확인 다이얼로그(오조작 방지).
- `@JavascriptInterface fun requestPreheat()` → `POST {cloudBaseUrl}/api/command`
  `{"deviceId": <DongleId>, "type": "preheat"}`, `Authorization: Bearer <LIVE_TOKEN>`.
  `requestWayonLiveSession()` 과 동일한 인증·에러 처리 패턴 재사용.
- 전송 후 `/api/command/status` 를 몇 초 폴링해 결과를 버튼에 표시.
- **버튼은 지금 만든다.** Worker 엔드포인트가 아직 없으면 명확한 오류를 띄운다.

### 2. Worker (wayon-cloud)
새 엔드포인트 3개. 저장은 기존 KV(`env.SNAPSHOTS`) 재사용, 키 `cmd:<deviceId>`.
- `POST /api/command` (앱, LIVE_TOKEN): 명령을 KV에 저장. TTL 5분(오래된 명령 자동 폐기).
- `GET  /api/command?deviceId=` (기기, UPLOAD_TOKEN): 대기 명령 반환. 반환 시 상태를 `taken` 으로.
- `POST /api/command/ack` (기기, UPLOAD_TOKEN): 실행 결과 저장.
- `GET  /api/command/status?deviceId=` (앱, LIVE_TOKEN): 최근 명령 상태 조회.
- **멀티테넌트 주의**: 반드시 deviceId 로 격리. 남의 차에 예열 명령이 가면 안 된다.

### 3. 기기 (wayon_preheat.py, 신규 프로세스)
- config.json 있을 때만 기동(`wayon_remote_ready`). 오프로드일 때만 폴링(주행 중 금지).
- 15초 주기로 `GET /api/command`. `preheat` 명령이 오면 언락 재생.
- **CAN 송신은 `messaging.pub_sock("sendcan")` 정식 경로.**
  `CANPacker("vw_meb").make_can_msg("Blinkmodi_02", 1, {"BM_ZV_auf": 1})` 를
  몇 프레임 publish → pandad 가 safety 통과 후 판다로 보낸다.
  allOutput(검증용)이 아니라 이게 상시 경로다. **safety 패치가 전제.**
- 재생 후 Klima_11(KL_AC_Schalter)을 몇 초 읽어 공조 ON 확인 → ack.
- 안전장치: 오프로드 아니면 즉시 중단. 재생은 짧게(예: 3프레임 × 0.1s).

### 4. safety 패치 (safety_volkswagen_meb.h)
- TX 화이트리스트에 `{MSG_BLINKMODI_02, 1, 8}` 추가(bus1, 8바이트).
- **위험**: 이 프레임은 controls_allowed 와 무관하게 항상 TX 허용된다.
  주행 안전에 영향은 없다(언락 프레임은 차량 제어와 무관). 하지만 판다
  펌웨어 재빌드+플래시가 필요하고, 잘못되면 판다 벽돌 위험.
- **그래서 이번엔 코드로만 작성하고 적용(플래시)은 재생 검증 성공 후.**
  패치 없으면 pandad 가 Blinkmodi_02 를 차단 → "차단됨" 로그로 확인 가능.

## 활성화 순서 (재생 검증 후)

1. **재생 검증(1-b)** 성공 확인 — allOutput 으로 언락→공조 가동 실증.
2. **safety 패치 적용** — 판다 펌웨어 재빌드+플래시.
3. **Worker 배포** — 명령 큐 엔드포인트.
4. **wayon_preheat 등록** — process_config 에 프로세스 추가.
5. 앱 버튼은 이미 있으므로 그대로 동작.

각 단계는 독립적으로 revert 가능하다. 앱 버튼/Worker/기기 프로세스는 가볍게
되돌릴 수 있고, safety 패치만 판다 관련이라 신중.

## 실패 시 어디서 막히나 (진단 매트릭스)

| 증상 | 원인 |
|---|---|
| 버튼이 "전송 실패" | Worker 엔드포인트 미배포, 또는 토큰/주소 오설정 |
| 명령은 갔는데 기기 무반응 | wayon_preheat 미등록, 또는 차가 자는 중 |
| 기기 로그 "sendcan 차단" | safety 패치 미적용 |
| 재생은 됐는데 공조 안 됨 | 인증 거부 / 상태신호였음 / sender 충돌 (1-b 정찰 참고) |

## 미확정 (재생 검증에서 답 나올 것)

- `BM_ZV_auf` 가 이벤트성(재생 가능)인지 상태신호(무반응)인지 — 1-a 정찰
- 게이트웨이가 원 송신자라 sender 충돌 가능성 — 1-b 재생
- 재생 프레임 수/주기 (1회? 반복?) — 1-b 튜닝
