#!/usr/bin/env python3
# ruff: noqa: ISC002
"""ID.4(MEB) 차량 상태를 Wayon Cloud로 올린다.

배터리 잔량·오도미터·외기온·12V 전압과 GPS를 주기적으로 POST해서 휴대폰 앱이
차 상태를 볼 수 있게 한다.

배터리는 CAN(Motor_16)에서 직접 디코딩한다. carState는 card 프로세스가 온로드에서만
돌아 주차 중엔 비어 있어, 충전 중 감시가 되려면 CAN 직접 샘플링이어야 한다.
HV 전압·배터리 전류·BMS 모드는 0x0CF(DBC에 없음, 원시 바이트 해석)에서 실측한다.
0x0CF가 없을 때만 충전 전력을 배터리량 증가 기울기로 추정한다.

- 주행 중: TELEMETRY_ONROAD_S 간격
- 주차 중: TELEMETRY_OFFROAD_S 간격 + 배터리 변화 감지 시 즉시
- /data/wayon_cloud/config.json 의 endpoint/token 이 있어야 동작 (없으면 조용히 대기)
"""
import json
import os
import time
from pathlib import Path

import requests

from cereal import messaging
from openpilot.common.params import Params
from datetime import UTC

CONFIG_PATH = Path(os.getenv("WAYON_CLOUD_CONFIG", "/data/wayon_cloud/config.json"))
STATE_PATH = Path(os.getenv("WAYON_VEHICLE_STATE", "/data/wayon_cloud/vehicle_state.json"))

# 주행 중엔 앱이 속도·위치를 실시간처럼 보여주므로 짧게 둔다. 앱은 주행 중 2초마다 가져간다.
# 3초 업로드는 속도·위치·도어만 싣는 가벼운 업로드다(CAN 을 따로 읽지 않음, 직전 값 재사용).
# 배터리·충전·외기온 등 CAN 전체 샘플링(1~2초 걸림)은 TELEMETRY_ONROAD_FULL_S 마다만 한다.
TELEMETRY_ONROAD_S = 3.0
TELEMETRY_ONROAD_FULL_S = 30.0
# 가벼운 업로드는 응답을 오래 기다리지 않는다(터널 등에서 루프가 수십 초 멈추지 않게)
LIGHT_POST_TIMEOUT = (3, 3)
TELEMETRY_OFFROAD_S = 600.0
# 주차 중 배터리량이 이만큼(Wh) 변하면 즉시 업로드 -> 충전 시작/종료를 빨리 반영
BATTERY_DELTA_TRIGGER_WH = 300.0
# 잔량% 계산 분모. 차량 계기판 표시와 맞추기 위해 64kWh로 고정한다.
# (BMS가 주는 실측 용량은 값이 계속 변해 계기판과 어긋난다)
USABLE_BATTERY_WH = 64000.0
# carstate 는 fuelGauge = Motor_16[Wh] / 77000 으로 넣는다(opendbc volkswagen/carstate.py).
# 주행 중 carState 에서 Wh 를 되살릴 때는 이 값으로 곱해야 CAN 직접 샘플과 맞는다.
# (예전엔 64000 을 곱해 주행 중 값이 17% 낮게 나왔고, 업로드 때 CAN 값과 늘 300Wh 넘게
#  어긋나 '배터리 급변'으로 판정돼 30초 주기 대신 매번 업로드했다)
CARSTATE_GAUGE_FULL_WH = 77000.0
# Motor_16 / HVEM_02 에너지량 무효값[Wh]. 차가 깨어나는 중이거나 BMS 가 다시 초기화될 때
# 이런 고정값이 섞여 들어온다(carrot-ha MEB_INVALID_ENERGY_WH). 잔량이 100% 넘게 튀고
# 충전 원장에 수십 kWh 가 한 번에 더해지므로 버린다. 실제 용량(~77kWh)을 넘는 값도 버린다.
MEB_INVALID_ENERGY_WH = {102250, 102300, 102350, 102375}
MAX_VALID_ENERGY_WH = 90000.0
# 충전으로 판정할 최소 증가율[W]
CHARGING_MIN_W = 300.0
# 오도미터(KBI_Kilometerstand)는 DBC상 20비트 [0|1048573] 이다. 1048574(2^20-2)는
# 규격상 Init/오류 표시값인데, 원격 공조 등으로 차가 깨어날 때 계기판 ECU가 아직
# 값을 안 실어서 이게 그대로 올라오곤 했다. 유효범위 밖이면 버린다.
ODO_MAX_VALID_KM = 1048573.0
# 이전 값 대비 이만큼 이상 튀면 계기판이 준 값이라도 믿지 않는다.
ODO_MAX_JUMP_KM = 1000.0
# 충전 요금[원/kWh] — 11kW 이하 완속, 초과 급속
CHARGE_SLOW_MAX_W = 11000.0
CHARGE_PRICE_SLOW = 280.0
CHARGE_PRICE_FAST = 320.0

# 0x0CF(207): BMS 실시간 전류/전압/모드, 100Hz. vw_meb.dbc 에 없어 원시 바이트로 해석한다.
#   모드 = b[2] & 0x07                                   (0,1 비충전 / 4 AC 충전 / 6 DC 충전)
#   전류 = ((((b[4] & 0x7F) << 8) | b[3]) - 16300) * 0.1 [A] (+ 충전/회생, - 방전)
#   전압 = ((b[7] << 4) | (b[6] >> 4)) * 0.25            [V]
# 실측 검증(2026-10): 주행 rlog 에서 V-I 상관 0.99, V×I 적산이 Motor_16 감소와 일치.
# AC 완속 충전 중 모드 4, +14.6A, 394.5V, +5.76kW. DC(6)는 미검증.
# MEB_HVEM_01.Battery_Voltage 는 주행 중에도 480.9V 고정이라 실제 팩 전압이 아니다(사용 중단).
BMS_0CF_ADDR = 0x0CF
BMS_CHARGE_MODES = {4: "ac", 6: "dc"}
BMS_IDLE_MODES = (0, 1)

# 도어 잠금/열림 (읽기 전용). 실측(2026-10): 주행 rlog 에서 시속 17km 자동 잠금 때
# ZV_02.ZV_verriegelt_intern_ist 와 문별 *_verriegelt 가 동시에 0->1. bus0/1/2 수신.
#   ZV_02 (0x583) byte2: bit0 안에서 잠김(intern_ist), bit1 밖에서 잠김(extern_ist), bit4 SAFE(gesafet_extern_ist)
#   Gateway_72 (0x3DB): 문 열림 비트 (DBC 시작비트 -> byte, bit)
ZV_02_ADDR = 1411
GATEWAY_72_ADDR = 987
DOOR_OPEN_BITS = {
  "driver": (3, 2),       # ZV_FT_offen   26
  "passenger": (3, 0),    # ZV_BT_offen   24
  "rear_left": (2, 4),    # ZV_HFS_offen  20
  "rear_right": (2, 5),   # ZV_HBFS_offen 21
  "tailgate": (3, 6),     # ZV_HD_offen   30
}
# 잠금 상태가 바뀌어 즉시 업로드할 때도 최소 이 간격은 둔다(신호가 잠깐 흔들려도 폭주하지 않게)
DOOR_UPLOAD_MIN_GAP_S = 10.0

# 전비/주행가능거리. 트립마다 출발·도착 배터리[Wh]와 GPS 거리로 km/kWh 를 기록한다.
# Motor_16 분해능이 25~50Wh 라 짧은 트립은 오차가 커서 거리·에너지가 작으면 버린다.
EFFICIENCY_PATH = Path(os.getenv("WAYON_EFFICIENCY", "/data/wayon_cloud/efficiency.json"))
EFFICIENCY_MIN_KM = 2.0
EFFICIENCY_MIN_KWH = 0.5
EFFICIENCY_KEEP_TRIPS = 30
# 주행가능거리는 최근 트립 전비의 하위 25%(보수적)로 잡는다. 트립이 적으면 평균, 없으면 기본값.
RANGE_QUARTILE_MIN_TRIPS = 4
DEFAULT_EFFICIENCY_KPL = 5.0
EFFICIENCY_VALID_KPL = (2.0, 12.0)   # 이 밖은 GPS 튐·배터리 값 오류로 본다

# 충전 완료 예상. 배터리 측 실측 전력(0x0CF)으로 목표 잔량까지 남은 시간을 낸다.
# DC 는 전력이 잔량에 따라 떨어지므로 ID.4 실측 급속 충전 곡선(carrot-ha, kW, 잔량 0~100%)을 쓴다.
ID4_DC_CURVE_KW = [
  181.3, 181.3, 183.4, 183.3, 185.0, 187.1, 187.2, 187.2, 188.3, 189.1, 189.2,
  190.2, 190.3, 190.2, 191.3, 191.3, 191.3, 191.3, 191.3, 192.3, 192.4,
  189.2, 187.1, 184.4, 181.2, 179.2, 176.2, 170.2, 165.0, 160.2, 155.0,
  150.2, 146.0, 144.0, 141.2, 138.2, 136.0, 132.9, 131.2, 129.1, 127.0,
  124.0, 122.9, 120.1, 119.1, 117.0, 114.0, 112.9, 110.1, 109.1, 107.0,
  107.0, 104.9, 103.8, 101.8, 101.9, 100.7, 100.1, 99.1, 98.0, 96.9,
  96.0, 95.0, 93.9, 92.8, 91.8, 91.8, 91.8, 91.8, 91.8, 91.8,
  91.8, 91.8, 91.9, 91.8, 92.9, 92.8, 92.8, 92.8, 91.8, 90.8,
  90.0, 88.9, 85.9, 81.8, 79.1, 76.0, 72.8, 69.7, 66.9, 62.8,
  59.7, 57.0, 52.7, 49.7, 46.8, 43.7, 40.6, 37.6, 34.8, 31.7,
]
CHARGE_ETA_TARGETS = (80, 100)

# can 구독 소켓 (재사용)
_CAN_SOCK = None
# CAN 해석기 [bus0, bus1] (재사용)
_CAN_PARSERS = None


def valid_battery_wh(value) -> float | None:
  """Motor_16/HVEM_02 에너지량이 믿을 만하면 Wh, 아니면 None."""
  try:
    wh = float(value)
  except (TypeError, ValueError):
    return None
  if not 0 < wh <= MAX_VALID_ENERGY_WH or round(wh) in MEB_INVALID_ENERGY_WH:
    return None
  return wh


def read_config() -> dict:
  try:
    with CONFIG_PATH.open("r", encoding="utf-8") as handle:
      config = json.load(handle)
      return config if isinstance(config, dict) else {}
  except (OSError, ValueError):
    return {}


def load_last_state() -> dict:
  try:
    return json.loads(STATE_PATH.read_text(encoding="utf-8"))
  except (OSError, ValueError):
    return {}


def save_last_state(payload: dict) -> None:
  STATE_PATH.parent.mkdir(parents=True, exist_ok=True)
  tmp = STATE_PATH.with_suffix(".tmp")
  tmp.write_text(json.dumps(payload, separators=(",", ":")), encoding="utf-8")
  os.replace(tmp, STATE_PATH)


def utc_now() -> str:
  from datetime import datetime
  return datetime.now(UTC).isoformat(timespec="seconds").replace("+00:00", "Z")


def build_vehicle_block(battery_wh: float | None, charge_power_w: float | None,
                        capacity_wh: float | None = None, bms_mode: int | None = None) -> dict:
  vehicle: dict = {}
  # 분모는 항상 고정값(계기판 일치). capacity_wh(BMS 실측)는 SOH 전용.
  usable = USABLE_BATTERY_WH
  if battery_wh is not None and battery_wh > 0:
    vehicle["battery_wh"] = round(battery_wh, 1)
    vehicle["capacity_wh"] = round(usable, 0)
    vehicle["soc_percent"] = round(min(100.0, battery_wh / usable * 100.0), 1)
  # 충전 여부는 BMS 모드가 있으면 그것을 따른다. 주행 중 회생도 전류가 + 라서
  # 전력만 보면 충전으로 오판하기 때문이다. 모드를 모를 때만 전력 기준으로 판정한다.
  if bms_mode in BMS_CHARGE_MODES:
    charging = True
    vehicle["charge_kind"] = BMS_CHARGE_MODES[bms_mode]
  elif bms_mode in BMS_IDLE_MODES:
    charging = False
  else:
    charging = charge_power_w is not None and charge_power_w >= CHARGING_MIN_W
  vehicle["charging"] = charging
  if charging and charge_power_w is not None and charge_power_w > 0:
    vehicle["charge_power_w"] = round(charge_power_w, 0)
  if bms_mode is not None:
    vehicle["bms_mode"] = bms_mode
  return vehicle


CHARGE_LEDGER_PATH = Path(os.getenv("WAYON_CHARGE_LEDGER", "/data/wayon_cloud/charge_ledger.json"))


def load_charge_ledger() -> dict:
  try:
    data = json.loads(CHARGE_LEDGER_PATH.read_text(encoding="utf-8"))
    if isinstance(data, dict) and isinstance(data.get("months"), dict):
      return data
  except (OSError, ValueError):
    pass
  return {"months": {}}


def save_charge_ledger(ledger: dict) -> None:
  CHARGE_LEDGER_PATH.parent.mkdir(parents=True, exist_ok=True)
  tmp = CHARGE_LEDGER_PATH.with_suffix(".tmp")
  tmp.write_text(json.dumps(ledger, separators=(",", ":")), encoding="utf-8")
  os.replace(tmp, CHARGE_LEDGER_PATH)


def add_charge_energy(ledger: dict, delta_wh: float, hours: float, kind_hint: str | None = None) -> None:
  """늘어난 에너지를 완속/급속으로 나눠 이번 달에 더한다.

  BMS 모드(0x0CF)로 AC/DC 를 알면 그것을 쓴다(kind_hint: 'ac'->완속, 'dc'->급속).
  모를 때만 구간 평균 전력(증가분 / 경과시간)으로 나눈다. 순간 전력을 쓰지 않는 건,
  기기가 자다 깨면 몇 시간치가 한 번에 들어와 판정이 엉뚱해지기 때문이다.
  """
  if delta_wh <= 0 or hours <= 0:
    return
  avg_power_w = delta_wh / hours
  # 상식 밖이면 계측 오류로 본다 (MEB 최대 급속이 ~175kW)
  if avg_power_w > 250000:
    return

  from datetime import datetime
  # 월 구분은 한국 시간 기준(KST=UTC+9). 월말 자정 근처 충전이 엉뚱한 달로 가지 않게.
  now = datetime.now(UTC).timestamp() + 9 * 3600
  month = datetime.utcfromtimestamp(now).strftime("%Y-%m")

  if kind_hint == "ac":
    kind = "slow"
  elif kind_hint == "dc":
    kind = "fast"
  else:
    kind = "slow" if avg_power_w <= CHARGE_SLOW_MAX_W else "fast"
  price = CHARGE_PRICE_SLOW if kind == "slow" else CHARGE_PRICE_FAST
  kwh = delta_wh / 1000.0

  entry = ledger["months"].setdefault(month, {"slow_kwh": 0.0, "fast_kwh": 0.0, "cost_krw": 0.0})
  entry[f"{kind}_kwh"] = round(entry.get(f"{kind}_kwh", 0.0) + kwh, 3)
  entry["cost_krw"] = round(entry.get("cost_krw", 0.0) + kwh * price, 0)

  # 최근 13개월만 남긴다 (페이로드가 무한정 커지지 않게)
  for old_month in sorted(ledger["months"])[:-13]:
    ledger["months"].pop(old_month, None)


def merge_last_known(vehicle: dict, last_known: dict) -> dict:
  """차가 꺼지면 CAN이 끊겨 값이 사라진다. 마지막 측정값을 채워
  주차 후에도 앱이 '주차 시점의 잔량/주행거리'를 보여주게 한다."""
  merged = dict(vehicle)
  fresh = False
  for key in ("battery_wh", "capacity_wh", "soc_percent", "odometer_km", "outside_temp_c",
              "hv_voltage", "measured_capacity_wh",
              "ac_on", "blower_volt", "blower_level", "seat_heat_left", "seat_heat_right", "recirc"):
    if merged.get(key) is None and last_known.get(key) is not None:
      merged[key] = last_known[key]
    elif merged.get(key) is not None and key in ("battery_wh", "odometer_km"):
      fresh = True
  if not fresh and last_known.get("measured_at"):
    # 값이 실시간이 아니라 '마지막 시동 시점' 측정치임을 앱에 알린다
    merged["measured_at"] = last_known["measured_at"]
    merged["stale"] = True
  return merged


def get_can_sock():
  """can 구독 소켓. 매번 새로 열면 첫 호출(프로세스 기동 직후)에 연결 워밍업 때문에
  아무것도 못 받으므로 한 번 만들어 재사용한다."""
  global _CAN_SOCK
  if _CAN_SOCK is None:
    _CAN_SOCK = messaging.sub_sock("can", timeout=200)
    time.sleep(0.5)          # 구독 성립 대기
    messaging.drain_sock(_CAN_SOCK)
  return _CAN_SOCK


def read_door_state(sock) -> dict | None:
  """쌓인 CAN 에서 도어 잠금/열림만 가볍게 읽는다(수신 전용). 버스가 조용하면 None.

  메인 루프가 매초 부른다. 주차 중 업로드는 10분마다라, 잠그는 순간을 놓치지 않으려면
  잠금 신호만큼은 계속 지켜보다가 바뀌면 바로 올려야 한다(차는 잠근 뒤 곧 잠들어 조용해짐).
  """
  zv = gw = None
  for m in messaging.drain_sock(sock):
    for f in m.can:
      if f.src > 2 or len(f.dat) < 4:
        continue
      if f.address == ZV_02_ADDR:
        zv = bytes(f.dat)
      elif f.address == GATEWAY_72_ADDR:
        gw = bytes(f.dat)
  if zv is None and gw is None:
    return None
  state: dict = {}
  if zv is not None:
    state["door_locked_inside"] = bool(zv[2] & 0x01)
    state["door_locked_outside"] = bool(zv[2] & 0x02)
    state["door_safe"] = bool(zv[2] & 0x10)
  if gw is not None:
    state["doors_open"] = [name for name, (byte, bit) in DOOR_OPEN_BITS.items() if (gw[byte] >> bit) & 1]
  return state


def sample_vehicle_can(timeout_s: float = 6.0) -> dict:
  """차량 값들을 CAN에서 짧게 샘플링한다.

  carState는 card 프로세스가 온로드에서만 돌아 주차 중엔 비어 있다.
  그래서 배터리까지 여기서 직접 디코딩해야 충전 중 모니터링이 된다 (주행 코드 무수정).

  주차/충전 중 실측(2026-08): 아래가 모두 bus1로 수신된다. 주행 중엔 bus0로 오므로
  양쪽 버스를 다 파싱한다.
  - Motor_16.MO_Energieinhalt_BMS      : HV 배터리 에너지량[Wh]  <- 잔량/충전 판정의 핵심
  - HVEM_02.HVEM_Nutzbare_Energie      : 가용 에너지[Wh] (Motor_16과 동일값으로 교차검증됨)
  - 0x0CF (원시 해석, BMS_0CF_ADDR 주석) : HV 전압[V]·배터리 전류[A]·BMS 모드
  - BMS_04.BMS_Kapazitaet_02           : 배터리 용량[Ah] -> 실제 용량/SOH
  - Diagnose_01.KBI_Kilometerstand     : 계기판 총 주행거리[km]
  - Klima_Sensor_02.BCM1_Aussen_Temp_ungef : 외기온도[degC] (주행 중에만 수신)
  """
  result: dict = {}
  # 같은 메시지가 주행 중엔 bus0, 주차/충전 중엔 bus1로 온다(실측). 양쪽 다 파싱한다.
  MSGS = [("Motor_16", 2), ("HVEM_02", 10),
          ("BMS_04", 2), ("Diagnose_01", 1), ("Klima_Sensor_02", 1),
          ("Klima_11", 5), ("Klima_12", 5)]
  # 해석기는 DBC 를 읽어 만드는 데 비용이 커서 한 번만 만들고 재사용한다
  global _CAN_PARSERS
  if _CAN_PARSERS is None:
    try:
      from opendbc.can import CANParser
      _CAN_PARSERS = [CANParser("vw_meb", MSGS, bus) for bus in (0, 1)]
    except Exception as exc:
      print(f"Wayon telemetry: CAN parser unavailable: {exc}", flush=True)
      return result
  parsers = _CAN_PARSERS

  sock = get_can_sock()
  deadline = time.monotonic() + max(0.5, timeout_s)
  capacity_ah = None
  bms_volts: list[float] = []
  bms_amps: list[float] = []
  bms_modes: list[int] = []
  while time.monotonic() < deadline:
    msgs = messaging.drain_sock(sock)
    if not msgs:
      continue
    frames = [(m.logMonoTime, [(f.address, f.dat, f.src) for f in m.can]) for m in msgs]

    # 0x0CF: DBC 에 없어 원시 바이트로 해석 (식은 BMS_0CF_ADDR 주석)
    for _, cans in frames:
      for addr, dat, src in cans:
        if addr != BMS_0CF_ADDR or src not in (0, 1) or len(dat) < 8:
          continue
        d = bytes(dat)
        bms_modes.append(d[2] & 0x07)
        bms_amps.append(((((d[4] & 0x7F) << 8) | d[3]) - 16300) * 0.1)
        bms_volts.append(((d[7] << 4) | (d[6] >> 4)) * 0.25)

    for cp in parsers:
      try:
        cp.update(frames)
      except Exception:
        continue

      # vl은 미수신이어도 기본값 0을 주므로, vl_all(이번 update에서 실제 수신된 값)로 판정한다
      wh = cp.vl_all["Motor_16"].get("MO_Energieinhalt_BMS") or \
           cp.vl_all["HVEM_02"].get("HVEM_Nutzbare_Energie")
      if wh:
        v_wh = valid_battery_wh(wh[-1] if isinstance(wh, (list, tuple)) else wh)
        if v_wh is not None:
          result["battery_wh"] = v_wh
      if cp.vl_all["Diagnose_01"].get("KBI_Kilometerstand"):
        odo = cp.vl["Diagnose_01"]["KBI_Kilometerstand"]
        if 0 < odo <= ODO_MAX_VALID_KM:
          result["odometer_km"] = float(odo)
      if cp.vl_all["Klima_Sensor_02"].get("BCM1_Aussen_Temp_ungef"):
        result["outside_temp_c"] = float(cp.vl["Klima_Sensor_02"]["BCM1_Aussen_Temp_ungef"])
      if cp.vl_all["BMS_04"].get("BMS_Kapazitaet_02"):
        ah = cp.vl["BMS_04"]["BMS_Kapazitaet_02"]
        if ah > 0:
          capacity_ah = float(ah)

      # --- 공조 상태 (2026-08 실차 확인: Klima_11=bus1, Klima_12=bus0) ---
      if cp.vl_all["Klima_11"].get("KL_AC_Schalter") is not None:
        result["ac_on"] = bool(cp.vl["Klima_11"]["KL_AC_Schalter"])
      if cp.vl_all["Klima_12"].get("KL_Geblspng_Soll"):
        volt = float(cp.vl["Klima_12"]["KL_Geblspng_Soll"])
        if volt > 0:
          result["blower_volt"] = round(volt, 2)
          # 1.45V=꺼짐, 14V=최대. 0~10단으로 환산해 표시한다.
          result["blower_level"] = max(0, min(10, round((volt - 1.45) / (14.0 - 1.45) * 10)))
      for sig, key in (("KL_SIH_Soll_li", "seat_heat_left"), ("KL_SIH_Soll_re", "seat_heat_right")):
        if cp.vl_all["Klima_12"].get(sig) is not None:
          result[key] = int(cp.vl["Klima_12"][sig])
      if cp.vl_all["Klima_12"].get("KL_Umluftklappe_Status") is not None:
        result["recirc"] = int(cp.vl["Klima_12"]["KL_Umluftklappe_Status"])

    if "battery_wh" in result and "odometer_km" in result:
      break

  # 0x0CF 집계: 샘플 구간 평균. 모드는 가장 많이 나온 값.
  hv_voltage = None
  if bms_volts:
    hv_voltage = sum(bms_volts) / len(bms_volts)
    if not 200.0 <= hv_voltage <= 500.0:  # 상식 밖이면 해석이 틀린 것 -> 이번 값 전부 버림
      hv_voltage = None
  if hv_voltage is not None:
    amps = sum(bms_amps) / len(bms_amps)
    power_w = sum(v * a for v, a in zip(bms_volts, bms_amps, strict=True)) / len(bms_volts)
    result["battery_current_a"] = round(amps, 1)
    result["battery_power_w"] = round(power_w, 0)
    result["bms_mode"] = max(set(bms_modes), key=bms_modes.count)

  # 측정 용량: BMS 용량[Ah] x HV 전압[V]
  # 주의: SOH(열화율)가 아니다. BMS가 온도/충전상태에 따라 그때그때 다시 추정하는 값이라
  # 세션마다 72~90kWh로 요동친다(실측, 164Ah <-> 196Ah). 열화율로 환산하면 며칠 만에
  # 88% -> 100%처럼 말이 안 되는 변화가 나오므로 측정값 그대로만 올린다.
  # 잔량%의 분모로도 쓰지 않는다(USABLE_BATTERY_WH 고정값 사용).
  if capacity_ah and hv_voltage:
    capacity_wh = capacity_ah * hv_voltage
    if 20000 < capacity_wh < 150000:  # 상식 범위 밖이면 스케일 해석이 틀린 것 -> 버림
      result["measured_capacity_wh"] = round(capacity_wh, 0)
  if hv_voltage:
    result["hv_voltage"] = round(hv_voltage, 1)
  return result


def load_efficiency() -> dict:
  try:
    data = json.loads(EFFICIENCY_PATH.read_text(encoding="utf-8"))
    if isinstance(data, dict) and isinstance(data.get("trips"), list):
      return data
  except (OSError, ValueError):
    pass
  return {"trips": []}


def record_trip_efficiency(distance_km: float, energy_kwh: float) -> bool:
  """트립 하나의 거리·소비 에너지를 전비 기록에 더한다. 기록했으면 True."""
  if distance_km < EFFICIENCY_MIN_KM or energy_kwh < EFFICIENCY_MIN_KWH:
    return False
  kpl = distance_km / energy_kwh
  lo, hi = EFFICIENCY_VALID_KPL
  if not lo <= kpl <= hi:
    print(f"Wayon telemetry: efficiency {kpl:.2f} km/kWh rejected "
          f"({distance_km:.1f}km / {energy_kwh:.2f}kWh)", flush=True)
    return False
  data = load_efficiency()
  data["trips"].append({"at": utc_now(), "km": round(distance_km, 2), "kwh": round(energy_kwh, 3)})
  data["trips"] = data["trips"][-EFFICIENCY_KEEP_TRIPS:]
  EFFICIENCY_PATH.parent.mkdir(parents=True, exist_ok=True)
  tmp = EFFICIENCY_PATH.with_suffix(".tmp")
  tmp.write_text(json.dumps(data, separators=(",", ":")), encoding="utf-8")
  os.replace(tmp, EFFICIENCY_PATH)
  return True


def efficiency_summary(data: dict) -> dict:
  """최근 트립 전비. efficiency_kpl = 거리 가중 평균(앱 표시용),
  range_kpl = 트립별 전비의 하위 25%(주행가능거리용, 추위·고속주행에 덜 낙관적)."""
  trips = [t for t in data.get("trips", []) if t.get("km", 0) > 0 and t.get("kwh", 0) > 0]
  if not trips:
    return {"range_kpl": DEFAULT_EFFICIENCY_KPL, "efficiency_trips": 0}
  total_km = sum(t["km"] for t in trips)
  avg = total_km / sum(t["kwh"] for t in trips)
  per_trip = sorted(t["km"] / t["kwh"] for t in trips)
  range_kpl = per_trip[len(per_trip) // 4] if len(per_trip) >= RANGE_QUARTILE_MIN_TRIPS else avg
  return {
    "efficiency_kpl": round(avg, 2),
    "range_kpl": round(min(avg, range_kpl), 2),
    "efficiency_trips": len(trips),
    "efficiency_km": round(total_km, 1),
  }


def estimate_charge_minutes(battery_wh: float, power_w: float, kind: str | None, target_pct: int) -> float | None:
  """지금 배터리 측 전력으로 target_pct(잔량%, USABLE_BATTERY_WH 기준)까지 남은 분.

  AC 는 끝까지 거의 일정한 전력이라 남은 에너지 / 전력. DC 는 1% 씩 걸으며 ID.4 충전 곡선을
  따라 전력을 깎는다. 충전기가 곡선보다 낮게 주고 있으면(충전기 한계) 곡선이 그 아래로
  떨어질 때까지 지금 전력을 유지하고, 곡선만큼 받고 있으면 곡선 비율대로 줄인다.
  """
  if power_w < CHARGING_MIN_W:
    return None
  soc = battery_wh / USABLE_BATTERY_WH * 100.0
  if soc >= target_pct:
    return 0.0
  wh_per_pct = USABLE_BATTERY_WH / 100.0
  if kind != "dc":
    return (target_pct - soc) * wh_per_pct / power_w * 60.0

  power_kw = power_w / 1000.0
  curve_now = ID4_DC_CURVE_KW[min(100, int(soc))]
  limited = power_kw < curve_now * 0.9
  minutes = 0.0
  step = soc
  while step < target_pct:
    end = min(float(target_pct), float(int(step) + 1))
    curve = ID4_DC_CURVE_KW[min(100, int(end))]
    kw = min(power_kw, curve) if limited else curve * power_kw / curve_now
    minutes += (end - step) * wh_per_pct / 1000.0 / max(kw, 1.0) * 60.0
    step = end
  return minutes


def haversine_m(a: tuple, b: tuple) -> float:
  """두 위경도 사이 거리[m]."""
  import math
  lat1, lon1 = a
  lat2, lon2 = b
  r = 6371000.0
  p1, p2 = math.radians(lat1), math.radians(lat2)
  dp = math.radians(lat2 - lat1)
  dl = math.radians(lon2 - lon1)
  h = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
  return 2 * r * math.asin(min(1.0, math.sqrt(h)))


class TripRecorder:
  """주행 시작~종료를 기록해 Wayon Cloud에 올린다.

  onroad 전환으로 시작/종료를 잡고, 주행 중 GPS를 샘플링해 경로와 거리를 만든다.
  이게 없으면 앱의 '주행 이력'이 계속 비어 있다 (기존에 업로더가 아예 없었음).
  """

  MIN_POINT_GAP_S = 5.0      # 경로 점 최소 간격
  MIN_TRIP_M = 300.0         # 이보다 짧으면 버림(주차장 이동 등)
  MAX_POINTS = 720           # 한 트립 최대 점 수 (1시간 @5초)

  def __init__(self):
    self.active = False
    self.started_mono = None
    self.route: list = []
    self.distance_m = 0.0
    self.last_point = None
    self.last_sample_at = 0.0
    self.trip_id = None
    self.start_wh = None
    self.end_wh = None

  def start(self):
    import uuid
    self.active = True
    # 시작 시각을 벽시계로 찍지 않는다. 차가 깨어나 onroad 가 되는 시점엔 아직
    # 시간 동기화 전이라 RTC 가 몇 달 전을 가리킬 수 있다(실측: 2026-03-24 로 시작된
    # 134일짜리 트립). 대신 monotonic 으로 경과시간만 재고, 종료 시점의 (그때는
    # 동기화가 끝난) 시계에서 빼서 시작 시각을 역산한다.
    self.started_mono = time.monotonic()
    self.route = []
    self.distance_m = 0.0
    self.last_point = None
    self.last_sample_at = 0.0
    self.start_wh = None
    self.end_wh = None
    self.trip_id = str(uuid.uuid4())
    print("Wayon telemetry: trip started", flush=True)

  def add_point(self, lat: float, lon: float, speed_mps: float | None, now: float):
    if not self.active or now - self.last_sample_at < self.MIN_POINT_GAP_S:
      return
    self.last_sample_at = now
    if self.last_point is not None:
      d = haversine_m(self.last_point, (lat, lon))
      if d < 2000:      # GPS 튐 방지
        self.distance_m += d
    self.last_point = (lat, lon)
    if len(self.route) < self.MAX_POINTS:
      point = {"latitude": round(lat, 6), "longitude": round(lon, 6), "t": utc_now()}
      if speed_mps is not None:
        point["speedMps"] = round(speed_mps, 2)
      self.route.append(point)

  def note_battery(self, wh: float | None):
    """주행 중 배터리[Wh]. 처음 본 값이 출발, 마지막 값이 도착 -> 트립 소비 에너지."""
    if not self.active or wh is None:
      return
    if self.start_wh is None:
      self.start_wh = wh
    self.end_wh = wh

  def finish(self, config: dict, device_id: str) -> bool:
    """주행 종료 -> 업로드. 성공/실패와 무관하게 상태는 초기화한다."""
    if not self.active:
      return False
    self.active = False
    if self.distance_m < self.MIN_TRIP_M or len(self.route) < 2:
      print(f"Wayon telemetry: trip discarded ({self.distance_m:.0f}m)", flush=True)
      return False

    if self.start_wh is not None and self.end_wh is not None:
      km = self.distance_m / 1000.0
      kwh = (self.start_wh - self.end_wh) / 1000.0
      if record_trip_efficiency(km, kwh):
        print(f"Wayon telemetry: trip efficiency {km / kwh:.2f} km/kWh ({km:.1f}km, {kwh:.2f}kWh)", flush=True)

    from datetime import datetime, timedelta
    now = datetime.now(UTC)
    if now.year < 2025:
      # 시계가 아직 안 맞았다. 지금 올리면 started_at/ended_at 둘 다 엉뚱한 값이 되고,
      # 앱의 '리셋 후 주행거리'가 이 트립을 잘못 포함/제외한다. 버리는 편이 낫다.
      print("Wayon telemetry: trip discarded (clock not synced)", flush=True)
      return False

    duration_s = max(0, int(time.monotonic() - (self.started_mono or time.monotonic())))
    fmt = "%Y-%m-%dT%H:%M:%SZ"
    ended_at = now.strftime(fmt)
    started_at = (now - timedelta(seconds=duration_s)).strftime(fmt)

    payload = {
      "id": self.trip_id,
      "deviceId": device_id,
      "startedAt": started_at,
      "endedAt": ended_at,
      "durationS": duration_s,
      "distanceM": round(self.distance_m, 1),
      "route": self.route,
    }
    ok = post_json(config, "/api/trips", payload)
    print(f"Wayon telemetry: trip upload {'ok' if ok else 'FAILED'} "
          f"({self.distance_m / 1000:.2f}km, {len(self.route)}pts)", flush=True)
    return ok


def post_json(config: dict, path: str, payload: dict) -> bool:
  endpoint = str(config.get("endpoint") or "").rstrip("/")
  token = str(config.get("token") or "")
  if not endpoint or not token:
    return False
  try:
    response = requests.post(
      f"{endpoint}{path}",
      json=payload,
      headers={"Authorization": f"Bearer {token}", "Content-Type": "application/json"},
      timeout=(10, 30),
    )
    return 200 <= response.status_code < 300
  except requests.RequestException as exc:
    print(f"Wayon telemetry: POST {path} failed: {exc}", flush=True)
    return False


def post_state(config: dict, payload: dict, timeout=(10, 30)) -> bool:
  endpoint = str(config.get("endpoint") or "").rstrip("/")
  token = str(config.get("token") or "")
  if not endpoint or not token:
    return False
  try:
    response = requests.post(
      f"{endpoint}/api/telemetry",
      json=payload,
      headers={"Authorization": f"Bearer {token}", "Content-Type": "application/json"},
      timeout=timeout,
    )
    return 200 <= response.status_code < 300
  except requests.RequestException as exc:
    print(f"Wayon telemetry: upload failed: {exc}", flush=True)
    return False


def main() -> None:
  params = Params()
  # carrot에는 liveLocationKalman이 없다 (gpsLocation / gpsLocationExternal 사용)
  # carState is deliberately not subscribed: msgq evicts every subscriber of a service once more than
  # 15 readers attach, and carState already has ~14 on StarPilot (commIssue on 2026-11-26).
  # Battery comes from the direct CAN sample, speed from livePose.
  sm = messaging.SubMaster(["livePose", "gpsLocation", "gpsLocationExternal", "peripheralState"])

  last = load_last_state()
  charge_ledger = load_charge_ledger()
  last_battery_wh = last.get("battery_wh")
  last_battery_at = None  # monotonic, 이번 실행에서만 사용
  last_upload_at = None   # None = 아직 한 번도 안 올림 -> 즉시 업로드
  last_full_at = None     # 마지막 CAN 전체 샘플링(monotonic)
  cached_sampled: dict = {}
  charge_power_w = None
  trip = TripRecorder()
  onroad_prev = None
  # 도어 상태는 마지막으로 알던 값에서 시작한다(재시작했다고 '바뀜'으로 오인하지 않게)
  door_keys = ("door_locked_inside", "door_locked_outside", "door_safe", "doors_open",
               "door_changed_at", "door_seen_at")
  door_state = {k: last.get(k) for k in door_keys}
  door_pending = False

  print("Wayon telemetry: started", flush=True)

  while True:
    time.sleep(1.0)
    sm.update(0)

    config = read_config()
    if not config.get("endpoint") or not config.get("token"):
      continue

    onroad = params.get_bool("IsOnroad")
    now = time.monotonic()

    dongle_id = params.get("DongleId")
    if isinstance(dongle_id, bytes):
      dongle_id = dongle_id.decode("utf-8", "replace")
    device_id = str(config.get("device_id") or dongle_id or "unknown")

    # --- 주행 기록: onroad 전환으로 시작/종료 ---
    if onroad_prev is not None and onroad != onroad_prev:
      if onroad:
        trip.start()
      else:
        trip.finish(config, device_id)
    onroad_prev = onroad

    if onroad and trip.active:
      for svc in ("gpsLocationExternal", "gpsLocation"):
        if sm.valid.get(svc) and sm.seen.get(svc):
          g = sm[svc]
          if g.hasFix:
            trip.add_point(float(g.latitude), float(g.longitude), float(g.speed), now)
            break

    # 배터리 잔량은 주행·주차 모두 아래의 CAN 직접 샘플(Motor_16)에서만 얻는다.
    # carState 는 구독하지 않는다(위 SubMaster 주석: msgq 구독자 15개 초과 시 전원 퇴출).
    battery_wh = None

    # 도어 잠금/열림: 매초 확인하고, 바뀌면 다음 주기를 기다리지 않고 올린다.
    door = read_door_state(get_can_sock())
    if door:
      stamp = utc_now()
      door_state["door_seen_at"] = stamp
      if any(door_state.get(k) != v for k, v in door.items()):
        door_state.update(door)
        door_state["door_changed_at"] = stamp
        door_pending = True
    door_due = door_pending and (last_upload_at is None or now - last_upload_at >= DOOR_UPLOAD_MIN_GAP_S)

    interval = TELEMETRY_ONROAD_S if onroad else TELEMETRY_OFFROAD_S
    battery_moved = (
      battery_wh is not None and last.get("battery_wh") is not None
      and abs(battery_wh - float(last["battery_wh"])) >= BATTERY_DELTA_TRIGGER_WH
    )
    # 부팅 직후엔 monotonic이 작아 'now - 0 < interval'이 참이 되어 첫 업로드가
    # 최대 10분 지연되던 문제가 있었다. 첫 회는 무조건 올린다.
    if last_upload_at is not None and now - last_upload_at < interval and not battery_moved and not door_due:
      continue

    # 배터리/오도미터/외기온·용량은 CAN에서 직접 샘플링 (업로드 직전에만).
    # 차가 완전히 잠들면 CAN이 없어 빈 dict가 온다.
    # 주행 중 가벼운 업로드(3초)는 CAN 을 다시 읽지 않고 직전 샘플을 재사용한다.
    full = (not onroad or last_full_at is None or battery_moved
            or now - last_full_at >= TELEMETRY_ONROAD_FULL_S)
    if full:
      sampled = sample_vehicle_can()
      cached_sampled = dict(sampled)
      last_full_at = now
    else:
      # Without carState the last CAN sample is the freshest battery value; keep it (30 s old at most).
      sampled = dict(cached_sampled)

    # CAN 샘플값이 배터리의 유일한 소스다(주행 중 가벼운 업로드는 30초 이내 직전 샘플).
    if sampled.get("battery_wh"):
      battery_wh = sampled["battery_wh"]
      trip.note_battery(battery_wh)
      # 충전 판정도 이 값으로 다시 계산 (증가 기울기)
      if last_battery_at is not None and last_battery_wh is not None:
        dt_h = (now - last_battery_at) / 3600.0
        if dt_h > 0.02:
          delta_wh = battery_wh - last_battery_wh
          charge_power_w = delta_wh / dt_h if delta_wh > 0 else None
      if last_battery_at is None or abs(battery_wh - (last_battery_wh or 0)) >= BATTERY_DELTA_TRIGGER_WH:
        last_battery_wh, last_battery_at = battery_wh, now

    # 0x0CF 실측(배터리 측 V×I)이 있으면 위의 기울기 추정 대신 그 값을 쓴다.
    bms_mode = sampled.get("bms_mode")
    if sampled.get("battery_power_w") is not None:
      measured_w = float(sampled["battery_power_w"])
      charge_power_w = measured_w if measured_w > 0 else None

    # 오도미터 급변 방어: 계기판이 깨어나는 중이면 엉뚱한 값이 한 번씩 섞인다.
    # 이전에 알던 값과 1000km 이상 벌어지면 이번 값은 버리고 기존 값을 유지한다.
    sampled_odo = sampled.get("odometer_km")
    if sampled_odo is not None:
      known_odo = last.get("odometer_km")
      if known_odo is not None and abs(sampled_odo - float(known_odo)) >= ODO_MAX_JUMP_KM:
        print(f"Wayon telemetry: odometer {sampled_odo:.0f} rejected "
              f"(last {float(known_odo):.0f})", flush=True)
        sampled.pop("odometer_km")

    # 충전량 누적.
    # 기준점(last_wh/last_at)은 원장 파일에 같이 저장한다. 메모리에만 두면 기기가
    # 자거나 매니저가 재시작될 때마다 날아가서, 그 사이에 들어간 전력을 통째로
    # 놓친다(실측: 배터리는 42.1kWh 늘었는데 30.0kWh 만 집계됨).
    # 주행 중 회생제동도 배터리를 늘리므로 주차 중(offroad)에만 센다.
    # 주행 중 3초 업로드마다 파일을 쓰지 않도록 전체 샘플링 때만 갱신한다.
    if battery_wh is not None and full:
      prev_wh = charge_ledger.get("last_wh")
      prev_at = charge_ledger.get("last_at")
      if not onroad and prev_wh is not None and prev_at is not None and battery_wh > prev_wh:
        hours = max(0.0, time.time() - float(prev_at)) / 3600.0  # noqa: TID251 wall clock survives reboots
        add_charge_energy(charge_ledger, battery_wh - float(prev_wh), hours,
                          kind_hint=BMS_CHARGE_MODES.get(bms_mode))
      charge_ledger["last_wh"] = battery_wh
      charge_ledger["last_at"] = time.time()  # noqa: TID251
      save_charge_ledger(charge_ledger)

    vehicle = build_vehicle_block(battery_wh, charge_power_w, sampled.get("capacity_wh"), bms_mode)
    vehicle["charge_months"] = charge_ledger.get("months", {})
    for key in ("odometer_km", "outside_temp_c", "hv_voltage", "measured_capacity_wh",
                "battery_current_a", "battery_power_w",
                "ac_on", "blower_volt", "blower_level", "seat_heat_left", "seat_heat_right", "recirc"):
      if sampled.get(key) is not None:
        vehicle[key] = sampled[key]

    # 12V 보조 배터리: 판다가 차량 전원선에서 측정한 입력 전압
    if sm.valid.get("peripheralState") and sm.seen.get("peripheralState"):
      mv = sm["peripheralState"].voltage
      # 차가 잠들면 판다가 자체 대기전원(~5V)을 보고한다. 12V계 실측으로 볼 수 있는
      # 범위(9V 이상)일 때만 채택한다.
      if mv and mv >= 9000:
        vehicle["aux_voltage"] = round(mv / 1000.0, 2)

    # 이번에 새로 측정된 값이 있으면 '마지막 알려진 값'으로 저장
    for key in ("battery_wh", "capacity_wh", "soc_percent", "odometer_km", "outside_temp_c",
                "hv_voltage", "aux_voltage", "measured_capacity_wh",
                "ac_on", "blower_volt", "blower_level", "seat_heat_left", "seat_heat_right", "recirc"):
      if vehicle.get(key) is not None:
        last[key] = vehicle[key]
        last["measured_at"] = utc_now()
    vehicle = merge_last_known(vehicle, last)

    # 전비·주행가능거리: 콤마가 기록한 최근 트립 전비 x 현재(또는 마지막) 배터리
    eff = efficiency_summary(load_efficiency())
    vehicle.update(eff)
    if vehicle.get("battery_wh"):
      vehicle["range_km"] = round(vehicle["battery_wh"] / 1000.0 * eff["range_kpl"])

    # 충전 완료 예상[분]. 기준 시각을 같이 실어 앱이 '몇 시 몇 분'으로 바꾼다.
    if vehicle.get("charging") and vehicle.get("charge_power_w") and battery_wh:
      for target in CHARGE_ETA_TARGETS:
        minutes = estimate_charge_minutes(battery_wh, vehicle["charge_power_w"],
                                          vehicle.get("charge_kind"), target)
        if minutes is not None:
          vehicle[f"charge_eta_{target}_min"] = round(minutes)
      vehicle["charge_eta_base"] = utc_now()

    # 도어 상태: 마지막으로 본 값을 그대로 싣는다. 차가 잠들어 버스가 조용해도
    # 잠금/해제는 차를 깨우므로, 콤마가 켜져 있는 동안엔 바뀌면 바로 보인다.
    for key, value in door_state.items():
      if value is not None:
        vehicle[key] = value
        last[key] = value
    if door_state.get("door_locked_inside") is not None or door_state.get("door_locked_outside") is not None:
      vehicle["door_locked"] = bool(door_state.get("door_locked_inside") or door_state.get("door_locked_outside"))

    # Worker(/api/telemetry)는 camelCase + 중첩 gps 를 기대하며, 페이로드 전체를 raw_json 에 저장한다.
    # 앱은 raw_json.vehicle 을 읽어 배터리/충전 정보를 표시한다.
    dongle = params.get("DongleId")
    if isinstance(dongle, bytes):
      dongle = dongle.decode("utf-8", "replace")
    payload: dict = {
      "deviceId": str(config.get("device_id") or dongle or "unknown"),
      "updatedAt": utc_now(),
      "onroad": 1 if onroad else 0,
      "ignition": 1 if onroad else 0,
      "enabled": 0,
      "vehicle": vehicle,
      "gps": {},
    }

    # 주행 속도: 바퀴 속도(carState.vEgo). Worker 는 vehicleSpeedMps 를 GPS 속도보다 먼저 쓰므로
    # GPS 가 비어도(터널·수신 불량) 앱에 속도가 나온다(carrot-ha collector 와 같은 방식).
    # CAN 샘플링에 몇 초 걸리므로 10초 이내에 받은 carState 면 쓴다.
    if onroad and sm.seen.get("livePose") and time.monotonic() - sm.recv_time["livePose"] < 10:
      lp = sm["livePose"]
      if lp.sensorsOK:
        payload["vehicleSpeedMps"] = round(max(float(lp.velocityDevice.x), 0.0), 2)

    for gps_service in ("gpsLocationExternal", "gpsLocation"):
      if sm.valid.get(gps_service) and sm.seen.get(gps_service):
        gps = sm[gps_service]
        if gps.hasFix:
          payload["gps"]["latitude"] = float(gps.latitude)
          payload["gps"]["longitude"] = float(gps.longitude)
          payload["gps"]["speedMps"] = float(gps.speed)
          payload["gps"]["bearingDeg"] = float(gps.bearingDeg)
          payload["gps"]["accuracyM"] = float(gps.horizontalAccuracy)
          # 클라우드는 이 시각으로 위치가 최신인지(fresh) 판정한다
          if gps.unixTimestampMillis > 0:
            payload["gps"]["timestampMillis"] = int(gps.unixTimestampMillis)
          break

    if post_state(config, payload, timeout=(10, 30) if full else LIGHT_POST_TIMEOUT):
      last_upload_at = now
      door_pending = False
      last["updated_at"] = payload["updatedAt"]
      # 가벼운 업로드(주행 중 3초)마다 파일을 쓰거나 로그를 남기지 않는다
      if not full:
        continue
      save_last_state(last)   # 마지막 측정값 유지 (주차 후에도 표시하기 위함)
      print(f"Wayon telemetry: uploaded soc={vehicle.get('soc_percent')}% "
            f"odo={vehicle.get('odometer_km')} charging={vehicle.get('charging')} "
            f"kind={vehicle.get('charge_kind')} power={vehicle.get('charge_power_w')} "
            f"bms_mode={vehicle.get('bms_mode')} hv={vehicle.get('hv_voltage')} "
            f"range={vehicle.get('range_km')}km@{vehicle.get('range_kpl')} "
            f"eta80={vehicle.get('charge_eta_80_min')} eta100={vehicle.get('charge_eta_100_min')} "
            f"locked={vehicle.get('door_locked')} open={vehicle.get('doors_open')} "
            f"stale={vehicle.get('stale', False)}", flush=True)


if __name__ == "__main__":
  main()
