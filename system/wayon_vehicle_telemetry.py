#!/usr/bin/env python3
"""ID.4(MEB) 차량 상태를 Wayon Cloud로 올린다.

배터리 잔량·오도미터·외기온·12V 전압과 GPS를 주기적으로 POST해서 휴대폰 앱이
차 상태를 볼 수 있게 한다.

배터리는 CAN(Motor_16)에서 직접 디코딩한다. carState는 card 프로세스가 온로드에서만
돌아 주차 중엔 비어 있어, 충전 중 감시가 되려면 CAN 직접 샘플링이어야 한다.
충전 전력은 배터리량 증가 기울기로 추정한다(전용 신호 MEB_HVEM_01 수신 여부 미검증).

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

CONFIG_PATH = Path(os.getenv("WAYON_CLOUD_CONFIG", "/data/wayon_cloud/config.json"))
STATE_PATH = Path(os.getenv("WAYON_VEHICLE_STATE", "/data/wayon_cloud/vehicle_state.json"))

TELEMETRY_ONROAD_S = 30.0
TELEMETRY_OFFROAD_S = 600.0
# 주차 중 배터리량이 이만큼(Wh) 변하면 즉시 업로드 -> 충전 시작/종료를 빨리 반영
BATTERY_DELTA_TRIGGER_WH = 300.0
# 잔량% 계산 분모. 차량 계기판 표시와 맞추기 위해 64kWh로 고정한다.
# (BMS가 주는 실측 용량은 값이 계속 변해 계기판과 어긋난다)
USABLE_BATTERY_WH = 64000.0
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

# can 구독 소켓 (재사용)
_CAN_SOCK = None


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
  from datetime import datetime, timezone
  return datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


def build_vehicle_block(battery_wh: float | None, charge_power_w: float | None,
                        capacity_wh: float | None = None) -> dict:
  vehicle: dict = {}
  # 분모는 항상 고정값(계기판 일치). capacity_wh(BMS 실측)는 SOH 전용.
  usable = USABLE_BATTERY_WH
  if battery_wh is not None and battery_wh > 0:
    vehicle["battery_wh"] = round(battery_wh, 1)
    vehicle["capacity_wh"] = round(usable, 0)
    vehicle["soc_percent"] = round(min(100.0, battery_wh / usable * 100.0), 1)
  if charge_power_w is not None and charge_power_w >= CHARGING_MIN_W:
    vehicle["charging"] = True
    vehicle["charge_power_w"] = round(charge_power_w, 0)
  else:
    vehicle["charging"] = False
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


def add_charge_energy(ledger: dict, delta_wh: float, hours: float) -> None:
  """늘어난 에너지를 구간 평균 전력으로 완속/급속을 나눠 이번 달에 더한다.

  전력은 '지금 값'이 아니라 (증가분 / 경과시간)으로 구한다. 기기가 자다 깨면
  몇 시간치가 한 번에 들어오는데, 그때 순간 전력을 쓰면 판정이 엉뚱해진다.
  """
  if delta_wh <= 0 or hours <= 0:
    return
  avg_power_w = delta_wh / hours
  # 상식 밖이면 계측 오류로 본다 (MEB 최대 급속이 ~175kW)
  if avg_power_w > 250000:
    return

  from datetime import datetime, timezone
  # 월 구분은 한국 시간 기준(KST=UTC+9). 월말 자정 근처 충전이 엉뚱한 달로 가지 않게.
  now = datetime.now(timezone.utc).timestamp() + 9 * 3600
  month = datetime.utcfromtimestamp(now).strftime("%Y-%m")

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


def sample_vehicle_can(timeout_s: float = 6.0) -> dict:
  """차량 값들을 CAN에서 짧게 샘플링한다.

  carState는 card 프로세스가 온로드에서만 돌아 주차 중엔 비어 있다.
  그래서 배터리까지 여기서 직접 디코딩해야 충전 중 모니터링이 된다 (주행 코드 무수정).

  주차/충전 중 실측(2026-08): 아래가 모두 bus1로 수신된다. 주행 중엔 bus0로 오므로
  양쪽 버스를 다 파싱한다.
  - Motor_16.MO_Energieinhalt_BMS      : HV 배터리 에너지량[Wh]  <- 잔량/충전 판정의 핵심
  - HVEM_02.HVEM_Nutzbare_Energie      : 가용 에너지[Wh] (Motor_16과 동일값으로 교차검증됨)
  - MEB_HVEM_01.Battery_Voltage        : HV 전압[V]  (실측 369.2V)
  - BMS_04.BMS_Kapazitaet_02           : 배터리 용량[Ah] -> 실제 용량/SOH
  - Diagnose_01.KBI_Kilometerstand     : 계기판 총 주행거리[km]
  - Klima_Sensor_02.BCM1_Aussen_Temp_ungef : 외기온도[degC] (주행 중에만 수신)
  """
  result: dict = {}
  # 같은 메시지가 주행 중엔 bus0, 주차/충전 중엔 bus1로 온다(실측). 양쪽 다 파싱한다.
  MSGS = [("Motor_16", 2), ("HVEM_02", 10), ("MEB_HVEM_01", 100),
          ("BMS_04", 2), ("Diagnose_01", 1), ("Klima_Sensor_02", 1),
          ("Klima_11", 5), ("Klima_12", 5)]
  try:
    from opendbc.can import CANParser
    parsers = [CANParser("vw_meb", MSGS, bus) for bus in (0, 1)]
  except Exception as exc:
    print(f"Wayon telemetry: CAN parser unavailable: {exc}", flush=True)
    return result

  # 소켓을 매번 새로 열면 첫 호출(프로세스 기동 직후)에 연결 워밍업 때문에 아무것도 못 받는다.
  # 한 번 만들어 재사용한다.
  global _CAN_SOCK
  if _CAN_SOCK is None:
    _CAN_SOCK = messaging.sub_sock("can", timeout=200)
    time.sleep(0.5)          # 구독 성립 대기
    messaging.drain_sock(_CAN_SOCK)
  sock = _CAN_SOCK
  deadline = time.monotonic() + max(0.5, timeout_s)
  hv_voltage = None
  capacity_ah = None
  while time.monotonic() < deadline:
    msgs = messaging.drain_sock(sock)
    if not msgs:
      continue
    frames = [(m.logMonoTime, [(f.address, f.dat, f.src) for f in m.can]) for m in msgs]
    for cp in parsers:
      try:
        cp.update(frames)
      except Exception:
        continue

      # vl은 미수신이어도 기본값 0을 주므로, vl_all(이번 update에서 실제 수신된 값)로 판정한다
      wh = cp.vl_all["Motor_16"].get("MO_Energieinhalt_BMS") or \
           cp.vl_all["HVEM_02"].get("HVEM_Nutzbare_Energie")
      if wh:
        v_wh = wh[-1] if isinstance(wh, (list, tuple)) else wh
        if v_wh > 0:
          result["battery_wh"] = float(v_wh)
      if cp.vl_all["Diagnose_01"].get("KBI_Kilometerstand"):
        odo = cp.vl["Diagnose_01"]["KBI_Kilometerstand"]
        if 0 < odo <= ODO_MAX_VALID_KM:
          result["odometer_km"] = float(odo)
      if cp.vl_all["Klima_Sensor_02"].get("BCM1_Aussen_Temp_ungef"):
        result["outside_temp_c"] = float(cp.vl["Klima_Sensor_02"]["BCM1_Aussen_Temp_ungef"])
      if cp.vl_all["MEB_HVEM_01"].get("Battery_Voltage"):
        v = cp.vl["MEB_HVEM_01"]["Battery_Voltage"]
        if v > 0:
          hv_voltage = float(v)
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

  def finish(self, config: dict, device_id: str) -> bool:
    """주행 종료 -> 업로드. 성공/실패와 무관하게 상태는 초기화한다."""
    if not self.active:
      return False
    self.active = False
    if self.distance_m < self.MIN_TRIP_M or len(self.route) < 2:
      print(f"Wayon telemetry: trip discarded ({self.distance_m:.0f}m)", flush=True)
      return False

    from datetime import datetime, timedelta, timezone
    now = datetime.now(timezone.utc)
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


def post_state(config: dict, payload: dict) -> bool:
  endpoint = str(config.get("endpoint") or "").rstrip("/")
  token = str(config.get("token") or "")
  if not endpoint or not token:
    return False
  try:
    response = requests.post(
      f"{endpoint}/api/telemetry",
      json=payload,
      headers={"Authorization": f"Bearer {token}", "Content-Type": "application/json"},
      timeout=(10, 30),
    )
    return 200 <= response.status_code < 300
  except requests.RequestException as exc:
    print(f"Wayon telemetry: upload failed: {exc}", flush=True)
    return False


def main() -> None:
  params = Params()
  # carrot에는 liveLocationKalman이 없다 (gpsLocation / gpsLocationExternal 사용)
  sm = messaging.SubMaster(["carState", "gpsLocation", "gpsLocationExternal", "peripheralState"])

  last = load_last_state()
  charge_ledger = load_charge_ledger()
  last_battery_wh = last.get("battery_wh")
  last_battery_at = None  # monotonic, 이번 실행에서만 사용
  last_upload_at = None   # None = 아직 한 번도 안 올림 -> 즉시 업로드
  charge_power_w = None
  trip = TripRecorder()
  onroad_prev = None

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

    # 배터리 잔량: 주행 중엔 carState(가벼움), 주차 중엔 CAN 직접 샘플링으로 얻는다.
    # (card 프로세스가 온로드 전용이라 주차 중엔 carState가 비어 충전 감시가 안 됐다)
    battery_wh = None
    if sm.updated["carState"] and sm.valid["carState"]:
      gauge = sm["carState"].fuelGauge
      if gauge > 0:
        battery_wh = gauge * USABLE_BATTERY_WH

    # 주차 중 배터리 증가 기울기로 충전 전력 추정
    if battery_wh is not None:
      if last_battery_wh is not None and last_battery_at is not None:
        dt_h = (now - last_battery_at) / 3600.0
        if dt_h > 0.02:  # 최소 ~1분 간격에서만 산출 (노이즈 방지)
          delta_wh = battery_wh - last_battery_wh
          charge_power_w = delta_wh / dt_h if delta_wh > 0 else None
      if last_battery_wh is None or last_battery_at is None or \
         abs(battery_wh - (last_battery_wh or 0)) >= BATTERY_DELTA_TRIGGER_WH:
        last_battery_wh = battery_wh
        last_battery_at = now

    interval = TELEMETRY_ONROAD_S if onroad else TELEMETRY_OFFROAD_S
    battery_moved = (
      battery_wh is not None and last.get("battery_wh") is not None
      and abs(battery_wh - float(last["battery_wh"])) >= BATTERY_DELTA_TRIGGER_WH
    )
    # 부팅 직후엔 monotonic이 작아 'now - 0 < interval'이 참이 되어 첫 업로드가
    # 최대 10분 지연되던 문제가 있었다. 첫 회는 무조건 올린다.
    if last_upload_at is not None and now - last_upload_at < interval and not battery_moved:
      continue

    # 배터리/오도미터/외기온·용량은 CAN에서 직접 샘플링 (업로드 직전에만).
    # 차가 완전히 잠들면 CAN이 없어 빈 dict가 온다.
    sampled = sample_vehicle_can()

    # 주차 중엔 carState가 없으므로 CAN 샘플값이 배터리의 유일한 소스가 된다.
    if sampled.get("battery_wh"):
      battery_wh = sampled["battery_wh"]
      # 충전 판정도 이 값으로 다시 계산 (증가 기울기)
      if last_battery_at is not None and last_battery_wh is not None:
        dt_h = (now - last_battery_at) / 3600.0
        if dt_h > 0.02:
          delta_wh = battery_wh - last_battery_wh
          charge_power_w = delta_wh / dt_h if delta_wh > 0 else None
      if last_battery_at is None or abs(battery_wh - (last_battery_wh or 0)) >= BATTERY_DELTA_TRIGGER_WH:
        last_battery_wh, last_battery_at = battery_wh, now

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
    if battery_wh is not None:
      prev_wh = charge_ledger.get("last_wh")
      prev_at = charge_ledger.get("last_at")
      if not onroad and prev_wh is not None and prev_at is not None and battery_wh > prev_wh:
        hours = max(0.0, time.time() - float(prev_at)) / 3600.0
        add_charge_energy(charge_ledger, battery_wh - float(prev_wh), hours)
      charge_ledger["last_wh"] = battery_wh
      charge_ledger["last_at"] = time.time()
      save_charge_ledger(charge_ledger)

    vehicle = build_vehicle_block(battery_wh, charge_power_w, sampled.get("capacity_wh"))
    vehicle["charge_months"] = charge_ledger.get("months", {})
    for key in ("odometer_km", "outside_temp_c", "hv_voltage", "measured_capacity_wh",
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

    for gps_service in ("gpsLocationExternal", "gpsLocation"):
      if sm.valid.get(gps_service) and sm.seen.get(gps_service):
        gps = sm[gps_service]
        if gps.hasFix:
          payload["gps"]["latitude"] = float(gps.latitude)
          payload["gps"]["longitude"] = float(gps.longitude)
          payload["gps"]["speedMps"] = float(gps.speed)
          payload["gps"]["bearingDeg"] = float(gps.bearingDeg)
          payload["gps"]["accuracyM"] = float(gps.horizontalAccuracy)
          break

    if post_state(config, payload):
      last_upload_at = now
      last["updated_at"] = payload["updatedAt"]
      save_last_state(last)   # 마지막 측정값 유지 (주차 후에도 표시하기 위함)
      print(f"Wayon telemetry: uploaded soc={vehicle.get('soc_percent')}% "
            f"odo={vehicle.get('odometer_km')} charging={vehicle.get('charging')} "
            f"stale={vehicle.get('stale', False)}", flush=True)


if __name__ == "__main__":
  main()
