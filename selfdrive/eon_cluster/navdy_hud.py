"""Navdy HUD over Bluetooth for StarPilot.

The Navdy has no Wi-Fi userspace, so our Android app on it (geniuth/navdy-hud, CommaHUD) listens
on RFCOMM and this process connects to it as a client through bt_link. Only the keys the app
actually draws are sent (bt_link.KEEP_KEYS); the wire format matches the carrot sender
(geniuth/openpilot feature/nmirror-bridge selfdrive/eon_cluster/remote_hud.py) so the same APK works.

Data sources here are StarPilot's own: carState, selfdriveState, modelV2, radarState,
starpilotRadarState (adjacent leads), starpilotPlan (speed limit, curve and final target speed)
and naviData (NDA cameras, section enforcement, traffic signal).

Pairing: the Navdy must be bonded with the comma first (StarPilot Bluetooth settings), and the
Bluetooth radio must be up (/sys/class/bluetooth). With no MAC configured every bonded device is
probed over SDP for the CommaHUD service.

Settings live in /data/navdy_hud.json (optional, no rebuild of the Params registry needed):
  {"enabled": true, "bt_mac": "AA:BB:CC:DD:EE:FF"}
"""
import json
import math
import os
import signal
import time

from cereal import messaging
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.eon_cluster import bt_link

CONFIG_PATH = os.getenv("NAVDY_HUD_CONFIG", "/data/navdy_hud.json")
TELEMETRY_HZ = 10
IDLE_HZ = 2
CONFIG_REFRESH_S = 5.0
BT_RECHECK_S = 30.0
NAVI_TIMEOUT_S = 2.0
MS_TO_KPH = 3.6
SPEED_BUMP_CAM_TYPE = 22
TRAFFIC_LAMPS = (("red", "isRedLightOn", "redLightRemainTime"),
                 ("left", "isLeftLightOn", "leftLightRemainTime"),
                 ("green", "isGreenLightOn", "greenLightRemainTime"))
# carState is the only busy service here. msgq evicts every subscriber of a service once more than 15
# readers attach, so this process must not add readers to carState/deviceState beyond this one.
SERVICES = ["carState", "selfdriveState", "modelV2", "radarState",
            "starpilotPlan", "starpilotRadarState", "naviData"]


def _field(obj, name, default=0):
  try:
    value = getattr(obj, name)
  except Exception:
    return default
  return default if value is None else value


def _finite(value, default=0.0):
  try:
    value = float(value)
  except (TypeError, ValueError):
    return default
  return value if math.isfinite(value) else default


def _kph(value_ms):
  return max(0, int(round(_finite(value_ms) * MS_TO_KPH)))


def read_config(path=CONFIG_PATH):
  config = {"enabled": True, "bt_mac": ""}
  try:
    with open(path, encoding="utf-8") as f:
      loaded = json.load(f)
    if isinstance(loaded, dict):
      config["enabled"] = bool(loaded.get("enabled", True))
      config["bt_mac"] = str(loaded.get("bt_mac", "") or "").strip()
  except FileNotFoundError:
    pass
  except (OSError, ValueError) as e:
    cloudlog.warning(f"navdy_hud: bad config {path}: {e}")
  return config


def _line_points(position, limit=33):
  xs = list(_field(position, "x", []) or [])
  ys = list(_field(position, "y", []) or [])
  count = min(len(xs), len(ys), limit)
  if count < 2:
    return []
  return [[round(_finite(xs[i]), 2), round(_finite(ys[i]), 2)] for i in range(count)]


def _model_lanes(model):
  """modelV2 laneLines with fixed slots (0 outer-left, 1/2 ego, 3 outer-right)."""
  lines = list(_field(model, "laneLines", []) or [])
  probs = list(_field(model, "laneLineProbs", []) or [])
  result = []
  for index in range(min(4, max(len(lines), len(probs)))):
    points = _line_points(lines[index]) if index < len(lines) else []
    prob = probs[index] if index < len(probs) else 0.0
    if len(points) < 2:
      result.append({"p": [], "c": 0.0})
      continue
    result.append({"p": points, "c": round(max(0.0, min(1.0, _finite(prob))), 2)})
  return result


def _vehicle(lead):
  if not bool(_field(lead, "status", False)):
    return None
  return {
    "d": round(max(0.0, _finite(_field(lead, "dRel", 0.0))), 1),
    "y": round(_finite(_field(lead, "yRel", 0.0)), 2),
    "v": round(_finite(_field(lead, "vRel", 0.0)) * MS_TO_KPH, 1),
    # Lead acceleration: a strongly negative value lights the brake lamps in the app.
    "a": round(_finite(_field(lead, "aLeadK", 0.0)), 2),
  }


def _others(starpilot_radar, taken, limit=8):
  """Adjacent-lane leads. One car can show up as several radar points, so merge within a car's size."""
  found = []
  for name in ("leadLeft", "leadRight"):
    vehicle = _vehicle(_field(starpilot_radar, name, None))
    if vehicle is None:
      continue
    if any(abs(vehicle["d"] - d) < 4.5 and abs(vehicle["y"] - y) < 1.4 for d, y in taken):
      continue
    taken.append((vehicle["d"], vehicle["y"]))
    found.append(vehicle)
  found.sort(key=lambda v: v["d"])
  return found[:limit]


def _navi_recent(sm, now):
  try:
    return sm.recv_frame["naviData"] > 0 and now - sm.recv_time["naviData"] < NAVI_TIMEOUT_S
  except (KeyError, AttributeError):
    return False


def navi_fields(navi, recent):
  """Camera / section / speed bump / road limit / traffic signal keys from naviData."""
  out = {"camera": 0, "cameraDist": 0, "cameraSection": False, "bumpDist": 0, "tmapSignal": None, "naviLimit": 0}
  if navi is None or not recent:
    return out

  cam_type = int(_finite(_field(navi, "camType", 0)))
  cam_limit = int(_finite(_field(navi, "camLimitSpeed", 0)))
  cam_dist = int(_finite(_field(navi, "camLimitSpeedLeftDist", 0)))
  section_limit = int(_finite(_field(navi, "sectionLimitSpeed", 0)))
  section_dist = int(_finite(_field(navi, "sectionLeftDist", 0)))

  if cam_type == SPEED_BUMP_CAM_TYPE:
    out["bumpDist"] = max(0, cam_dist)
  elif cam_limit > 0 and cam_dist > 0:
    out["camera"], out["cameraDist"] = cam_limit, cam_dist
  elif section_limit > 0 and section_dist > 0:
    out["camera"], out["cameraDist"], out["cameraSection"] = section_limit, section_dist, True

  out["naviLimit"] = max(0, int(_finite(_field(navi, "roadLimitSpeed", 0))))

  ts = _field(navi, "ts", None)
  if ts is not None:
    distance = max(0, int(_finite(_field(ts, "distance", 0))))
    for lamp, on_key, remain_key in TRAFFIC_LAMPS:
      remain = int(_finite(_field(ts, remain_key, 0)))
      if bool(_field(ts, on_key, False)) and remain > 0:
        out["tmapSignal"] = {"l": lamp, "s": remain, "d": distance}
        break
  return out


def build_packet(sm, now=None):
  now = time.monotonic() if now is None else now
  car = sm["carState"]
  plan = sm["starpilotPlan"]
  radar = sm["radarState"]

  set_speed = max(0, int(round(_finite(_field(car, "vCruiseCluster", 0.0)) or _finite(_field(car, "vCruise", 0.0)))))
  if set_speed >= 250:  # 255 = cruise not set
    set_speed = 0

  navi = navi_fields(sm["naviData"] if "naviData" in sm.data else None, _navi_recent(sm, now))
  limit = navi.pop("naviLimit") or _kph(_field(plan, "slcSpeedLimit", 0.0))

  curve = _kph(_field(plan, "cscSpeed", 0.0)) if bool(_field(plan, "cscControllingSpeed", False)) else 0
  desired = _kph(_field(plan, "vCruise", 0.0))
  if set_speed <= 0 or desired >= set_speed - 1:
    desired = 0  # 0 = nothing slowing us below the set speed, the app hides it

  lead = _vehicle(_field(radar, "leadOne", None))
  lead2 = _vehicle(_field(radar, "leadTwo", None))
  taken = [(v["d"], v["y"]) for v in (lead, lead2) if v]

  packet = {
    "v": 6,
    "speed": _kph(_field(car, "vEgoCluster", _field(car, "vEgo", 0.0))),
    "set": set_speed,
    "limit": limit,
    "enabled": bool(_field(sm["selfdriveState"], "enabled", False)),
    "lanes": _model_lanes(sm["modelV2"]),
    "path": _line_points(_field(sm["modelV2"], "position", None)),
    "lead": lead,
    "lead2": lead2,
    "others": _others(sm["starpilotRadarState"], taken),
    # Lane line type is not reported by StarPilot's carState; -1 = unknown.
    "laneL": -1,
    "laneR": -1,
    "leftBsd": bool(_field(car, "leftBlindspot", False)),
    "rightBsd": bool(_field(car, "rightBlindspot", False)),
    "leftBlinker": bool(_field(car, "leftBlinker", False)),
    "rightBlinker": bool(_field(car, "rightBlinker", False)),
    "vTurnSpeed": curve if 0 < curve < 250 else 0,
    "desiredSpeed": desired,
    "goTime": 0,
  }
  packet.update(navi)
  return packet


def main():
  running = [True]
  signal.signal(signal.SIGINT, lambda *_: running.__setitem__(0, False))
  signal.signal(signal.SIGTERM, lambda *_: running.__setitem__(0, False))

  sm = messaging.SubMaster(SERVICES)
  params = Params()
  onroad = False
  next_onroad_check = 0.0
  config = read_config()
  next_config = time.monotonic() + CONFIG_REFRESH_S
  link = None
  next_bt_check = 0.0
  send_errors = 0

  while running[0]:
    started_at = time.monotonic()
    if started_at >= next_config:
      new_config = read_config()
      if link is not None and new_config["bt_mac"] != config["bt_mac"]:
        link.close()
        link = None
        next_bt_check = 0.0
      config = new_config
      next_config = started_at + CONFIG_REFRESH_S

    # The radio can come up later (StarPilot Bluetooth toggle), so keep checking until it does.
    if link is None and config["enabled"] and started_at >= next_bt_check:
      next_bt_check = started_at + BT_RECHECK_S
      if bt_link.available():
        try:
          link = bt_link.BluetoothLink(config["bt_mac"])
          cloudlog.event("navdy_hud_bt_ready", mac=config["bt_mac"] or "bonded")
        except Exception as e:
          cloudlog.info(f"navdy_hud: bluetooth link unavailable: {e}")

    sm.update(0)
    if started_at >= next_onroad_check:
      try:
        onroad = params.get_bool("IsOnroad")
      except Exception:
        onroad = False
      next_onroad_check = started_at + 1.0
    active = config["enabled"] and onroad
    if link is not None:
      link.set_active(active)
      if active:
        try:
          link.send_packet(build_packet(sm, started_at))
          send_errors = 0
        except Exception as e:
          send_errors += 1
          if send_errors == 1 or send_errors % 100 == 0:
            cloudlog.exception(f"navdy_hud: packet build failed ({send_errors}): {e}")

    rate = TELEMETRY_HZ if active else IDLE_HZ
    time.sleep(max(0.0, 1.0 / rate - (time.monotonic() - started_at)))

  if link is not None:
    link.close()


if __name__ == "__main__":
  main()
