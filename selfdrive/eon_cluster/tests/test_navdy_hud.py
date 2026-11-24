"""navdy_hud packet building from StarPilot messages, without cereal or a device."""
import json
from types import SimpleNamespace as NS

from openpilot.selfdrive.eon_cluster import bt_link, navdy_hud


class FakeSM:
  def __init__(self, now=100.0, navi_age=0.1, **msgs):
    self.data = msgs
    self.recv_frame = {name: (1 if name in msgs else 0) for name in navdy_hud.SERVICES}
    self.recv_time = dict.fromkeys(navdy_hud.SERVICES, now - navi_age)

  def __getitem__(self, name):
    return self.data[name]


def _xy(n=33, y=0.0):
  return NS(x=[float(i * 2) for i in range(n)], y=[y] * n)


def _lead(d, y=0.0, v=-1.0, a=0.0, status=True):
  return NS(status=status, dRel=d, yRel=y, vRel=v, aLeadK=a)


def _ts(**kw):
  base = dict(isGreenLightOn=False, isLeftLightOn=False, isRedLightOn=False,
              greenLightRemainTime=0, leftLightRemainTime=0, redLightRemainTime=0, distance=0)
  base.update(kw)
  return NS(**base)


def _navi(**kw):
  base = dict(active=1, camType=0, camLimitSpeed=0, camLimitSpeedLeftDist=0, sectionLimitSpeed=0,
              sectionLeftDist=0, roadLimitSpeed=0, ts=_ts())
  base.update(kw)
  return NS(**base)


def _sm(navi=None, navi_age=0.1, plan=None, car=None):
  msgs = dict(
    carState=car or NS(vEgo=20.0, vEgoCluster=20.0, vCruise=90.0, vCruiseCluster=90.0, leftBlindspot=True,
                       rightBlindspot=False, leftBlinker=False, rightBlinker=True),
    selfdriveState=NS(enabled=True),
    deviceState=NS(started=True),
    modelV2=NS(position=_xy(), laneLines=[_xy(y=y) for y in (-5.4, -1.8, 1.8, 5.4)], laneLineProbs=[0.2, 0.9, 0.9, 0.3]),
    radarState=NS(leadOne=_lead(30.0), leadTwo=_lead(60.0, status=False)),
    starpilotRadarState=NS(leadLeft=_lead(25.0, y=3.5), leadRight=_lead(30.5, y=0.3)),
    starpilotPlan=plan or NS(slcSpeedLimit=80 / 3.6, cscSpeed=0.0, cscControllingSpeed=False, vCruise=25.0),
  )
  if navi is not None:
    msgs["naviData"] = navi
  return FakeSM(navi_age=navi_age, **msgs)


def test_basic_packet_has_every_key_the_app_reads():
  packet = navdy_hud.build_packet(_sm(), now=100.0)
  assert set(bt_link.KEEP_KEYS) <= set(packet)
  assert packet["speed"] == 72 and packet["set"] == 90 and packet["limit"] == 80
  assert packet["enabled"] is True
  assert packet["leftBsd"] and packet["rightBlinker"]
  assert len(packet["lanes"]) == 4 and packet["lanes"][1]["c"] == 0.9
  assert packet["lead"]["d"] == 30.0 and packet["lead2"] is None
  # The right "adjacent" lead sits on top of leadOne, so it is merged away.
  assert [o["y"] for o in packet["others"]] == [3.5]
  assert packet["camera"] == 0 and packet["tmapSignal"] is None


def test_slim_packet_is_json_and_small():
  blob = json.dumps(bt_link.slim_packet(navdy_hud.build_packet(_sm(), now=100.0)), separators=(",", ":"))
  assert len(blob) < 1500


def test_unset_cruise_and_no_slowdown_fold_to_zero():
  car = NS(vEgo=10.0, vEgoCluster=10.0, vCruise=255.0, vCruiseCluster=255.0,
           leftBlindspot=False, rightBlindspot=False, leftBlinker=False, rightBlinker=False)
  packet = navdy_hud.build_packet(_sm(car=car), now=100.0)
  assert packet["set"] == 0 and packet["desiredSpeed"] == 0


def test_curve_and_target_speed():
  plan = NS(slcSpeedLimit=0.0, cscSpeed=50 / 3.6, cscControllingSpeed=True, vCruise=50 / 3.6)
  packet = navdy_hud.build_packet(_sm(plan=plan), now=100.0)
  assert packet["vTurnSpeed"] == 50 and packet["desiredSpeed"] == 50 and packet["limit"] == 0


def test_nda_camera_section_bump_and_road_limit():
  packet = navdy_hud.build_packet(_sm(_navi(camType=1, camLimitSpeed=60, camLimitSpeedLeftDist=320, roadLimitSpeed=70)), now=100.0)
  assert (packet["camera"], packet["cameraDist"], packet["cameraSection"], packet["limit"]) == (60, 320, False, 70)

  packet = navdy_hud.build_packet(_sm(_navi(sectionLimitSpeed=100, sectionLeftDist=2500)), now=100.0)
  assert (packet["camera"], packet["cameraDist"], packet["cameraSection"]) == (100, 2500, True)

  packet = navdy_hud.build_packet(_sm(_navi(camType=22, camLimitSpeed=30, camLimitSpeedLeftDist=40)), now=100.0)
  assert packet["bumpDist"] == 40 and packet["camera"] == 0


def test_traffic_signal_uses_lit_lamp():
  navi = _navi(ts=_ts(isLeftLightOn=True, leftLightRemainTime=12, distance=150))
  assert navdy_hud.build_packet(_sm(navi), now=100.0)["tmapSignal"] == {"l": "left", "s": 12, "d": 150}


def test_stale_navi_is_ignored():
  navi = _navi(camType=1, camLimitSpeed=60, camLimitSpeedLeftDist=320, roadLimitSpeed=70)
  packet = navdy_hud.build_packet(_sm(navi, navi_age=navdy_hud.NAVI_TIMEOUT_S + 1), now=100.0)
  assert packet["camera"] == 0 and packet["limit"] == 80


def test_read_config_defaults_and_overrides(tmp_path):
  path = tmp_path / "navdy_hud.json"
  assert navdy_hud.read_config(str(path)) == {"enabled": True, "bt_mac": ""}
  path.write_text(json.dumps({"enabled": False, "bt_mac": " AA:BB:CC:DD:EE:FF "}))
  assert navdy_hud.read_config(str(path)) == {"enabled": False, "bt_mac": "AA:BB:CC:DD:EE:FF"}
  path.write_text("{broken")
  assert navdy_hud.read_config(str(path))["enabled"] is True
