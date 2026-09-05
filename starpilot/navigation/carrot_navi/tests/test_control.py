"""Tests for the bridge between carrotNaviState and the cruise target.

These use a stand-in for the cereal message so they run without capnp, and they pin the
property that matters most: with no navi connected the feature contributes nothing, so a
car that never runs a navi app drives exactly as it did before.
"""
from types import SimpleNamespace

from openpilot.starpilot.navigation.carrot_navi.control import get_carrot_navi_target
from openpilot.starpilot.navigation.carrot_navi.speed import NaviSpeedConfig


class FakeSM:
  def __init__(self, state, valid=True):
    self._state = state
    self.valid = {"carrotNaviState": valid}

  def __getitem__(self, key):
    assert key == "carrotNaviState"
    return self._state


def state(**kw):
  base = dict(connected=True, active=3, offRoute=False, roadLimitSpeed=0.0, roadCategory=2,
              sdiType=-1, sdiSpeedLimit=0.0, sdiDistance=0.0,
              sdiBlockType=-1, sdiBlockSpeed=0.0, sdiBlockDistance=0.0,
              sdiPlusType=-1, sdiPlusSpeedLimit=0.0, sdiPlusDistance=0.0,
              tbtTurnType=-1, tbtDistance=0.0, tbtTurnTypeNext=-1, tbtDistanceNext=0.0,
              tbtNextRoadWidth=0.0, routeCurveSpeed=0.0)
  base.update(kw)
  return SimpleNamespace(**base)


class TestQuietWhenItShouldBe:
  def test_no_config(self):
    assert get_carrot_navi_target(FakeSM(state()), None) == (0.0, "none")

  def test_service_not_valid(self):
    assert get_carrot_navi_target(FakeSM(state(), valid=False), NaviSpeedConfig()) == (0.0, "none")

  def test_not_connected(self):
    # the case on this device today: the receiver runs, no navi app ever connects
    sm = FakeSM(state(connected=False, active=0))
    assert get_carrot_navi_target(sm, NaviSpeedConfig()) == (0.0, "none")

  def test_connected_but_inactive(self):
    assert get_carrot_navi_target(FakeSM(state(active=0)), NaviSpeedConfig()) == (0.0, "none")

  def test_off_route(self):
    sm = FakeSM(state(offRoute=True, sdiType=1, sdiSpeedLimit=50, sdiDistance=200))
    assert get_carrot_navi_target(sm, NaviSpeedConfig()) == (0.0, "none")

  def test_nothing_to_say(self):
    assert get_carrot_navi_target(FakeSM(state()), NaviSpeedConfig()) == (0.0, "none")


class TestTargets:
  def test_camera_produces_a_target_in_ms(self):
    sm = FakeSM(state(sdiType=1, sdiSpeedLimit=50, sdiDistance=200, roadLimitSpeed=100))
    target, source = get_carrot_navi_target(sm, NaviSpeedConfig())
    assert source == "cam"
    # returned in m/s, and below the road limit it is easing us down from
    assert 0 < target < 100 / 3.6

  def test_section_holds_the_limit(self):
    sm = FakeSM(state(sdiType=1, sdiSpeedLimit=80, sdiDistance=300,
                      sdiBlockType=2, sdiBlockDistance=5000, roadLimitSpeed=110))
    target, source = get_carrot_navi_target(sm, NaviSpeedConfig())
    assert source == "section"
    assert abs(target - 80 / 3.6) < 1e-6

  def test_bump(self):
    sm = FakeSM(state(sdiType=22, sdiDistance=60, roadCategory=3))
    target, source = get_carrot_navi_target(sm, NaviSpeedConfig(ctrl_mode=2, bump_speed=25))
    assert source == "bump"
    assert target > 0

  def test_route_curvature(self):
    sm = FakeSM(state(routeCurveSpeed=45.0, tbtDistance=100.0, roadLimitSpeed=90))
    target, source = get_carrot_navi_target(sm, NaviSpeedConfig(turn_speed_mode=3,
                                                               curve_lower_limit=30))
    assert source == "route"
    assert abs(target - 45 / 3.6) < 1e-6

  def test_atc_turn(self):
    sm = FakeSM(state(tbtTurnType=1, tbtDistance=150.0, roadLimitSpeed=80))
    target, source = get_carrot_navi_target(sm, NaviSpeedConfig(turn_speed_mode=2, turn_speed=20))
    assert source == "atc"
    assert target > 0

  def test_absurdly_low_targets_are_refused(self):
    # a stop manoeuvre asks for 1 km/h; bringing the car to a halt is not this feature's job
    sm = FakeSM(state(tbtTurnType=7, tbtDistance=5.0, roadLimitSpeed=60))
    target, _ = get_carrot_navi_target(sm, NaviSpeedConfig(turn_speed_mode=2))
    assert target == 0.0
