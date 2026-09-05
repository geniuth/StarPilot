"""Tests for the Carrot Navi deceleration decisions.

None of this has run on a car, so the tests pin the behaviour that matters: that each
source decelerates when it should, stays quiet when it should not, and that the gates
carrot put in place (control mode, road category, section handling) actually hold.
"""
import math
from types import SimpleNamespace

from openpilot.starpilot.navigation.carrot_navi.speed import (
  NO_LIMIT,
  NaviSpeedConfig,
  atc_target,
  calculate_current_speed,
  desired_speed,
  expire_restriction,
  route_target,
  sdi_target,
  select_sdi_restriction,
)


def navi(**kw):
  base = dict(sdiType=-1, sdiSpeedLimit=0.0, sdiDistance=0.0,
              sdiBlockType=-1, sdiBlockSpeed=0.0, sdiBlockDistance=0.0,
              sdiPlusType=-1, sdiPlusSpeedLimit=0.0, sdiPlusDistance=0.0,
              roadCategory=2, tbtTurnType=-1, tbtDistance=0.0,
              tbtTurnTypeNext=-1, tbtDistanceNext=0.0, tbtNextRoadWidth=0.0,
              routeCurveSpeed=0.0)
  base.update(kw)
  return SimpleNamespace(**base)


def cfg(**kw):
  return NaviSpeedConfig(**kw)


class TestCalculateCurrentSpeed:
  def test_at_the_point_returns_the_limit(self):
    assert calculate_current_speed(0.0, 50.0, 10.0, 0.8) == 50.0

  def test_far_away_allows_more_speed(self):
    # both distances are outside the reserve, so both still allow more than the limit
    far = calculate_current_speed(500.0, 50.0, 10.0, 0.8)
    near = calculate_current_speed(300.0, 50.0, 10.0, 0.8)
    assert far > near > 50.0

  def test_inside_the_reserve_is_already_the_limit(self):
    # 10s at 50 km/h is ~139 m, so at 100 m we should already be down to the limit
    assert calculate_current_speed(100.0, 50.0, 10.0, 0.8) == 50.0

  def test_never_below_the_limit(self):
    for dist in (0.0, 1.0, 10.0, 1000.0):
      assert calculate_current_speed(dist, 50.0, 10.0, 0.8) >= 50.0

  def test_reserved_time_arrives_slowed(self):
    # within ctrl_end seconds of travel at the limit, the target is already the limit
    reserve_m = 50.0 / 3.6 * 10.0
    assert calculate_current_speed(reserve_m, 50.0, 10.0, 0.8) == 50.0

  def test_matches_the_kinematic_formula(self):
    dist, limit, ctrl_end, decel = 300.0, 50.0, 10.0, 0.8
    safe = limit / 3.6
    expected = math.sqrt(safe ** 2 + 2 * decel * (dist - safe * ctrl_end)) * 3.6
    assert abs(calculate_current_speed(dist, limit, ctrl_end, decel) - expected) < 1e-9

  def test_capped(self):
    assert calculate_current_speed(1e6, 50.0, 10.0, 0.8) == NO_LIMIT


class TestSelectSdiRestriction:
  def test_camera(self):
    t, limit, dist = select_sdi_restriction(navi(sdiType=1, sdiSpeedLimit=50, sdiDistance=300), cfg())
    assert (t, limit, dist) == (1, 50.0, 300.0)

  def test_safety_factor_scales_the_limit(self):
    _, limit, _ = select_sdi_restriction(navi(sdiType=1, sdiSpeedLimit=50, sdiDistance=300),
                                         cfg(safety_factor=0.9))
    assert limit == 45.0

  def test_control_mode_off_disables_everything(self):
    assert select_sdi_restriction(navi(sdiType=1, sdiSpeedLimit=50, sdiDistance=300),
                                  cfg(ctrl_mode=0)) == (-1, 0.0, 0.0)

  def test_section_overrides_the_point_camera(self):
    # inside average-speed enforcement: the block distance is what matters
    t, limit, dist = select_sdi_restriction(
      navi(sdiType=1, sdiSpeedLimit=80, sdiDistance=300, sdiBlockType=2, sdiBlockDistance=4200),
      cfg())
    assert t == 4
    assert (limit, dist) == (80.0, 4200.0)

  def test_section_end_also_counts(self):
    t, _, _ = select_sdi_restriction(
      navi(sdiType=1, sdiSpeedLimit=80, sdiDistance=300, sdiBlockType=3, sdiBlockDistance=100),
      cfg())
    assert t == 4

  def test_mobile_camera_needs_mode_three(self):
    n = navi(sdiType=7, sdiSpeedLimit=60, sdiDistance=200)
    assert select_sdi_restriction(n, cfg(ctrl_mode=2)) == (-1, 0.0, 0.0)
    assert select_sdi_restriction(n, cfg(ctrl_mode=3))[0] == 7

  def test_bump_needs_mode_two_and_a_local_road(self):
    n = navi(sdiType=22, sdiDistance=80, roadCategory=2)
    assert select_sdi_restriction(n, cfg(ctrl_mode=1)) == (-1, 0.0, 0.0)
    t, limit, dist = select_sdi_restriction(n, cfg(ctrl_mode=2, bump_speed=25))
    assert (t, limit, dist) == (22, 25.0, 80.0)

  def test_no_bump_on_the_motorway(self):
    # this is the gate the 7714 doc says silently kills bumps when road_category is missing
    n = navi(sdiType=22, sdiDistance=80, roadCategory=1)
    assert select_sdi_restriction(n, cfg(ctrl_mode=2)) == (-1, 0.0, 0.0)

  def test_bump_from_the_secondary_slot(self):
    n = navi(sdiPlusType=22, sdiPlusDistance=60, roadCategory=3)
    t, _, dist = select_sdi_restriction(n, cfg(ctrl_mode=2))
    assert (t, dist) == (22, 60.0)


class TestExpireRestriction:
  def test_point_camera_expires_at_zero(self):
    assert expire_restriction(1, 50.0, 0.0) == (-1, 0.0, 0.0)

  def test_police_lingers_past_the_point(self):
    assert expire_restriction(100, 50.0, -100.0)[0] == 100
    assert expire_restriction(100, 50.0, -300.0) == (-1, 0.0, 0.0)


class TestSdiTarget:
  def test_camera_eases_down(self):
    speed, source = sdi_target(1, 50.0, 300.0, cfg())
    assert source == "cam"
    assert speed > 50.0

  def test_section_holds_the_limit_flat(self):
    # the whole point of section enforcement: no easing, just sit at the limit
    for dist in (100.0, 2000.0, 8000.0):
      speed, source = sdi_target(4, 80.0, dist, cfg())
      assert (speed, source) == (80.0, "section")

  def test_bump_uses_its_own_reserve_time(self):
    slow = sdi_target(22, 25.0, 100.0, cfg(bump_time=1.0))[0]
    lenient = sdi_target(22, 25.0, 100.0, cfg(bump_time=10.0))[0]
    assert slow > lenient >= 25.0

  def test_nothing_when_expired(self):
    assert sdi_target(-1, 0.0, 0.0, cfg()) == (NO_LIMIT, "none")


class TestAtcTarget:
  def test_off_unless_speed_mode_enabled(self):
    assert atc_target(1, 200.0, 60.0, 10.0, cfg(turn_speed_mode=0)) == (NO_LIMIT, "none")

  def test_turn_slows_to_the_turn_speed(self):
    speed, source = atc_target(1, 200.0, 60.0, 10.0, cfg(turn_speed_mode=2, turn_speed=20))
    assert source == "atc"
    assert speed >= 20.0

  def test_fork_only_slows_to_the_road_limit(self):
    turn = atc_target(1, 150.0, 60.0, 10.0, cfg(turn_speed_mode=2, turn_speed=20))[0]
    fork = atc_target(3, 150.0, 60.0, 10.0, cfg(turn_speed_mode=2, turn_speed=20))[0]
    assert fork > turn

  def test_unknown_manoeuvre_is_quiet(self):
    assert atc_target(-1, 100.0, 60.0, 10.0, cfg(turn_speed_mode=2)) == (NO_LIMIT, "none")

  def test_passed_turn_is_quiet(self):
    assert atc_target(1, 0.0, 60.0, 10.0, cfg(turn_speed_mode=2)) == (NO_LIMIT, "none")


class TestRouteTarget:
  def test_off_by_default(self):
    assert route_target(40.0, 100.0, cfg(turn_speed_mode=0)) == (NO_LIMIT, "none")

  def test_mode_two_only_near_a_manoeuvre(self):
    assert route_target(40.0, 100.0, cfg(turn_speed_mode=2))[1] == "route"
    assert route_target(40.0, 900.0, cfg(turn_speed_mode=2)) == (NO_LIMIT, "none")

  def test_mode_three_applies_everywhere(self):
    assert route_target(40.0, 900.0, cfg(turn_speed_mode=3))[1] == "route"

  def test_never_below_the_lower_limit(self):
    speed, _ = route_target(5.0, 0.0, cfg(turn_speed_mode=3, curve_lower_limit=30.0))
    assert speed == 30.0


class TestDesiredSpeed:
  def test_quiet_with_no_data(self):
    assert desired_speed(navi(), cfg()) == (NO_LIMIT, "none")

  def test_camera_binds(self):
    speed, source = desired_speed(navi(sdiType=1, sdiSpeedLimit=50, sdiDistance=200), cfg(),
                                  road_limit_kph=100.0)
    assert source == "cam"
    assert speed < 100.0

  def test_road_limit_binds_when_nothing_else_does(self):
    assert desired_speed(navi(), cfg(), road_limit_kph=60.0) == (60.0, "road")

  def test_lowest_source_wins(self):
    # a bump close by should beat a distant camera
    n = navi(sdiType=22, sdiDistance=30, roadCategory=2)
    speed, source = desired_speed(n, cfg(ctrl_mode=2, bump_speed=25), road_limit_kph=80.0)
    assert source == "bump"
    assert speed < 80.0

  def test_section_wins_over_road_limit(self):
    n = navi(sdiType=1, sdiSpeedLimit=80, sdiDistance=300, sdiBlockType=2, sdiBlockDistance=5000)
    speed, source = desired_speed(n, cfg(), road_limit_kph=110.0)
    assert (speed, source) == (80.0, "section")
