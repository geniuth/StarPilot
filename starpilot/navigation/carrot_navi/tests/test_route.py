"""Tests for route curvature deceleration.

Routes are built as synthetic geometry, since there is no recorded navi route to replay.
"""
import math

from openpilot.starpilot.navigation.carrot_navi.route import (
  NO_LIMIT,
  calculate_curvature,
  closest_point_on_segment,
  get_path_after_distance,
  gps_to_relative_xy,
  haversine,
  resample,
  route_curve_speed,
)

LAT0, LON0 = 37.5, 127.0
M_PER_DEG_LAT = 40008000.0 / 360.0


def straight_route(length_m=600.0, step_m=10.0):
  """Due north from the origin."""
  n = int(length_m / step_m)
  return [(LON0, LAT0 + i * step_m / M_PER_DEG_LAT) for i in range(n)]


def arc_route(radius_m=50.0, sweep_deg=120.0, step_deg=2.0, lead_in_m=150.0):
  """A straight approach heading north, then a constant-radius curve.

  The lead-in matters: curvature is measured over a sliding window of nine resampled
  points, so a bare arc shorter than ~90 m yields no samples at all and the route asks
  for nothing. Real routes always have road leading up to the corner.
  """
  pts = []
  m_per_deg_lon = M_PER_DEG_LAT * math.cos(math.radians(LAT0))
  step_m = 10.0
  for i in range(int(lead_in_m / step_m)):
    pts.append((LON0, LAT0 + i * step_m / M_PER_DEG_LAT))

  y0 = lead_in_m
  for i in range(int(sweep_deg / step_deg) + 1):
    th = math.radians(i * step_deg)
    x = radius_m * (1 - math.cos(th))
    y = y0 + radius_m * math.sin(th)
    pts.append((LON0 + x / m_per_deg_lon, LAT0 + y / M_PER_DEG_LAT))
  return pts


class TestGeometry:
  def test_haversine_matches_a_known_northward_step(self):
    d = haversine(LON0, LAT0, LON0, LAT0 + 1000.0 / M_PER_DEG_LAT)
    assert abs(d - 1000.0) < 5.0

  def test_closest_point_clamps_to_the_segment(self):
    assert closest_point_on_segment((0.0, 0.0), (1.0, 0.0), (-5.0, 0.0)) == (0.0, 0.0)
    assert closest_point_on_segment((0.0, 0.0), (1.0, 0.0), (5.0, 0.0)) == (1.0, 0.0)

  def test_relative_xy_puts_the_heading_on_the_first_axis(self):
    pts = [(LON0, LAT0 + 100.0 / M_PER_DEG_LAT)]
    fwd = gps_to_relative_xy(pts, (LON0, LAT0), 0.0)[0]
    assert abs(fwd[0] - 100.0) < 1.0     # 100 m straight ahead
    assert abs(fwd[1]) < 1.0

  def test_resample_is_evenly_spaced(self):
    pts = resample([(0.0, 0.0), (100.0, 0.0)], 10.0)
    assert len(pts) == 11
    for i in range(1, len(pts)):
      assert abs((pts[i][0] - pts[i - 1][0]) - 10.0) < 1e-6

  def test_resample_handles_degenerate_input(self):
    assert resample([], 10.0) == []
    assert resample([(0.0, 0.0)], 10.0) == [(0.0, 0.0)]
    assert resample([(0.0, 0.0), (0.0, 0.0)], 10.0) == [(0.0, 0.0)]

  def test_curvature_is_zero_on_a_straight_line(self):
    assert calculate_curvature((0.0, 0.0), (10.0, 0.0), (20.0, 0.0)) == 0.0

  def test_curvature_sign_follows_the_turn(self):
    left = calculate_curvature((0.0, 0.0), (10.0, 0.0), (20.0, 5.0))
    right = calculate_curvature((0.0, 0.0), (10.0, 0.0), (20.0, -5.0))
    assert left > 0 > right

  def test_tighter_turn_has_more_curvature(self):
    gentle = abs(calculate_curvature((0.0, 0.0), (10.0, 0.0), (20.0, 1.0)))
    sharp = abs(calculate_curvature((0.0, 0.0), (10.0, 0.0), (20.0, 8.0)))
    assert sharp > gentle


class TestGetPathAfterDistance:
  def test_returns_the_stretch_ahead(self):
    route = straight_route()
    path, idx, start = get_path_after_distance(0, route, (LON0, LAT0), 100.0)
    assert len(path) >= 2
    assert idx == 0

  def test_short_route_is_handled(self):
    assert get_path_after_distance(0, [], (LON0, LAT0), 100.0)[0] == []
    assert get_path_after_distance(0, [(LON0, LAT0)], (LON0, LAT0), 100.0)[0] == []


class TestRouteCurveSpeed:
  def test_no_route_asks_for_nothing(self):
    speed, _ = route_curve_speed([], (LON0, LAT0), 0.0, 60.0, 60.0, 0.8, 10.0)
    assert speed == NO_LIMIT

  def test_straight_road_does_not_slow_below_the_limit(self):
    speed, _ = route_curve_speed(straight_route(), (LON0, LAT0), 0.0, 60.0, 60.0, 0.8, 10.0)
    assert speed >= 60.0

  def test_tight_curve_slows_down(self):
    speed, _ = route_curve_speed(arc_route(radius_m=40.0), (LON0, LAT0), 0.0,
                                 80.0, 80.0, 0.8, 10.0)
    assert speed < 80.0, "a 40 m radius curve should have asked for less speed"

  def test_tighter_curve_slows_more(self):
    tight, _ = route_curve_speed(arc_route(radius_m=30.0), (LON0, LAT0), 0.0, 80.0, 80.0, 0.8, 10.0)
    wide, _ = route_curve_speed(arc_route(radius_m=200.0), (LON0, LAT0), 0.0, 80.0, 80.0, 0.8, 10.0)
    assert tight < wide

  def test_route_shorter_than_the_window_yields_nothing(self):
    # curvature needs nine resampled points at 10 m, so under ~90 m of route there is no
    # answer at all. Worth pinning: it means a stub route silently disables this source.
    speed, _ = route_curve_speed(straight_route(length_m=50.0), (LON0, LAT0), 0.0,
                                 80.0, 80.0, 0.8, 10.0)
    assert speed == NO_LIMIT

  def test_result_is_never_negative(self):
    speed, _ = route_curve_speed(arc_route(radius_m=15.0), (LON0, LAT0), 0.0, 100.0, 100.0, 0.8, 10.0)
    assert speed >= 0.0

  def test_index_advances_and_does_not_crash_when_reused(self):
    route = straight_route()
    speed, idx = route_curve_speed(route, (LON0, LAT0), 0.0, 60.0, 60.0, 0.8, 10.0)
    further = (LON0, LAT0 + 200.0 / M_PER_DEG_LAT)
    speed2, idx2 = route_curve_speed(route, further, 0.0, 60.0, 60.0, 0.8, 10.0, start_index=idx)
    assert idx2 >= 0 and speed2 >= 0.0
