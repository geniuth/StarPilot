"""Tests for both Carrot Navi wire formats.

The payloads here are the ones documented in
docs/carrot_navi_7713_7714_deceleration.md, so these double as a check that the decoder
still matches the schema that document pins down.
"""
from openpilot.starpilot.navigation.carrot_navi.protocol import (
  CarrotNaviV2,
  NaviData,
  flatten_rgdata,
  normalise_road_limit,
  parse_7713,
  parse_7713_route,
)

DOC_7713 = {
  "timestamp_ms": 1710000000000,
  "rgdata": {
    "nRoadLimitSpeed": 50,
    "nSdiType": 1,
    "nSdiSpeedLimit": 50,
    "nSdiSection": -1,
    "nSdiDist": 420,
    "nSdiBlockType": -1,
    "nSdiBlockSpeed": 0,
    "nSdiBlockDist": 0,
    "nSdiPlusType": 22,
    "nSdiPlusSpeedLimit": 0,
    "nSdiPlusDist": 93,
    "nSdiPlusBlockType": -1,
    "nSdiPlusBlockSpeed": 0,
    "nSdiPlusBlockDist": 0,
    "roadcate": 8,
  },
}

DOC_7714_SPEED = {
  "type": "item_update",
  "protocol_version": 2,
  "session_id": "0123456789abcdef",
  "manifest_revision": 1,
  "schema_version": 1,
  "kind": "json",
  "name": "speed",
  "stream_handle": 6,
  "sequence": 101,
  "present": True,
  "value": {
    "current_kph": 48.0,
    "road_limit_kph": 50,
    "sdi": {"type": 1, "distance_m": 420, "speed_limit_kph": 50, "section_type": -1,
            "block_type": -1, "block_speed_kph": 0, "block_distance_m": 0},
    "sdi_secondary": {"type": 22, "distance_m": 93, "speed_limit_kph": 0, "section_type": -1,
                      "block_type": -1, "block_speed_kph": 0, "block_distance_m": 0},
    "section": {"active": False, "speed_limit_kph": 0, "off_route": False},
  },
}


class TestRoadLimit:
  def test_normal(self):
    assert normalise_road_limit(50.0) == 50.0

  def test_legacy_encoding_above_200(self):
    assert normalise_road_limit(250.0) == 50.0

  def test_zero_falls_back_to_thirty(self):
    # matches the nRoadLimitSpeed=30 seen in this device's own logs with no navi connected
    assert normalise_road_limit(0.0) == 30.0


class TestFlatten:
  def test_nested_groups_are_flattened(self):
    flat = flatten_rgdata({"sdi": {"nSdiType": 1}, "guidance": {"nTBTDist": 100},
                           "nRoadLimitSpeed": 50})
    assert flat["nSdiType"] == 1 and flat["nTBTDist"] == 100 and flat["nRoadLimitSpeed"] == 50

  def test_top_level_wins(self):
    flat = flatten_rgdata({"sdi": {"nSdiType": 1}, "nSdiType": 7, "nRoadLimitSpeed": 50})
    assert flat["nSdiType"] == 7


class TestParse7713:
  def test_documented_payload(self):
    d = parse_7713(DOC_7713)
    assert d.connected and d.source == "7713"
    assert d.road_limit_kph == 50.0
    assert (d.sdi_type, d.sdi_speed_limit, d.sdi_distance) == (1, 50.0, 420.0)
    assert (d.sdi_plus_type, d.sdi_plus_distance) == (22, 93.0)
    assert d.road_category == 8

  def test_missing_gate_is_a_keepalive(self):
    # without nRoadLimitSpeed carrot does not refresh, and neither may we, or a keepalive
    # would wipe a live camera warning
    assert parse_7713({"rgdata": {"nSdiType": 1}}) is None

  def test_no_rgdata(self):
    assert parse_7713({"timestamp_ms": 1}) is None

  def test_route_survives_an_update(self):
    prev = NaviData(route_points=[(127.0, 37.0)], route_sequence=3)
    d = parse_7713(DOC_7713, previous=prev)
    assert d.route_points == [(127.0, 37.0)]

  def test_garbage_values_do_not_raise(self):
    d = parse_7713({"rgdata": {"nRoadLimitSpeed": 50, "nSdiType": "x", "nSdiDist": None}})
    assert d.sdi_type == -1 and d.sdi_distance == 0.0


class TestParse7713Route:
  def test_dict_points(self):
    pts = parse_7713_route({"route": [{"latitude": 37.5, "longitude": 127.0}]})
    assert pts == [(127.0, 37.5)]

  def test_pair_points_and_vrtx_alias(self):
    assert parse_7713_route({"vrtx": [[37.5, 127.0]]}) == [(127.0, 37.5)]


class TestCarrotNaviV2:
  def test_documented_speed_item(self):
    c = CarrotNaviV2()
    d = c.update(DOC_7714_SPEED)
    assert d.connected and d.session_id == "0123456789abcdef"
    assert d.road_limit_kph == 50.0
    assert (d.sdi_type, d.sdi_speed_limit, d.sdi_distance) == (1, 50.0, 420.0)
    assert (d.sdi_plus_type, d.sdi_plus_distance) == (22, 93.0)

  def test_road_category_comes_from_lane_current(self):
    # the doc's headline finding: without this item bumps never fire
    c = CarrotNaviV2()
    c.update(DOC_7714_SPEED)
    assert c.data.road_category == 0
    c.update({"type": "item_update", "session_id": "0123456789abcdef", "name": "lane_current",
              "sequence": 5, "present": True, "value": {"road_category": 8}})
    assert c.data.road_category == 8

  def test_repeated_sequence_is_a_heartbeat(self):
    c = CarrotNaviV2()
    c.update(DOC_7714_SPEED)
    stale = dict(DOC_7714_SPEED)
    stale["value"] = dict(DOC_7714_SPEED["value"], road_limit_kph=90)
    c.update(stale)  # same sequence
    assert c.data.road_limit_kph == 50.0, "heartbeat overwrote locally tracked state"

  def test_new_sequence_applies(self):
    c = CarrotNaviV2()
    c.update(DOC_7714_SPEED)
    nxt = dict(DOC_7714_SPEED, sequence=102)
    nxt["value"] = dict(DOC_7714_SPEED["value"], road_limit_kph=90)
    c.update(nxt)
    assert c.data.road_limit_kph == 90.0

  def test_out_of_range_road_limit_is_no_limit(self):
    c = CarrotNaviV2()
    e = dict(DOC_7714_SPEED, sequence=1)
    e["value"] = dict(DOC_7714_SPEED["value"], road_limit_kph=250)
    c.update(e)
    # unlike 7713, v2 does not decode the legacy encoding
    assert c.data.road_limit_kph == 0.0

  def test_off_route_suppresses_sdi(self):
    c = CarrotNaviV2()
    e = dict(DOC_7714_SPEED, sequence=7)
    e["value"] = dict(DOC_7714_SPEED["value"], section={"off_route": True})
    c.update(e)
    assert c.data.off_route
    assert c.data.sdi_type == -1 and c.data.sdi_distance == 0.0

  def test_absent_item_clears_it(self):
    c = CarrotNaviV2()
    c.update(DOC_7714_SPEED)
    c.update({"type": "item_update", "session_id": "0123456789abcdef", "name": "speed",
              "sequence": 999, "present": False})
    assert c.data.sdi_type == -1

  def test_session_change_resets(self):
    c = CarrotNaviV2()
    c.update(DOC_7714_SPEED)
    c.update(dict(DOC_7714_SPEED, session_id="other", sequence=1))
    assert c.data.session_id == "other"

  def test_guidance_items(self):
    c = CarrotNaviV2()
    c.update({"type": "item_update", "session_id": "s", "name": "guidance_current",
              "sequence": 1, "present": True, "value": {"turn_type": 1, "distance_m": 250}})
    c.update({"type": "item_update", "session_id": "s", "name": "guidance_next",
              "sequence": 1, "present": True, "value": {"turn_type": 3, "distance_m": 900}})
    assert (c.data.tbt_turn_type, c.data.tbt_distance) == (1, 250.0)
    assert (c.data.tbt_turn_type_next, c.data.tbt_distance_next) == (3, 900.0)

  def test_route_polyline(self):
    c = CarrotNaviV2()
    c.update({"type": "item_update", "session_id": "s", "name": "route", "sequence": 1,
              "present": True, "value": {"polyline": [[37.5, 127.0], [37.6, 127.1]]}})
    assert c.data.route_points == [(127.0, 37.5), (127.1, 37.6)]

  def test_ignores_other_envelope_types(self):
    assert CarrotNaviV2().update({"type": "manifest"}) is None
