"""Wire formats for the two Carrot Navi feeds.

Both feeds carry the same information under different names, so both are decoded into
one NaviData. Field names on the way out follow carrot's, so the mapping back to that
port stays readable.

  7713   legacy HTTP. POST /api/navi/<tmap_version> with a flat `rgdata` object using
         the nSdi* / nTBT* names. Nested guidance/sdi/lane groups are flattened, with
         top-level rgdata keys winning on conflict.

  7714   Carrot Navi v2. A WebSocket item stream at
         /api/navi/ws/v2/json/<session_id>/<item>, one envelope per item, values in
         snake_case. The items that carry deceleration input are speed, lane_current,
         guidance_current, guidance_next, route and vehicle.

Both schemas are as documented in docs/carrot_navi_7713_7714_deceleration.md.
"""
from dataclasses import dataclass, field

# 7713 sends the road limit as its update gate; values above this are a legacy encoding
ROAD_LIMIT_MAX_KPH = 200.0
# carrot substitutes this when 7713 reports no road limit at all
ROAD_LIMIT_FALLBACK_KPH = 30.0
ROUTE_POINT_LIMIT = 4096


@dataclass
class NaviData:
  """Everything the deceleration engine reads, in carrot's units."""
  connected: bool = False
  source: str = ""
  session_id: str = ""
  off_route: bool = False

  road_limit_kph: float = 0.0
  road_category: int = 0
  road_name: str = ""

  sdi_type: int = -1
  sdi_speed_limit: float = 0.0
  sdi_distance: float = 0.0
  sdi_block_type: int = -1
  sdi_block_speed: float = 0.0
  sdi_block_distance: float = 0.0
  sdi_plus_type: int = -1
  sdi_plus_speed_limit: float = 0.0
  sdi_plus_distance: float = 0.0

  tbt_turn_type: int = -1
  tbt_distance: float = 0.0
  tbt_turn_type_next: int = -1
  tbt_distance_next: float = 0.0
  tbt_next_road_width: float = 0.0

  latitude: float = 0.0
  longitude: float = 0.0
  bearing: float = 0.0

  route_points: list = field(default_factory=list)   # (lon, lat) pairs
  route_sequence: int = -1

  go_pos_distance: float = 0.0
  go_pos_time: float = 0.0


def _num(d, key, default=0.0):
  try:
    v = d.get(key, default)
  except AttributeError:
    return default
  if v is None or isinstance(v, bool):
    return default
  try:
    return float(v)
  except (TypeError, ValueError):
    return default


def _int(d, key, default=-1):
  return int(_num(d, key, default))


def normalise_road_limit(raw):
  """carrot's 7713 road limit handling.

  Values over 200 are a legacy encoding whose low two digits are the real limit, and a
  missing or zero limit falls back to 30 rather than meaning "unrestricted". The 7714 path
  deliberately does neither, treating anything out of range as no limit at all.
  """
  if raw > ROAD_LIMIT_MAX_KPH:
    decoded = raw % 100.0
    return decoded if decoded > 0 else ROAD_LIMIT_FALLBACK_KPH
  if raw <= 0:
    return ROAD_LIMIT_FALLBACK_KPH
  return raw


def flatten_rgdata(rgdata):
  """7713 nests guidance/sdi/lane groups; top-level keys win on conflict."""
  flat = {}
  for group in ("guidance", "sdi", "lane"):
    nested = rgdata.get(group)
    if isinstance(nested, dict):
      flat.update(nested)
  flat.update({k: v for k, v in rgdata.items() if not isinstance(v, dict)})
  return flat


def parse_7713(payload, previous=None):
  """Decode a legacy HTTP body. Returns None when the SDI update gate is absent.

  carrot only refreshes navi fields when nRoadLimitSpeed is present, so a body without it
  is a keepalive rather than an update and must not clear live state.
  """
  rgdata = payload.get("rgdata")
  if not isinstance(rgdata, dict):
    return None

  flat = flatten_rgdata(rgdata)
  if "nRoadLimitSpeed" not in flat:
    return None

  d = NaviData(connected=True, source="7713")
  if previous is not None:
    d.route_points = previous.route_points
    d.route_sequence = previous.route_sequence
    d.latitude, d.longitude = previous.latitude, previous.longitude
    d.bearing = previous.bearing

  d.road_limit_kph = normalise_road_limit(_num(flat, "nRoadLimitSpeed"))
  d.road_category = _int(flat, "roadcate", 0)
  d.road_name = str(flat.get("szPosRoadName", "") or "")

  d.sdi_type = _int(flat, "nSdiType")
  d.sdi_speed_limit = _num(flat, "nSdiSpeedLimit")
  d.sdi_distance = _num(flat, "nSdiDist")
  d.sdi_block_type = _int(flat, "nSdiBlockType")
  d.sdi_block_speed = _num(flat, "nSdiBlockSpeed")
  d.sdi_block_distance = _num(flat, "nSdiBlockDist")
  d.sdi_plus_type = _int(flat, "nSdiPlusType")
  d.sdi_plus_speed_limit = _num(flat, "nSdiPlusSpeedLimit")
  d.sdi_plus_distance = _num(flat, "nSdiPlusDist")

  d.tbt_turn_type = _int(flat, "nTBTTurnType")
  d.tbt_distance = _num(flat, "nTBTDist")
  d.tbt_turn_type_next = _int(flat, "nTBTTurnTypeNext")
  d.tbt_distance_next = _num(flat, "nTBTDistNext")
  d.tbt_next_road_width = _num(flat, "nTBTNextRoadWidth")

  d.go_pos_distance = _num(flat, "nGoPosDist")
  d.go_pos_time = _num(flat, "nGoPosTime")

  lat, lon = _num(flat, "vpPosPointLat"), _num(flat, "vpPosPointLon")
  if lat or lon:
    d.latitude, d.longitude = lat, lon
  bearing = _num(flat, "nPosAngle")
  if bearing:
    d.bearing = bearing

  return d


def parse_7713_route(payload):
  """Decode a 7713 route/vrtx polyline into (lon, lat) pairs."""
  points = payload.get("route") or payload.get("vrtx") or []
  out = []
  for p in points[:ROUTE_POINT_LIMIT]:
    if isinstance(p, dict):
      lat, lon = _num(p, "latitude"), _num(p, "longitude")
    elif isinstance(p, (list, tuple)) and len(p) >= 2:
      lat, lon = float(p[0]), float(p[1])
    else:
      continue
    if lat or lon:
      out.append((lon, lat))
  return out


class CarrotNaviV2:
  """Accumulates 7714 item envelopes into a NaviData.

  Items arrive independently, each with its own sequence, so state persists between
  envelopes and is only reset when the session changes.
  """

  def __init__(self):
    self.data = NaviData(source="7714")
    self.sequences = {}

  def _reset(self, session_id):
    self.data = NaviData(connected=True, source="7714", session_id=session_id)
    self.sequences = {}

  def update(self, envelope):
    """Apply one item_update envelope. Returns the current NaviData, or None if ignored."""
    if envelope.get("type") != "item_update":
      return None

    session_id = str(envelope.get("session_id", "") or "")
    if session_id != self.data.session_id:
      self._reset(session_id)

    name = envelope.get("name")
    sequence = envelope.get("sequence")
    if name is None:
      return None

    # A repeated sequence is a heartbeat: it must not overwrite distances we are counting
    # down locally, which is the bug the 7713/7714 comparison calls out.
    if sequence is not None and self.sequences.get(name) == sequence:
      return self.data
    self.sequences[name] = sequence

    self.data.connected = True
    if not envelope.get("present", True):
      self._clear_item(name)
      return self.data

    value = envelope.get("value")
    if not isinstance(value, dict):
      return self.data

    handler = {
      "speed": self._speed,
      "lane_current": self._lane,
      "guidance_current": self._guidance_current,
      "guidance_next": self._guidance_next,
      "route": self._route,
      "vehicle": self._vehicle,
    }.get(name)
    if handler is not None:
      handler(value)
    return self.data

  def _clear_item(self, name):
    d = self.data
    if name == "speed":
      d.sdi_type = d.sdi_block_type = d.sdi_plus_type = -1
      d.sdi_speed_limit = d.sdi_distance = 0.0
      d.sdi_block_speed = d.sdi_block_distance = 0.0
      d.sdi_plus_speed_limit = d.sdi_plus_distance = 0.0
      d.road_limit_kph = 0.0
    elif name == "guidance_current":
      d.tbt_turn_type, d.tbt_distance = -1, 0.0
    elif name == "guidance_next":
      d.tbt_turn_type_next, d.tbt_distance_next = -1, 0.0
    elif name == "route":
      d.route_points = []

  def _speed(self, v):
    d = self.data
    raw_limit = _num(v, "road_limit_kph")
    # 7714 treats an out-of-range road limit as no limit, rather than decoding or defaulting
    d.road_limit_kph = raw_limit if 0 < raw_limit <= ROAD_LIMIT_MAX_KPH else 0.0

    sdi = v.get("sdi") if isinstance(v.get("sdi"), dict) else {}
    d.sdi_type = _int(sdi, "type")
    d.sdi_speed_limit = _num(sdi, "speed_limit_kph")
    d.sdi_distance = _num(sdi, "distance_m")
    d.sdi_block_type = _int(sdi, "block_type")
    d.sdi_block_speed = _num(sdi, "block_speed_kph")
    d.sdi_block_distance = _num(sdi, "block_distance_m")

    plus = v.get("sdi_secondary") if isinstance(v.get("sdi_secondary"), dict) else {}
    d.sdi_plus_type = _int(plus, "type")
    d.sdi_plus_speed_limit = _num(plus, "speed_limit_kph")
    d.sdi_plus_distance = _num(plus, "distance_m")

    section = v.get("section") if isinstance(v.get("section"), dict) else {}
    d.off_route = bool(section.get("off_route", False))
    if d.off_route:
      # 7714 suppresses SDI and TBT off route; 7713 has no such gate
      self._clear_item("speed")
      self._clear_item("guidance_current")
      self._clear_item("guidance_next")

  def _lane(self, v):
    self.data.road_category = _int(v, "road_category", 0)

  def _guidance_current(self, v):
    self.data.tbt_turn_type = _int(v, "turn_type")
    self.data.tbt_distance = _num(v, "distance_m")
    width = _num(v, "road_width")
    if width:
      self.data.tbt_next_road_width = width

  def _guidance_next(self, v):
    self.data.tbt_turn_type_next = _int(v, "turn_type")
    self.data.tbt_distance_next = _num(v, "distance_m")

  def _route(self, v):
    points = v.get("polyline") or []
    out = []
    for p in points[:ROUTE_POINT_LIMIT]:
      if isinstance(p, (list, tuple)) and len(p) >= 2:
        lat, lon = float(p[0]), float(p[1])
        out.append((lon, lat))
    self.data.route_points = out
    self.data.route_sequence = int(_num(v, "sequence", -1))

  def _vehicle(self, v):
    d = self.data
    lat, lon = _num(v, "latitude"), _num(v, "longitude")
    if lat or lon:
      d.latitude, d.longitude = lat, lon
    bearing = _num(v, "bearing")
    if bearing:
      d.bearing = bearing
