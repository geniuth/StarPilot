"""Deceleration targets derived from Carrot Navi data.

Ported from carrot's CarrotServ. Everything here is a pure function of the navi state
plus the car's own speed, so the decision logic can be tested without a phone, a route
or a running receiver -- which matters, because none of this has ever been exercised on
a car.

The four sources carrot decelerates for, all in km/h:

  camera   an upcoming speed camera or similar SDI point; ease down so the car is at the
           posted limit `ctrl_end` seconds before reaching it
  section  average-speed enforcement, where the limit applies over a stretch of road
           rather than at a point, so the target is held flat at the limit
  bump     a speed bump, only on non-motorway roads
  atc      a turn-by-turn manoeuvre coming up
  route    curvature of the route polyline ahead

Speeds are km/h throughout, matching carrot, and 250 means "no limit from this source".
"""
import math

NO_LIMIT = 250.0

# SDI types carrot treats as a point speed restriction
SDI_CAMERA_TYPES = (0, 1, 2, 3, 4, 7, 8, 75, 76)
SDI_MOBILE_CAMERA = 7        # only honoured at control mode 3
SDI_SPEED_BUMP = 22
SDI_SECTION = 4              # synthesised when a block type says we are in a section
SDI_BLOCK_IN_SECTION = (2, 3)   # 2: inside section, 3: section end
SDI_POLICE = 100
SDI_WAZE_CAMERA = 101
SDI_POINTLESS_DISTANCE = -250.0   # police/waze stay live this far past the point


def calculate_current_speed(left_dist, safe_speed_kph, safe_time, safe_decel_rate):
  """Speed we may still be doing now and brake comfortably to safe_speed_kph in time.

  v_i^2 = v_f^2 + 2ad, with the last safe_time seconds at the target speed reserved so the
  car arrives already slowed rather than still braking. carrot's formula, unchanged.
  """
  safe_speed = safe_speed_kph / 3.6
  safe_dist = safe_speed * safe_time
  decel_dist = left_dist - safe_dist

  if decel_dist <= 0:
    return safe_speed_kph

  temp = safe_speed ** 2 + 2 * safe_decel_rate * decel_dist
  speed_mps = safe_speed if temp < 0 else math.sqrt(temp)
  return max(safe_speed_kph, min(NO_LIMIT, speed_mps * 3.6))


class NaviSpeedConfig:
  """The AutoNaviSpeed* params, converted out of their stored integer units."""

  def __init__(self, ctrl_mode=2, ctrl_end=10.0, safety_factor=1.0, decel_rate=0.8,
               bump_speed=25.0, bump_time=1.0, turn_control=0, turn_speed_mode=0,
               turn_speed=20.0, turn_end=6.0, curve_lower_limit=30.0, map_turn_factor=1.0):
    self.ctrl_mode = ctrl_mode                # 0 off, 1 cameras, 2 +bumps, 3 +mobile cameras
    self.ctrl_end = ctrl_end                  # s, arrive at the limit this early
    self.safety_factor = safety_factor        # multiplies the posted limit
    self.decel_rate = decel_rate              # m/s^2
    self.bump_speed = bump_speed              # kph over a speed bump
    self.bump_time = bump_time                # s, the bump equivalent of ctrl_end
    self.turn_control = turn_control          # 0 off, 1 steer only, 2 steer+speed, 3 speed only
    self.turn_speed_mode = turn_speed_mode    # route curvature mode, 0 off
    self.turn_speed = turn_speed              # kph through a turn manoeuvre
    self.turn_end = turn_end                  # s, distance ahead of the turn to be slowed
    self.curve_lower_limit = curve_lower_limit
    self.map_turn_factor = map_turn_factor

  @classmethod
  def from_params(cls, params):
    def num(key, default, scale=1.0):
      # Read through get() rather than get_int(), which reports an unset key as 0. That
      # would silently disable the feature instead of falling back to the default here.
      try:
        raw = params.get(key)
      except Exception:
        return default
      if raw is None:
        return default
      if isinstance(raw, bytes):
        raw = raw.decode("utf-8", "replace")
      try:
        return float(str(raw).strip()) * scale
      except (TypeError, ValueError):
        return default

    return cls(
      ctrl_mode=int(num("AutoNaviSpeedCtrlMode", 2)),
      ctrl_end=num("AutoNaviSpeedCtrlEnd", 10.0),
      safety_factor=num("AutoNaviSpeedSafetyFactor", 100.0) / 100.0,
      decel_rate=num("AutoNaviSpeedDecelRate", 80.0) / 100.0,
      bump_speed=num("AutoNaviSpeedBumpSpeed", 25.0),
      bump_time=num("AutoNaviSpeedBumpTime", 1.0),
      turn_control=int(num("AutoTurnControl", 0)),
      turn_speed_mode=int(num("TurnSpeedControlMode", 0)),
      turn_speed=num("AutoTurnControlSpeedTurn", 20.0),
      turn_end=num("AutoTurnControlTurnEnd", 6.0),
      curve_lower_limit=num("AutoCurveSpeedLowerLimit", 30.0),
      map_turn_factor=num("MapTurnSpeedFactor", 100.0) / 100.0,
    )


def select_sdi_restriction(navi, cfg):
  """carrot's _update_sdi: pick the restriction to decelerate for.

  Returns (type, limit_kph, distance_m). Type -1 and limit 0 mean nothing applies.
  A block type of 2 or 3 means we are inside average-speed enforcement, which overrides the
  point camera: the limit then holds over the block distance rather than easing to a point.
  """
  if cfg.ctrl_mode <= 0:
    return -1, 0.0, 0.0

  if navi.sdiType in SDI_CAMERA_TYPES and navi.sdiSpeedLimit > 0:
    spd_type = navi.sdiType
    limit = navi.sdiSpeedLimit * cfg.safety_factor
    dist = navi.sdiDistance

    if navi.sdiBlockType in SDI_BLOCK_IN_SECTION:
      return SDI_SECTION, limit, navi.sdiBlockDistance
    if spd_type == SDI_MOBILE_CAMERA and cfg.ctrl_mode < 3:
      return -1, 0.0, 0.0
    return spd_type, limit, dist

  # Speed bumps only off the motorway, and only once bumps are enabled
  is_bump = SDI_SPEED_BUMP in (navi.sdiType, navi.sdiPlusType)
  if is_bump and navi.roadCategory > 1 and cfg.ctrl_mode >= 2:
    dist = navi.sdiPlusDistance if navi.sdiPlusType == SDI_SPEED_BUMP else navi.sdiDistance
    return SDI_SPEED_BUMP, cfg.bump_speed, dist

  return -1, 0.0, 0.0


def expire_restriction(spd_type, limit, dist):
  """Drop a restriction we have already passed.

  Point cameras expire as soon as the distance runs out. Police and Waze reports stay live
  for a while past the point, because their position is only approximate.
  """
  if spd_type < 0:
    return -1, 0.0, 0.0
  if spd_type in (SDI_POLICE, SDI_WAZE_CAMERA):
    if dist < SDI_POINTLESS_DISTANCE:
      return -1, 0.0, 0.0
  elif dist <= 0:
    return -1, 0.0, 0.0
  return spd_type, limit, dist


def sdi_target(spd_type, limit, dist, cfg):
  """Deceleration target for the selected restriction, and the label for the UI."""
  if limit <= 0 or spd_type < 0:
    return NO_LIMIT, "none"
  if spd_type not in (SDI_POLICE, SDI_WAZE_CAMERA) and dist <= 0:
    return NO_LIMIT, "none"

  # Section enforcement applies over the stretch, so sit at the limit rather than easing to it.
  # A police or Waze point we have already passed behaves the same way.
  if spd_type == SDI_SECTION or (spd_type in (SDI_POLICE, SDI_WAZE_CAMERA) and dist <= 0):
    return limit, _sdi_source(spd_type)

  safe_sec = cfg.bump_time if spd_type == SDI_SPEED_BUMP else cfg.ctrl_end
  return calculate_current_speed(dist, limit, safe_sec, cfg.decel_rate), _sdi_source(spd_type)


def _sdi_source(spd_type):
  return {
    SDI_SPEED_BUMP: "bump",
    SDI_SECTION: "section",
    SDI_POLICE: "police",
    SDI_WAZE_CAMERA: "waze",
  }.get(spd_type, "cam")


# Turn-by-turn manoeuvre types, and how carrot treats each
TURN_LEFT, TURN_RIGHT = 1, 2
FORK_LEFT, FORK_RIGHT = 3, 4
TURN_STRAIGHT, FORK_STRAIGHT = 5, 6
TBT_STOP_TYPES = (7, 8)


def atc_target(turn_type, dist_to_turn, road_limit_kph, next_road_width, cfg):
  """Deceleration for an upcoming turn-by-turn manoeuvre.

  Turns get slowed to the configured turn speed; forks and straights only to the road limit,
  since they do not need much scrubbing off; stop manoeuvres go to a crawl.
  """
  if cfg.turn_speed_mode not in (2, 3):
    return NO_LIMIT, "none"

  turn_speed = cfg.turn_speed
  fork_speed = road_limit_kph
  stop_speed = 1.0

  turn_dist = cfg.turn_end * turn_speed / 3.6
  fork_dist = cfg.turn_end * fork_speed / 3.6

  mapping = {
    TURN_LEFT: (turn_speed, turn_dist),
    TURN_RIGHT: (turn_speed, turn_dist),
    TURN_STRAIGHT: (turn_speed, turn_dist),
    FORK_LEFT: (fork_speed, fork_dist),
    FORK_RIGHT: (fork_speed, fork_dist),
    FORK_STRAIGHT: (fork_speed, fork_dist),
  }
  if turn_type in TBT_STOP_TYPES:
    speed, stop_dist = stop_speed, 5.0
    mapping[turn_type] = (speed, stop_dist)

  if turn_type not in mapping:
    return NO_LIMIT, "none"

  atc_speed, atc_dist = mapping[turn_type]
  if atc_speed <= 0 or dist_to_turn <= 0:
    return NO_LIMIT, "none"

  # carrot reserves 2s here rather than ctrl_end
  return calculate_current_speed(dist_to_turn - atc_dist, atc_speed, 2.0, cfg.decel_rate), "atc"


def route_target(route_speed_kph, dist_to_turn, cfg):
  """Curvature deceleration from the route polyline, gated by the configured mode."""
  if cfg.turn_speed_mode <= 0 or route_speed_kph <= 0:
    return NO_LIMIT, "none"

  target = max(route_speed_kph * cfg.map_turn_factor, cfg.curve_lower_limit)

  # Mode 2 only trusts the route near a manoeuvre; modes 3 and 4 use it throughout
  if cfg.turn_speed_mode == 2:
    if not -500.0 < dist_to_turn < 500.0:
      return NO_LIMIT, "none"
  elif cfg.turn_speed_mode not in (3, 4):
    return NO_LIMIT, "none"

  return target, "route"


def desired_speed(navi, cfg, road_limit_kph=0.0):
  """Combine every source into one target, returning (speed_kph, source).

  Sources that have nothing to say return 250, so the minimum is the binding one. When
  nothing binds the result is 250 with source "none", and the caller should not apply it.
  """
  spd_type, limit, dist = select_sdi_restriction(navi, cfg)
  spd_type, limit, dist = expire_restriction(spd_type, limit, dist)

  candidates = [sdi_target(spd_type, limit, dist, cfg)]

  candidates.append(atc_target(navi.tbtTurnType, navi.tbtDistance, road_limit_kph,
                               navi.tbtNextRoadWidth, cfg))
  candidates.append(atc_target(navi.tbtTurnTypeNext, navi.tbtDistanceNext, road_limit_kph,
                               navi.tbtNextRoadWidth, cfg))
  candidates.append(route_target(getattr(navi, "routeCurveSpeed", 0.0), navi.tbtDistance, cfg))

  if road_limit_kph > 0:
    candidates.append((road_limit_kph, "road"))

  speed, source = min(candidates, key=lambda c: c[0])
  if speed >= NO_LIMIT:
    return NO_LIMIT, "none"
  return speed, source
