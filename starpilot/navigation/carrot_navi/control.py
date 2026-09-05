"""Bridge between the published carrotNaviState and the cruise target.

Kept apart from speed.py so the deceleration maths stays free of cereal, and apart from
the receiver so the planner never touches a socket.
"""
from openpilot.starpilot.navigation.carrot_navi.speed import NO_LIMIT, desired_speed

CV_KPH_TO_MS = 1 / 3.6

# Below this the navi has effectively asked for a stop, which is not this feature's job
MIN_TARGET_KPH = 5.0


def get_carrot_navi_target(sm, cfg):
  """Return (target_ms, source). A target of 0 means the navi is asking for nothing.

  The caller appends the target to its minimum, so returning 0 rather than a large number
  keeps a disconnected navi from ever influencing the cruise target.
  """
  if cfg is None or not sm.valid.get("carrotNaviState", False):
    return 0.0, "none"

  navi = sm["carrotNaviState"]
  if not navi.connected or navi.active <= 0:
    return 0.0, "none"

  # 7714 suppresses its own guidance off route; do not second-guess it here
  if navi.offRoute:
    return 0.0, "none"

  speed_kph, source = desired_speed(navi, cfg, road_limit_kph=navi.roadLimitSpeed)
  if speed_kph >= NO_LIMIT or speed_kph < MIN_TARGET_KPH:
    return 0.0, "none"

  return speed_kph * CV_KPH_TO_MS, source
