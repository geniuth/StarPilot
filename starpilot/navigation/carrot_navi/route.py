"""Curvature deceleration from the navi route polyline.

Ported from carrot's carrot_man route handling. carrot resamples the path with shapely;
this does the same resampling with plain linear interpolation so the feature does not
drag in a geometry dependency for one call.

The shape of the calculation is carrot's:

  1. find where we are on the polyline and take the stretch ahead of us
  2. project it to metres relative to the car, rotated so +x is straight ahead
  3. resample at a fixed interval and measure curvature over a sliding window
  4. look the curvature up as a speed
  5. walk backwards applying the deceleration limit, so a tight corner 200 m out starts
     slowing us now rather than demanding an impossible stop when we reach it
"""
import math

EARTH_RADIUS_M = 6371000.0
DEG_TO_M = 40008000.0 / 360.0

# carrot's curvature to speed table, curvature in 1/m and speed in km/h
V_CURVE_LOOKUP_BP = [0., 1 / 800., 1 / 670., 1 / 560., 1 / 440., 1 / 360., 1 / 265.,
                     1 / 190., 1 / 135., 1 / 85., 1 / 55., 1 / 30., 1 / 25.]
V_CURVE_LOOKUP_VALS = [300., 150., 120., 110., 100., 90., 80., 70., 60., 50., 40., 15., 5.]

LOOKAHEAD_M = 300.0
RESAMPLE_INTERVAL_M = 10.0
CURVATURE_SAMPLE = 4          # window half-width, in resampled points
STRAIGHT_CURVATURE = 0.02     # below this carrot lets the road limit stand
NO_LIMIT = 300.0


def haversine(lon1, lat1, lon2, lat2):
  phi1, phi2 = math.radians(lat1), math.radians(lat2)
  dphi = math.radians(lat2 - lat1)
  dlambda = math.radians(lon2 - lon1)
  a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
  return 2 * EARTH_RADIUS_M * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def closest_point_on_segment(p1, p2, p):
  """Nearest point to p on segment p1..p2, in lon/lat treated as a plane."""
  x1, y1 = p1
  x2, y2 = p2
  px, py = p
  dx, dy = x2 - x1, y2 - y1
  if dx == 0 and dy == 0:
    return p1
  t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
  t = max(0.0, min(1.0, t))
  return (x1 + t * dx, y1 + t * dy)


def get_path_after_distance(start_index, coordinates, current_position, distance_m):
  """The stretch of polyline ahead of us, plus the index to resume searching from."""
  if len(coordinates) < 2:
    return [], start_index, current_position

  start_index = max(0, start_index - 2)
  closest_index, closest_point, min_distance = -1, None, float("inf")

  for i in range(start_index, len(coordinates) - 1):
    candidate = closest_point_on_segment(coordinates[i], coordinates[i + 1], current_position)
    d = haversine(current_position[0], current_position[1], candidate[0], candidate[1])
    if d < min_distance:
      min_distance, closest_index, closest_point = d, i, candidate

  if closest_index < 0:
    return [], start_index, current_position

  path, total = [closest_point], 0.0
  prev = closest_point
  for i in range(closest_index + 1, len(coordinates)):
    pt = coordinates[i]
    total += haversine(prev[0], prev[1], pt[0], pt[1])
    path.append(pt)
    prev = pt
    if total >= distance_m:
      break

  return path, closest_index, closest_point


def gps_to_relative_xy(gps_path, reference_point, heading_deg):
  """Project to metres relative to reference_point, rotated so the car heads along +x."""
  ref_lon, ref_lat = reference_point
  heading_rad = math.radians(heading_deg)
  out = []
  for lon, lat in gps_path:
    x = (lon - ref_lon) * DEG_TO_M * math.cos(math.radians(ref_lat))
    y = (lat - ref_lat) * DEG_TO_M
    x_rot = x * math.cos(heading_rad) - y * math.sin(heading_rad)
    y_rot = x * math.sin(heading_rad) + y * math.cos(heading_rad)
    out.append((y_rot, x_rot))
  return out


def resample(points, interval):
  """Even spacing along the polyline, replacing shapely's interpolate."""
  if len(points) < 2:
    return list(points)

  seg_len = []
  for i in range(len(points) - 1):
    dx = points[i + 1][0] - points[i][0]
    dy = points[i + 1][1] - points[i][1]
    seg_len.append(math.hypot(dx, dy))
  total = sum(seg_len)
  if total <= 0:
    return [points[0]]

  out, target, i, walked = [], 0.0, 0, 0.0
  while target <= total and i < len(seg_len):
    while i < len(seg_len) and walked + seg_len[i] < target:
      walked += seg_len[i]
      i += 1
    if i >= len(seg_len):
      break
    remain = target - walked
    t = remain / seg_len[i] if seg_len[i] > 0 else 0.0
    x = points[i][0] + t * (points[i + 1][0] - points[i][0])
    y = points[i][1] + t * (points[i + 1][1] - points[i][1])
    out.append((x, y))
    target += interval
  return out


def calculate_curvature(p1, p2, p3):
  v1 = (p2[0] - p1[0], p2[1] - p1[1])
  v2 = (p3[0] - p2[0], p3[1] - p2[1])
  cross = v1[0] * v2[1] - v1[1] * v2[0]
  len_v1 = math.hypot(*v1)
  len_v2 = math.hypot(*v2)
  if len_v1 * len_v2 == 0:
    return 0.0
  return cross / (len_v1 * len_v2 * len_v1)


def _interp(x, xp, fp):
  if x <= xp[0]:
    return fp[0]
  if x >= xp[-1]:
    return fp[-1]
  for i in range(1, len(xp)):
    if x <= xp[i]:
      span = xp[i] - xp[i - 1]
      t = 0.0 if span == 0 else (x - xp[i - 1]) / span
      return fp[i - 1] + t * (fp[i] - fp[i - 1])
  return fp[-1]


def route_curve_speed(route_points, position, bearing_deg, v_ego_kph, road_limit_kph,
                      decel_rate, ctrl_end, start_index=0):
  """Lowest speed the route curvature ahead demands, in km/h.

  Returns (speed_kph, new_start_index). NO_LIMIT means the route asks for nothing.
  """
  if not route_points or len(route_points) < 2:
    return NO_LIMIT, start_index

  path, new_index, start_point = get_path_after_distance(
    start_index, route_points, position, LOOKAHEAD_M)
  if len(path) < 2:
    return NO_LIMIT, new_index

  relative = gps_to_relative_xy(path, start_point, bearing_deg)
  resampled = resample(relative, RESAMPLE_INTERVAL_M)
  if len(resampled) < CURVATURE_SAMPLE * 2 + 1:
    return NO_LIMIT, new_index

  speeds = []
  for i in range(len(resampled) - CURVATURE_SAMPLE * 2):
    p1 = resampled[i]
    p2 = resampled[i + CURVATURE_SAMPLE]
    p3 = resampled[i + CURVATURE_SAMPLE * 2]
    curvature = calculate_curvature(p1, p2, p3)
    speed = _interp(abs(curvature), V_CURVE_LOOKUP_BP, V_CURVE_LOOKUP_VALS)
    if abs(curvature) < STRAIGHT_CURVATURE:
      speed = max(speed, road_limit_kph)
    speeds.append(speed)

  if not speeds:
    return NO_LIMIT, new_index

  # Walk backwards so a corner further along starts slowing us now, within the decel limit
  accel_limit_kmh = decel_rate * 3.6
  out = list(speeds)
  time_wait = 0.0
  for i in range(len(speeds) - 2, -1, -1):
    target_speed = speeds[i]
    next_out = out[i + 1]

    if target_speed < next_out:
      time_wait = -max(0.0, (v_ego_kph - target_speed) / accel_limit_kmh) if accel_limit_kmh else 0.0

    time_interval = RESAMPLE_INTERVAL_M / (next_out / 3.6) if next_out > 0 else 0.0
    time_apply = min(time_interval, max(0.0, time_interval + time_wait))
    max_allowed = next_out + accel_limit_kmh * time_apply
    out[i] = min(target_speed, max_allowed)

  return max(min(out[0], NO_LIMIT), 0.0), new_index
