"""NDA receiver: speed cameras, section enforcement, road limits and traffic signals from a phone app.

Implements the neokii ROAD_LIMIT_SERVICE protocol used by npilot and nmirror2:
  1. Until a phone answers, send the SDP string to UDP 2899 of every host on the wlan/bridge /24 every 5 s.
  2. The phone sends {"active", "road_limit", "traffic_signal", ...} JSON to UDP 3843 (older senders: 2843).
  3. Keep answering with SDP so the phone keeps the session; 6 s of silence drops it.
  4. When the phone sends request_gps=1, device GPS is sent back to its port 2899.

Received values are published as naviData (~3 Hz). SpeedLimiter turns them into a camera/section
target speed for the longitudinal planner and for selfdrived's camera warning.

Ported from jc01rho/StarPilot paddle5 (whrho, jc01rho) with the receiver hardened as in
geniuth/openpilot feature/nmirror-bridge (selfdrive/carrot/nmirror_bridge.py):
  - cmd / echo_cmd (shell execution) are never run: anyone on the same Wi-Fi could run commands.
  - Discovery covers every wlan*/br* interface (hotspot or client), not just wlan0.
  - The configured GPS service is used (gpsLocation or gpsLocationExternal).
  - Raw road_limit / traffic_signal values are logged only when they change.
"""
import ipaddress
import json
import select
import signal
import socket
import sys
import threading
import time
from collections import deque

import numpy as np
import psutil

from cereal import messaging
from openpilot.common.constants import CV
from openpilot.common.gps import get_gps_location_service
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.navi_camera import SECTION_SMOOTH_CAMERA_TYPES, RearCameraMotion, traffic_signal_fields

CAMERA_SPEED_FACTOR = 1.0
# v2: rear cameras (75/76/86~89) arrive as a signed distance plus cam_rear_hold_dist/cam_distance_id/cam_distance_age.
ROAD_LIMIT_SERVICE = b'EON:ROAD_LIMIT_SERVICE:v2'
terminate_flag = threading.Event()

DISCOVERY_INTERVAL_S = 5.0
SDP_INTERVAL_S = 1.0
GPS_INTERVAL_S = 1.25
CONNECTION_TIMEOUT_S = 6.0
PUBLISH_INTERVAL_S = 0.33
DISCOVERY_IFACE_PREFIXES = ("wlan", "br")


class Port:
  BROADCAST_PORT = 2899
  # 2843 is the pre-2024-06 npilot port that some senders still hardcode; 3843 is the current npilot port.
  RECEIVE_PORTS = (2843, 3843)
  LOCATION_PORT = BROADCAST_PORT


def discovery_targets(if_addrs=None) -> list[str]:
  """Host addresses of the /24 around every wlan*/br* IPv4 address, excluding ourselves."""
  if if_addrs is None:
    try:
      if_addrs = psutil.net_if_addrs()
    except Exception:
      return []
  targets: list[str] = []
  for name, addresses in if_addrs.items():
    if not name.startswith(DISCOVERY_IFACE_PREFIXES):
      continue
    for addr in addresses:
      if addr.family != socket.AF_INET or not addr.address or addr.address.startswith("127."):
        continue
      network = ipaddress.ip_network(f"{addr.address}/24", strict=False)
      targets.extend(str(host) for host in network.hosts() if str(host) != addr.address)
  return targets


def location_payload(location) -> bytes:
  # Same array order as npilot navi_controller.gps_timer.
  return json.dumps({"location": [
    location.latitude,
    location.longitude,
    location.altitude,
    location.speed,
    location.bearingDeg,
    location.horizontalAccuracy,
    location.unixTimestampMillis,
    location.verticalAccuracy,
    location.bearingAccuracyDeg,
    location.speedAccuracy,
  ]}).encode()


class NaviServer:
  def __init__(self, gps_service: str | None = None, start_threads: bool = True):
    if gps_service is None:
      try:
        gps_service = get_gps_location_service(Params())
      except Exception:
        gps_service = "gpsLocationExternal"
    self.gps_service = gps_service
    self.sm = messaging.SubMaster([gps_service, 'carState'], frequency=int(1. / DT_CTRL))
    self.json_road_limit = None
    self.json_traffic_signal = None
    self.active = 0
    self.last_updated = 0.0
    self.last_updated_active = 0.0
    self.last_sdp = 0.0
    self.lock = threading.Lock()
    self.remote_addr = None
    self.remote_gps_addr = None
    self.seen_receive_ports: set[int] = set()
    self.ignored_cmd_logged = False
    self.logged_road_limit = None
    self.logged_traffic_signal = None
    self.gps_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.location = None

    if start_threads:
      for target in (self.discovery_thread, self.update_thread, self.gps_thread):
        threading.Thread(target=target, daemon=True).start()

  def gps_thread(self):
    while not terminate_flag.is_set():
      self.gps_timer()
      time.sleep(GPS_INTERVAL_S)

  def gps_timer(self):
    if self.remote_gps_addr is None:
      return

    try:
      if self.sm.valid[self.gps_service] and self.sm.updated[self.gps_service]:
        self.location = self.sm[self.gps_service]
    except Exception as e:
      cloudlog.info(f"road_speed_limiter: GPS read error: {e}")
      return

    if self.location is None:
      return

    try:
      self.gps_socket.sendto(location_payload(self.location), (self.remote_gps_addr[0], Port.LOCATION_PORT))
    except Exception as e:
      cloudlog.info(f"road_speed_limiter: GPS send error: {e}")
      self.remote_gps_addr = None

  def discovery_thread(self):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
      while not terminate_flag.is_set():
        if self.remote_addr is None:
          for ip in discovery_targets():
            try:
              sock.sendto(ROAD_LIMIT_SERVICE, (ip, Port.BROADCAST_PORT))
            except OSError:
              pass
        time.sleep(DISCOVERY_INTERVAL_S)

  def update_thread(self):
    while not terminate_flag.is_set():
      try:
        self.sm.update(0)
      except Exception as e:
        cloudlog.error(f"road_speed_limiter: update_thread error: {e}")
      time.sleep(0.05)

  def send_sdp(self, sock, now: float | None = None):
    now = time.monotonic() if now is None else now
    if self.remote_addr is None or now - self.last_sdp < SDP_INTERVAL_S:
      return
    self.last_sdp = now
    try:
      sock.sendto(ROAD_LIMIT_SERVICE, (self.remote_addr[0], Port.BROADCAST_PORT))
    except OSError as e:
      cloudlog.info(f"road_speed_limiter: send SDP error: {e}")

  def udp_recv(self, socks):
    try:
      ready = select.select(socks, [], [], 0.1)[0]
    except Exception as e:
      cloudlog.info(f"road_speed_limiter: UDP select error: {e}")
      return False
    received = False
    for sock in ready:
      try:
        data, addr = sock.recvfrom(4096)
      except OSError:
        continue
      received = self.handle_packet(data, addr, sock) or received
    return received

  def handle_packet(self, data: bytes, addr, sock) -> bool:
    try:
      json_obj = json.loads(data.decode())
    except (UnicodeDecodeError, ValueError):
      return False
    if not isinstance(json_obj, dict):
      return False

    if self.remote_addr is None or self.remote_addr[0] != addr[0]:
      cloudlog.event("nda_connected", remote=addr[0])
    self.remote_addr = addr

    port = sock.getsockname()[1]
    if port not in self.seen_receive_ports:
      self.seen_receive_ports.add(port)
      cloudlog.info(f"road_speed_limiter: first packet on port {port} from {addr[0]}")

    if ('cmd' in json_obj or 'echo_cmd' in json_obj) and not self.ignored_cmd_logged:
      # Shell execution from the LAN is never supported.
      cloudlog.warning("road_speed_limiter: ignoring cmd/echo_cmd")
      self.ignored_cmd_logged = True

    if 'request_gps' in json_obj:
      self.remote_gps_addr = addr if json_obj['request_gps'] == 1 else None

    if 'echo' in json_obj:
      try:
        sock.sendto(json.dumps(json_obj['echo']).encode(), (addr[0], Port.BROADCAST_PORT))
      except (OSError, TypeError, ValueError):
        pass

    road_limit = json_obj.get('road_limit')
    traffic_signal = json_obj.get('traffic_signal')
    if isinstance(road_limit, dict) and road_limit != self.logged_road_limit:
      self.logged_road_limit = road_limit
      cloudlog.event("nda_road_limit", active=json_obj.get('active'), road_limit=road_limit)
    if isinstance(traffic_signal, dict) and traffic_signal != self.logged_traffic_signal:
      self.logged_traffic_signal = traffic_signal
      cloudlog.event("nda_traffic_signal", traffic_signal=traffic_signal)

    now = time.monotonic()
    received = False
    with self.lock:
      if 'active' in json_obj:
        self.active = json_obj['active']
        self.last_updated_active = now
        received = True
      if isinstance(road_limit, dict):
        self.json_road_limit = road_limit
        self.last_updated = now
        received = True
      if isinstance(traffic_signal, dict):
        self.json_traffic_signal = traffic_signal
        received = True
    return received

  def check(self, now: float | None = None):
    now = time.monotonic() if now is None else now
    if now - self.last_updated > CONNECTION_TIMEOUT_S:
      with self.lock:
        self.json_road_limit = None
        self.json_traffic_signal = None

    if now - self.last_updated_active > CONNECTION_TIMEOUT_S:
      self.active = 0
      if self.remote_addr is not None:
        cloudlog.event("nda_disconnected", remote=self.remote_addr[0])
      self.remote_addr = None
      self.remote_gps_addr = None
      self.logged_road_limit = self.logged_traffic_signal = None

  @staticmethod
  def get_json_val(json_data, key, default=None):
    try:
      if json_data is not None and key in json_data:
        return json_data[key]
    except Exception as e:
      cloudlog.info(f"road_speed_limiter: JSON value get error for key '{key}': {e}")
    return default

  def get_limit_val(self, key, default=None):
    return self.get_json_val(self.json_road_limit, key, default)


def main():
  server = NaviServer()
  sm = server.sm
  navi_data_sock = messaging.pub_sock('naviData')
  last_run_time = time.monotonic()
  v_ego_q: deque[float] = deque(maxlen=3)
  rear_motion = RearCameraMotion()

  socks = []
  for port in Port.RECEIVE_PORTS:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', port))
    sock.setblocking(False)
    socks.append(sock)

  try:
    while not terminate_flag.is_set():
      try:
        server.udp_recv(socks)

        dat = messaging.new_message('naviData', valid=True)
        dat.init('naviData')
        dat.naviData.active = server.active
        dat.naviData.roadLimitSpeed = server.get_limit_val('road_limit_speed', 0)
        dat.naviData.isHighway = server.get_limit_val('is_highway', False)
        dat.naviData.camType = server.get_limit_val('cam_type', 0)
        dat.naviData.camLimitSpeed = server.get_limit_val('cam_limit_speed', 0)
        dat.naviData.sectionLimitSpeed = server.get_limit_val('section_limit_speed', 0)
        dat.naviData.sectionLeftDist = server.get_limit_val('section_left_dist', 0)
        dat.naviData.sectionAvgSpeed = server.get_limit_val('section_avg_speed', 0)
        dat.naviData.sectionLeftTime = server.get_limit_val('section_left_time', 0)
        dat.naviData.sectionAdjustSpeed = server.get_limit_val('section_adjust_speed', False)
        dat.naviData.camSpeedFactor = server.get_limit_val('cam_speed_factor', CAMERA_SPEED_FACTOR)
        dat.naviData.currentRoadName = server.get_limit_val('current_road_name', '')
        dat.naviData.isNda2 = server.get_limit_val('is_nda2', False)
        dat.naviData.ts = traffic_signal_fields(server.json_traffic_signal)

        if sm.updated['carState']:
          v_ego_q.append(sm['carState'].vEgo)

        v_ego = float(np.mean(v_ego_q)) if len(v_ego_q) > 0 else 0.0
        now = time.monotonic()
        traveled = (now - server.last_updated) * v_ego

        dat.naviData.camLimitSpeedLeftDist, dat.naviData.camRearPassed = rear_motion.update(
          dat.naviData.camType, server.get_limit_val('cam_limit_speed_left_dist', 0),
          server.get_limit_val('cam_rear_hold_dist'), server.get_limit_val('cam_distance_id'),
          server.last_updated, now, server.get_limit_val('cam_distance_age', 0.0), v_ego)
        if dat.naviData.sectionLeftDist > 0:
          dat.naviData.sectionLeftDist = int(max(dat.naviData.sectionLeftDist - traveled, 0))

        server.send_sdp(socks[-1], now)
        if now - last_run_time < PUBLISH_INTERVAL_S:
          continue
        last_run_time = now

        navi_data_sock.send(dat.to_bytes())
        server.check(now)
      except Exception as e:
        cloudlog.info(f"road_speed_limiter loop error: {e}")
        time.sleep(1.0)
  finally:
    for sock in socks:
      sock.close()


class SpeedLimiter:
  __instance = None

  @classmethod
  def instance(cls):
    if cls.__instance is None:
      cls.__instance = cls()
    return cls.__instance

  def __init__(self):
    self.slowing_down = False
    self.started_dist = 0
    self.last_limit_speed_left_dist = 0
    self.rear_passed = False
    self.sock = messaging.sub_sock('naviData')
    self.naviData = None
    self.logMonoTime = 0

  def recv(self):
    try:
      dat = messaging.recv_sock(self.sock, wait=False)
      if dat is not None:
        self.logMonoTime = dat.logMonoTime
        self.naviData = dat.naviData
    except Exception as e:
      cloudlog.info(f"SpeedLimiter recv error: {e}")
      self.naviData = None

  def get_active(self):
    self.recv()
    return self.naviData.active if self.naviData is not None else 0

  def get_cam_active(self):
    self.recv()
    return self.naviData is not None and self.naviData.active and (self.naviData.camLimitSpeedLeftDist > 0 or self.naviData.sectionLeftDist > 0)

  def get_road_limit_speed(self):
    self.recv()
    if self.naviData is None:
      return 0

    current_time = time.monotonic_ns()
    if (current_time - self.logMonoTime) / 1e9 > 3.0:
      return 0

    return self.naviData.roadLimitSpeed

  def get_max_speed(self, CS, v_cruise_kph):
    self.recv()
    if self.naviData is None or CS is None:
      self.slowing_down = False
      return 0, 0, 0, False, ''

    # Drop stale data the same way get_road_limit_speed does.
    if (time.monotonic_ns() - self.logMonoTime) / 1e9 > 3.0:
      self.slowing_down = False
      self.rear_passed = False
      return 0, 0, 0, False, ''

    is_highway = self.naviData.isHighway
    cam_type = int(self.naviData.camType)
    cam_limit_speed_left_dist = self.naviData.camLimitSpeedLeftDist
    cam_limit_speed = self.naviData.camLimitSpeed
    rear_passed = bool(self.naviData.camRearPassed)
    if self.rear_passed and not rear_passed:
      self.slowing_down = False
    self.rear_passed = rear_passed
    section_limit_speed = self.naviData.sectionLimitSpeed
    section_left_dist = self.naviData.sectionLeftDist
    section_avg_speed = self.naviData.sectionAvgSpeed
    section_adjust_speed = self.naviData.sectionAdjustSpeed

    if is_highway is not None:
      if is_highway:
        min_limit = 40
        max_limit = 120
      else:
        min_limit = 20
        max_limit = 100
    else:
      min_limit = 10
      max_limit = 120

    if cam_type == 22:
      min_limit = 10

    if cam_limit_speed_left_dist > 0 and cam_limit_speed <= 0:
      self.slowing_down = False
      self.started_dist = 0
      self.last_limit_speed_left_dist = 0
      return 0, 0, cam_limit_speed_left_dist, False, ''

    if cam_limit_speed_left_dist > 0 and cam_limit_speed > 0:
      # Past a rear camera the distance counts down the hold zone, so hold the limit instead of re-approaching.
      if rear_passed and min_limit <= cam_limit_speed <= max_limit:
        first_started = not self.slowing_down
        self.slowing_down = True
        self.last_limit_speed_left_dist = cam_limit_speed_left_dist
        return cam_limit_speed * CAMERA_SPEED_FACTOR, cam_limit_speed, cam_limit_speed_left_dist, first_started, ''

      v_ego = max(float(CS.vEgo), 0.0)
      diff_speed = v_ego * CV.MS_TO_KPH - (cam_limit_speed * CAMERA_SPEED_FACTOR)
      safe_dist = v_ego * (4.0 if cam_type == 22 else 7.0)
      starting_dist = v_ego * (8.0 if cam_type == 22 else 30.0)

      if self.slowing_down and self.last_limit_speed_left_dist - cam_limit_speed_left_dist < -(v_ego * 5):
        self.slowing_down = False

      if min_limit <= cam_limit_speed <= max_limit and (self.slowing_down or cam_limit_speed_left_dist < starting_dist):
        first_started = not self.slowing_down
        if first_started:
          self.started_dist = cam_limit_speed_left_dist
          self.slowing_down = True

        td = self.started_dist - safe_dist
        d = cam_limit_speed_left_dist - safe_dist
        if d > 0.0 and td > 0.0 and diff_speed > 0.0 and (section_left_dist < 10 or cam_type in SECTION_SMOOTH_CAMERA_TYPES):
          pp = (d / td) ** 0.6
        else:
          pp = 0.0

        self.last_limit_speed_left_dist = cam_limit_speed_left_dist
        return cam_limit_speed * CAMERA_SPEED_FACTOR + int(pp * diff_speed), cam_limit_speed, cam_limit_speed_left_dist, first_started, ''

      self.slowing_down = False
      return 0, cam_limit_speed, cam_limit_speed_left_dist, False, ''

    if section_left_dist > 0 and section_limit_speed <= 0:
      self.slowing_down = False
      self.started_dist = 0
      self.last_limit_speed_left_dist = 0
      return 0, 0, section_left_dist, False, ''

    if section_left_dist > 0 and section_limit_speed > 0:
      if min_limit <= section_limit_speed <= max_limit:
        first_started = not self.slowing_down
        self.slowing_down = True
        speed_diff = 0.0
        if section_adjust_speed:
          speed_diff = (section_limit_speed - section_avg_speed) / 2.0
          speed_diff *= float(np.interp(section_left_dist, [500, 1000], [0.0, 1.0]))
        return section_limit_speed * CAMERA_SPEED_FACTOR + speed_diff, section_limit_speed, section_left_dist, first_started, ''

      self.slowing_down = False
      return 0, section_limit_speed, section_left_dist, False, ''

    self.slowing_down = False
    return 0, 0, 0, False, ''


def signal_handler(sig, frame):
  terminate_flag.set()
  sys.exit(0)


if __name__ == '__main__':
  signal.signal(signal.SIGINT, signal_handler)
  main()
