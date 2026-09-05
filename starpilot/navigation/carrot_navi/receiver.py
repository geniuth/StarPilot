#!/usr/bin/env python3
"""Receives Carrot Navi data and publishes it as carrotNaviState.

Runs two listeners, because the two feeds are alternatives rather than a sequence:

  7713   HTTP. POST /api/navi/<tmap_version> with the legacy flat rgdata body, and
         POST /api/navi/route with a polyline.
  7714   WebSocket item stream at /api/navi/ws/v2/json/<session_id>/<item>.

carrot's own notes say the two ports are not meant to be used at the same time, and this
keeps that property honest rather than letting the last writer win: whichever feed spoke
most recently owns the published state, and the other is ignored until it goes quiet.

The WebSocket side speaks enough of RFC 6455 to read text frames. That avoids adding a
websocket dependency for one consumer, at the cost of not supporting extensions, which
the Carrot Navi app does not ask for.
"""
import base64
import hashlib
import json
import socket
import struct
import threading
import time

from openpilot.starpilot.navigation.carrot_navi.protocol import (
  CarrotNaviV2,
  NaviData,
  parse_7713,
  parse_7713_route,
)


def _log():
  """cereal and swaglog are only needed when actually running.

  Importing them at module scope would make the framing and arbitration logic below
  untestable anywhere the capnp schemas cannot load, which is most development machines.
  """
  try:
    from openpilot.common.swaglog import cloudlog
    return cloudlog
  except Exception:
    import logging
    return logging.getLogger("carrot_navi")

HTTP_PORT = 7713
WS_PORT = 7714
PUBLISH_HZ = 10.0
# How long a feed keeps ownership after its last message before the other may take over
FEED_OWNERSHIP_TIMEOUT = 3.0
# Drop the whole state if nothing has arrived for this long
CONNECTION_TIMEOUT = 10.0
WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
MAX_BODY = 4 * 1024 * 1024


class NaviStore:
  """The published state, and which feed currently owns it."""

  def __init__(self):
    self.lock = threading.Lock()
    self.data = NaviData()
    self.v2 = CarrotNaviV2()
    self.owner = ""
    self.last_update = 0.0

  def _may_write(self, source, now):
    if self.owner in ("", source):
      return True
    return now - self.last_update > FEED_OWNERSHIP_TIMEOUT

  def put(self, source, data):
    now = time.monotonic()
    with self.lock:
      if not self._may_write(source, now):
        return False
      self.owner = source
      self.last_update = now
      self.data = data
      return True

  def snapshot(self):
    now = time.monotonic()
    with self.lock:
      if self.last_update and now - self.last_update > CONNECTION_TIMEOUT:
        self.data = NaviData()
        self.owner = ""
        self.last_update = 0.0
      return self.data


def _read_http_request(conn):
  """Read one HTTP request. Returns (path, body) or None."""
  buf = b""
  while b"\r\n\r\n" not in buf:
    chunk = conn.recv(4096)
    if not chunk:
      return None
    buf += chunk
    if len(buf) > MAX_BODY:
      return None

  head, _, rest = buf.partition(b"\r\n\r\n")
  lines = head.decode("latin-1", "replace").split("\r\n")
  if not lines:
    return None
  parts = lines[0].split()
  if len(parts) < 2:
    return None
  path = parts[1]

  length = 0
  headers = {}
  for line in lines[1:]:
    k, _, v = line.partition(":")
    headers[k.strip().lower()] = v.strip()
  try:
    length = int(headers.get("content-length", "0"))
  except ValueError:
    length = 0
  length = min(length, MAX_BODY)

  body = rest
  while len(body) < length:
    chunk = conn.recv(min(65536, length - len(body)))
    if not chunk:
      break
    body += chunk
  return path, body[:length], headers


def serve_7713(store, stop):
  srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
  srv.bind(("0.0.0.0", HTTP_PORT))
  srv.listen(8)
  srv.settimeout(0.5)
  _log().info("carrot navi: listening for legacy HTTP on %d", HTTP_PORT)

  while not stop.is_set():
    try:
      conn, _ = srv.accept()
    except socket.timeout:
      continue
    except OSError:
      break

    try:
      conn.settimeout(2.0)
      req = _read_http_request(conn)
      if req is None:
        continue
      path, body, _ = req
      try:
        payload = json.loads(body.decode("utf-8", "replace")) if body else {}
      except ValueError:
        payload = {}

      if isinstance(payload, dict):
        if "route" in payload or "vrtx" in payload:
          points = parse_7713_route(payload)
          if points:
            with store.lock:
              store.data.route_points = points
        parsed = parse_7713(payload, previous=store.data)
        if parsed is not None:
          parsed.route_points = store.data.route_points
          store.put("7713", parsed)

      conn.sendall(b"HTTP/1.1 200 OK\r\nContent-Length: 2\r\n"
                   b"Content-Type: application/json\r\n\r\n{}")
    except Exception:
      _log().exception("carrot navi: 7713 request failed")
    finally:
      try:
        conn.close()
      except OSError:
        pass

  srv.close()


def _ws_handshake(conn, head):
  key = ""
  for line in head.split("\r\n")[1:]:
    k, _, v = line.partition(":")
    if k.strip().lower() == "sec-websocket-key":
      key = v.strip()
  if not key:
    return False
  accept = base64.b64encode(hashlib.sha1((key + WS_GUID).encode()).digest()).decode()
  conn.sendall(("HTTP/1.1 101 Switching Protocols\r\n"
                "Upgrade: websocket\r\nConnection: Upgrade\r\n"
                f"Sec-WebSocket-Accept: {accept}\r\n\r\n").encode())
  return True


def _ws_recv_frames(conn, buf):
  """Yield complete text payloads from whatever is buffered, returning the remainder."""
  out = []
  while True:
    if len(buf) < 2:
      break
    b0, b1 = buf[0], buf[1]
    opcode = b0 & 0x0F
    masked = bool(b1 & 0x80)
    length = b1 & 0x7F
    idx = 2
    if length == 126:
      if len(buf) < idx + 2:
        break
      length = struct.unpack(">H", buf[idx:idx + 2])[0]
      idx += 2
    elif length == 127:
      if len(buf) < idx + 8:
        break
      length = struct.unpack(">Q", buf[idx:idx + 8])[0]
      idx += 8
    if length > MAX_BODY:
      return out, b""
    mask = b""
    if masked:
      if len(buf) < idx + 4:
        break
      mask = buf[idx:idx + 4]
      idx += 4
    if len(buf) < idx + length:
      break

    payload = bytearray(buf[idx:idx + length])
    if masked:
      for i in range(length):
        payload[i] ^= mask[i % 4]
    buf = buf[idx + length:]

    if opcode == 0x8:      # close
      return out, None
    if opcode in (0x1, 0x2):
      out.append(bytes(payload))
  return out, buf


def serve_7714(store, stop):
  srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
  srv.bind(("0.0.0.0", WS_PORT))
  srv.listen(8)
  srv.settimeout(0.5)
  _log().info("carrot navi: listening for v2 stream on %d", WS_PORT)

  while not stop.is_set():
    try:
      conn, _ = srv.accept()
    except socket.timeout:
      continue
    except OSError:
      break
    threading.Thread(target=_handle_ws_client, args=(conn, store, stop), daemon=True).start()

  srv.close()


def _handle_ws_client(conn, store, stop):
  try:
    conn.settimeout(1.0)
    head = b""
    while b"\r\n\r\n" not in head:
      chunk = conn.recv(4096)
      if not chunk:
        return
      head += chunk
      if len(head) > 65536:
        return
    header_text = head.split(b"\r\n\r\n")[0].decode("latin-1", "replace")
    if "upgrade: websocket" not in header_text.lower():
      conn.sendall(b"HTTP/1.1 400 Bad Request\r\nContent-Length: 0\r\n\r\n")
      return
    if not _ws_handshake(conn, header_text):
      return

    buf = head.split(b"\r\n\r\n", 1)[1]
    while not stop.is_set():
      frames, buf = _ws_recv_frames(conn, buf)
      if buf is None:
        return
      for payload in frames:
        try:
          envelope = json.loads(payload.decode("utf-8", "replace"))
        except ValueError:
          continue
        if not isinstance(envelope, dict):
          continue
        data = store.v2.update(envelope)
        if data is not None:
          store.put("7714", data)
      try:
        chunk = conn.recv(65536)
      except socket.timeout:
        continue
      if not chunk:
        return
      buf += chunk
  except Exception:
    _log().exception("carrot navi: 7714 client failed")
  finally:
    try:
      conn.close()
    except OSError:
      pass


def fill_message(msg, data, active, route_curve_speed):
  s = msg.carrotNaviState
  s.active = active
  s.connected = data.connected
  s.source = data.source
  s.sessionId = data.session_id
  s.offRoute = data.off_route
  s.roadLimitSpeed = float(data.road_limit_kph)
  s.roadCategory = int(data.road_category)
  s.roadName = data.road_name
  s.sdiType = int(data.sdi_type)
  s.sdiSpeedLimit = float(data.sdi_speed_limit)
  s.sdiDistance = float(data.sdi_distance)
  s.sdiBlockType = int(data.sdi_block_type)
  s.sdiBlockSpeed = float(data.sdi_block_speed)
  s.sdiBlockDistance = float(data.sdi_block_distance)
  s.sdiPlusType = int(data.sdi_plus_type)
  s.sdiPlusSpeedLimit = float(data.sdi_plus_speed_limit)
  s.sdiPlusDistance = float(data.sdi_plus_distance)
  s.tbtTurnType = int(data.tbt_turn_type)
  s.tbtDistance = float(data.tbt_distance)
  s.tbtTurnTypeNext = int(data.tbt_turn_type_next)
  s.tbtDistanceNext = float(data.tbt_distance_next)
  s.tbtNextRoadWidth = float(data.tbt_next_road_width)
  s.latitude = float(data.latitude)
  s.longitude = float(data.longitude)
  s.bearing = float(data.bearing)
  s.routeSequence = int(data.route_sequence)
  s.goPosDistance = float(data.go_pos_distance)
  s.goPosTime = float(data.go_pos_time)
  flat = []
  for lon, lat in data.route_points:
    flat.extend((float(lon), float(lat)))
  s.routePoints = flat
  s.routeCurveSpeed = float(route_curve_speed)
  return msg


def activity_level(data):
  """carrot's active_carrot, as far as the receiver can tell without the control loop."""
  if not data.connected:
    return 0
  if data.sdi_type == 22:
    return 5
  if data.sdi_block_type in (2, 3):
    return 4
  if data.sdi_type >= 0 and data.sdi_speed_limit > 0:
    return 3
  return 2


def main():
  from cereal import messaging
  from openpilot.common.params import Params
  from openpilot.starpilot.navigation.carrot_navi.route import NO_LIMIT, route_curve_speed
  from openpilot.starpilot.navigation.carrot_navi.speed import NaviSpeedConfig

  store = NaviStore()
  stop = threading.Event()

  threading.Thread(target=serve_7713, args=(store, stop), daemon=True).start()
  threading.Thread(target=serve_7714, args=(store, stop), daemon=True).start()

  params = Params()
  pm = messaging.PubMaster(["carrotNaviState"])
  sm = messaging.SubMaster(["carState"])
  rk_period = 1.0 / PUBLISH_HZ

  cfg = NaviSpeedConfig.from_params(params)
  route_index = 0
  frame = 0

  while True:
    sm.update(0)
    if frame % 100 == 0:      # pick up param edits without a restart
      cfg = NaviSpeedConfig.from_params(params)
    frame += 1

    data = store.snapshot()

    curve_speed = 0.0
    if data.route_points and (data.latitude or data.longitude):
      v_ego_kph = sm["carState"].vEgo * 3.6
      try:
        curve_speed, route_index = route_curve_speed(
          data.route_points, (data.longitude, data.latitude), data.bearing,
          v_ego_kph, data.road_limit_kph, cfg.decel_rate, cfg.ctrl_end, route_index)
        if curve_speed >= NO_LIMIT:
          curve_speed = 0.0
      except Exception:
        _log().exception("carrot navi: route curvature failed")
        curve_speed = 0.0
    else:
      route_index = 0

    msg = messaging.new_message("carrotNaviState", valid=True)
    fill_message(msg, data, activity_level(data), curve_speed)
    pm.send("carrotNaviState", msg)
    time.sleep(rk_period)


if __name__ == "__main__":
  main()
