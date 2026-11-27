#!/usr/bin/env python3
"""MyID4 local cloud: the Wayon Cloud Worker API served by the comma itself, reached over Tailscale.

Nothing leaves the car. The device's own telemetry, trip and live-capture uploaders keep their
Worker protocol and simply post to this server on loopback (/data/wayon_cloud/config.json
"endpoint": "http://127.0.0.1:8770"). The MyID4 app reaches the same server through
`tailscale serve` (https://<device>.<tailnet>.ts.net), so the app only needs a new address and tokens.

Endpoints (same shapes the app already parses):
  device  POST /api/telemetry, POST /api/trips, POST /api/live-capture[-part|-commit]   (device token)
  app     GET  /api/json, /api/state, /api/trips, /api/trips/<id>, /api/live-captures,
               /api/live-capture?id=, /api/snapshots, /api/snapshot, /api/impacts
          POST /api/live/session, /api/push/register; WS /api/live/stream                (view or live token)

Live view: the app's WebSocket is bridged to the local live server (raw TCP, wayon_live_stream).
Push: door lock changes and "parked and unlocked for N minutes" go straight to FCM HTTP v1 when a
Firebase service account file is present (/data/wayon_local/fcm-service-account.json).

Everything is stored under /data/wayon_local.
"""
import asyncio
import hmac
import json
import os
import secrets
import shutil
import threading
import time
import uuid
from datetime import UTC, datetime
from pathlib import Path

from aiohttp import WSMsgType, web

LOCAL_DIR = Path(os.getenv("WAYON_LOCAL_DIR", "/data/wayon_local"))
CLOUD_CONFIG = Path(os.getenv("WAYON_CLOUD_CONFIG", "/data/wayon_cloud/config.json"))
LISTEN_HOST = os.getenv("WAYON_LOCAL_HOST", "127.0.0.1")
LISTEN_PORT = int(os.getenv("WAYON_LOCAL_PORT", "8770"))
LIVE_HOST = os.getenv("WAYON_LIVE_CONNECT_HOST", "127.0.0.1")
LIVE_PORT = int(os.getenv("WAYON_LIVE_PORT", "8765"))

LIVE_PROTOCOL = "wayon-live.v1"
LIVE_TICKET_TTL_S = 60.0
JSON_TRIPS = 30                      # trips embedded in /api/json; the app loads the rest from /api/trips
MAX_UPLOAD_BYTES = 64 * 1024 * 1024
CAPTURE_DIR_LIMIT_BYTES = 2 * 1024 ** 3
DEFAULT_PARKING_UNLOCK_DELAY_S = 180
PARKING_CHECK_S = 15.0
FCM_SCOPE = "https://www.googleapis.com/auth/firebase.messaging"
DEVICE_PATHS = ("/api/telemetry", "/api/live-capture", "/api/live-capture-part", "/api/live-capture-commit")


def utc_now() -> str:
  return datetime.now(UTC).strftime("%Y-%m-%dT%H:%M:%SZ")


def _iso_to_epoch(value) -> float | None:
  try:
    return datetime.strptime(str(value), "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=UTC).timestamp()
  except (TypeError, ValueError):
    return None


def _read_json(path: Path, default):
  try:
    with open(path, encoding="utf-8") as f:
      return json.load(f)
  except (FileNotFoundError, ValueError, OSError):
    return default


def _write_json(path: Path, value) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  tmp = path.with_name(f".{path.name}.{os.getpid()}.tmp")
  with open(tmp, "w", encoding="utf-8") as f:
    json.dump(value, f, ensure_ascii=False, separators=(",", ":"))
  os.replace(tmp, path)


def _safe_id(value) -> str | None:
  value = str(value or "")
  return value if value and len(value) <= 80 and all(c.isalnum() or c in "-_" for c in value) else None


def _num(value) -> float | None:
  try:
    value = float(value)
  except (TypeError, ValueError):
    return None
  return value if value == value and abs(value) != float("inf") else None


def trip_record(payload: dict) -> dict:
  """Device trip upload (camelCase) -> the Worker's stored row (snake_case + derived fields)."""
  route = [p for p in (payload.get("route") or []) if isinstance(p, dict)]
  points = [(_num(p.get("latitude")), _num(p.get("longitude"))) for p in route]
  points = [p for p in points if p[0] is not None and p[1] is not None]
  speeds = [s for s in (_num(p.get("speedMps")) for p in route) if s is not None]
  duration_s = int(_num(payload.get("durationS")) or 0)
  distance_m = _num(payload.get("distanceM")) or 0.0
  return {
    "id": payload.get("id"),
    "device_id": payload.get("deviceId"),
    "started_at": payload.get("startedAt"),
    "ended_at": payload.get("endedAt"),
    "duration_s": duration_s,
    "distance_m": round(distance_m, 1),
    "avg_speed_mps": round(distance_m / duration_s, 3) if duration_s > 0 else None,
    "max_speed_mps": max(speeds) if speeds else None,
    "start_lat": points[0][0] if points else None,
    "start_lon": points[0][1] if points else None,
    "end_lat": points[-1][0] if points else None,
    "end_lon": points[-1][1] if points else None,
    "route_point_count": len(route),
    "route": route,
  }


def trip_summary(record: dict) -> dict:
  return {k: v for k, v in record.items() if k != "route"}


def state_record(payload: dict, previous: dict | None) -> dict:
  """Telemetry upload -> the Worker's `state` row. Keeps the last known position when a sample has no fix."""
  gps = payload.get("gps") if isinstance(payload.get("gps"), dict) else {}
  vehicle = payload.get("vehicle") if isinstance(payload.get("vehicle"), dict) else {}
  previous = previous or {}
  latitude, longitude = _num(gps.get("latitude")), _num(gps.get("longitude"))
  if latitude is None or longitude is None:
    latitude, longitude = previous.get("latitude"), previous.get("longitude")
  speed = _num(payload.get("vehicleSpeedMps"))
  if speed is None:
    speed = _num(gps.get("speedMps"))
  return {
    "device_id": payload.get("deviceId"),
    "updated_at": payload.get("updatedAt") or utc_now(),
    "onroad": 1 if payload.get("onroad") else 0,
    "voltage_v": _num(vehicle.get("aux_voltage")),
    "latitude": latitude,
    "longitude": longitude,
    "speed_mps": speed if payload.get("onroad") else 0.0,
    "raw_json": payload,
  }


class FcmSender:
  """FCM HTTP v1 with a Firebase service account. Missing account file = push disabled."""

  def __init__(self, account_path: Path, tokens_path: Path):
    self.account_path = account_path
    self.tokens_path = tokens_path
    self._access_token = None
    self._access_expiry = 0.0
    self._lock = threading.Lock()

  @property
  def configured(self) -> bool:
    return self.account_path.is_file()

  def _token(self, account: dict) -> str:
    import jwt
    import requests
    now = time.time()  # noqa: TID251 JWT iat/exp are wall clock
    if self._access_token and now < self._access_expiry - 60:
      return self._access_token
    assertion = jwt.encode({
      "iss": account["client_email"], "scope": FCM_SCOPE, "aud": account["token_uri"],
      "iat": int(now), "exp": int(now) + 3600,
    }, account["private_key"], algorithm="RS256")
    r = requests.post(account["token_uri"], data={
      "grant_type": "urn:ietf:params:oauth:grant-type:jwt-bearer", "assertion": assertion}, timeout=15)
    r.raise_for_status()
    body = r.json()
    self._access_token = body["access_token"]
    self._access_expiry = now + float(body.get("expires_in", 3600))
    return self._access_token

  def send(self, data: dict) -> int:
    """Blocking. Returns how many registered phones accepted the message."""
    import requests
    if not self.configured:
      return 0
    with self._lock:
      account = _read_json(self.account_path, {})
      tokens = _read_json(self.tokens_path, {})
      if not tokens or not account.get("project_id"):
        return 0
      try:
        access = self._token(account)
      except Exception as e:
        print(f"wayon_local: FCM auth failed: {e}", flush=True)
        return 0
      url = f"https://fcm.googleapis.com/v1/projects/{account['project_id']}/messages:send"
      sent = 0
      for fcm_token in list(tokens):
        message = {"message": {"token": fcm_token, "data": {k: str(v) for k, v in data.items()},
                               "android": {"priority": "high"}}}
        try:
          r = requests.post(url, json=message, headers={"Authorization": f"Bearer {access}"}, timeout=15)
        except requests.RequestException as e:
          print(f"wayon_local: FCM send failed: {e}", flush=True)
          continue
        if r.status_code == 200:
          sent += 1
        elif r.status_code == 404 or "UNREGISTERED" in r.text:
          tokens.pop(fcm_token, None)  # the app was uninstalled or the token rotated
        else:
          print(f"wayon_local: FCM HTTP {r.status_code}: {r.text[:160]}", flush=True)
      _write_json(self.tokens_path, tokens)
      return sent


class LocalCloud:
  def __init__(self, root: Path = LOCAL_DIR, cloud_config: Path = CLOUD_CONFIG, push: FcmSender | None = None):
    self.root = root
    self.cloud_config = cloud_config
    self.config = self._load_config()
    self.state = _read_json(root / "state.json", None)
    self.lock = _read_json(root / "lock.json", {"locked": None, "occurredAt": None, "eventId": None})
    if self.lock.get("unlockedAtMono") is not None:
      self.lock["unlockedAtMono"] = time.monotonic()  # monotonic does not survive a reboot: restart the timer
    self.trips: dict[str, dict] = {}
    for path in (root / "trips").glob("*.json"):
      record = _read_json(path, None)
      if isinstance(record, dict) and record.get("id"):
        self.trips[record["id"]] = trip_summary(record)
    self.captures = {m["id"]: m for m in (_read_json(p, None) for p in (root / "captures").glob("*.json"))
                     if isinstance(m, dict) and m.get("id")}
    self.tickets: dict[str, float] = {}
    self.push = push or FcmSender(root / "fcm-service-account.json", root / "push_tokens.json")
    self.parking_alerted_event = self.lock.get("parkingAlertedEventId")

  # ----- config / auth -----

  def _load_config(self) -> dict:
    path = self.root / "config.json"
    config = _read_json(path, {})
    changed = False
    for key in ("view_token", "live_token"):
      if not config.get(key):
        config[key] = secrets.token_urlsafe(24)
        changed = True
    config.setdefault("parking_unlock_delay_s", DEFAULT_PARKING_UNLOCK_DELAY_S)
    if changed or not path.exists():
      _write_json(path, config)
      os.chmod(path, 0o600)
    return config

  def device_token(self) -> str:
    return str(_read_json(self.cloud_config, {}).get("token") or "")

  def authorized(self, request: web.Request, device: bool) -> bool:
    header = request.headers.get("Authorization", "")
    token = header[7:] if header.startswith("Bearer ") else request.query.get("token", "")
    if not token:
      return False
    allowed = [self.device_token()] if device else \
      [self.config.get("view_token"), self.config.get("live_token"), self.device_token()]
    return any(t and hmac.compare_digest(token, t) for t in allowed)

  # ----- device uploads -----

  def put_telemetry(self, payload: dict, now: float | None = None) -> dict | None:
    """Store a telemetry sample. Returns a lock event if the door lock state changed."""
    self.state = state_record(payload, self.state)
    _write_json(self.root / "state.json", self.state)
    vehicle = payload.get("vehicle") if isinstance(payload.get("vehicle"), dict) else {}
    locked = vehicle.get("door_locked")
    if not isinstance(locked, bool) or locked == self.lock.get("locked"):
      return None
    first = self.lock.get("locked") is None
    self.lock = {"locked": locked, "occurredAt": payload.get("updatedAt") or utc_now(),
                 "eventId": uuid.uuid4().hex, "unlockedAtMono": None if locked else (now or time.monotonic())}
    _write_json(self.root / "lock.json", self.lock)
    return None if first else dict(self.lock)

  def put_trip(self, payload: dict) -> dict:
    record = trip_record(payload)
    trip_id = _safe_id(record.get("id")) or uuid.uuid4().hex
    record["id"] = trip_id
    _write_json(self.root / "trips" / f"{trip_id}.json", record)
    self.trips[trip_id] = trip_summary(record)
    return self.trips[trip_id]

  def put_capture(self, data: bytes, meta: dict) -> dict:
    capture_id = uuid.uuid4().hex
    meta = {**meta, "id": capture_id, "sizeBytes": len(data)}
    directory = self.root / "captures"
    directory.mkdir(parents=True, exist_ok=True)
    with open(directory / f"{capture_id}.bin", "wb") as f:
      f.write(data)
    _write_json(directory / f"{capture_id}.json", meta)
    self.captures[capture_id] = meta
    self._trim_captures()
    return meta

  def _trim_captures(self) -> None:
    oldest_first = sorted(self.captures.values(), key=lambda m: _iso_to_epoch(m.get("capturedAt")) or 0)
    total = sum(int(m.get("sizeBytes") or 0) for m in oldest_first)
    while total > CAPTURE_DIR_LIMIT_BYTES and len(oldest_first) > 1:
      meta = oldest_first.pop(0)
      total -= int(meta.get("sizeBytes") or 0)
      for suffix in (".bin", ".json"):
        (self.root / "captures" / f"{meta['id']}{suffix}").unlink(missing_ok=True)
      self.captures.pop(meta["id"], None)

  # ----- app reads -----

  def vehicle_lock(self) -> dict:
    return {"locked": self.lock.get("locked"), "occurredAt": self.lock.get("occurredAt")}

  def trip_list(self, limit: int) -> list[dict]:
    trips = sorted(self.trips.values(), key=lambda t: _iso_to_epoch(t.get("started_at")) or 0, reverse=True)
    return trips[:max(0, limit)]

  def trip_detail(self, trip_id: str) -> dict | None:
    trip_id = _safe_id(trip_id)
    return _read_json(self.root / "trips" / f"{trip_id}.json", None) if trip_id else None

  def capture_list(self, limit: int) -> list[dict]:
    captures = sorted(self.captures.values(), key=lambda m: _iso_to_epoch(m.get("capturedAt")) or 0, reverse=True)
    return captures[:max(0, limit)]

  # ----- live -----

  def new_ticket(self) -> str:
    now = time.monotonic()
    self.tickets = {t: exp for t, exp in self.tickets.items() if exp > now}
    ticket = secrets.token_urlsafe(18)
    self.tickets[ticket] = now + LIVE_TICKET_TTL_S
    return ticket

  def take_ticket(self, ticket: str) -> bool:
    expiry = self.tickets.pop(ticket, 0.0)
    return expiry > time.monotonic()

  # ----- parking alert -----

  def parking_alert_due(self, now: float | None = None) -> dict | None:
    """Parked, unlocked and nobody locked it for the configured delay: one alert per unlock event."""
    now = time.monotonic() if now is None else now
    lock, state = self.lock, self.state or {}
    if lock.get("locked") is not False or state.get("onroad") or lock.get("unlockedAtMono") is None:
      return None
    if lock.get("eventId") == self.parking_alerted_event:
      return None
    delay = int(self.config.get("parking_unlock_delay_s") or DEFAULT_PARKING_UNLOCK_DELAY_S)
    if now - float(lock["unlockedAtMono"]) < delay:
      return None
    self.parking_alerted_event = lock.get("eventId")
    self.lock["parkingAlertedEventId"] = self.parking_alerted_event
    _write_json(self.root / "lock.json", self.lock)
    return {"type": "wayon_parking_unlocked", "vehicleEventId": lock.get("eventId"), "delaySeconds": delay,
            "latitude": state.get("latitude") or "", "longitude": state.get("longitude") or ""}


def _limit(request: web.Request, default: int) -> int:
  try:
    return max(1, min(int(request.query.get("limit", default)), 5000))
  except ValueError:
    return default


def _ws_base(request: web.Request) -> str:
  secure = request.headers.get("X-Forwarded-Proto", "").lower() == "https" or request.secure or \
    request.host.split(":")[0].endswith(".ts.net")
  return f"{'wss' if secure else 'ws'}://{request.host}"


def create_app(cloud: LocalCloud) -> web.Application:
  @web.middleware
  async def cors_and_auth(request: web.Request, handler):
    if request.method == "OPTIONS":
      response = web.Response(status=204)
    elif request.path == "/healthz":
      response = await handler(request)
    else:
      device = request.method == "POST" and (request.path in DEVICE_PATHS or request.path == "/api/trips")
      if request.path == "/api/live/stream":
        response = await handler(request)  # authorized by the one-time ticket
      elif not cloud.authorized(request, device):
        response = web.json_response({"error": "unauthorized"}, status=401)
      else:
        response = await handler(request)
    if not isinstance(response, web.WebSocketResponse):
      response.headers["Access-Control-Allow-Origin"] = "*"
      response.headers["Access-Control-Allow-Headers"] = "Authorization, Content-Type"
      response.headers["Access-Control-Allow-Methods"] = "GET, POST, OPTIONS"
    return response

  def push_async(data: dict) -> None:
    if cloud.push.configured:
      asyncio.get_running_loop().run_in_executor(None, cloud.push.send, data)

  async def healthz(_):
    return web.json_response({"ok": True, "push": cloud.push.configured})

  async def telemetry(request):
    payload = await request.json()
    if not isinstance(payload, dict):
      raise web.HTTPBadRequest()
    event = cloud.put_telemetry(payload)
    if event:
      push_async({"type": "wayon_door_lock", "locked": "true" if event["locked"] else "false",
                  "vehicleEventId": event["eventId"]})
    return web.json_response({"ok": True})

  async def trips(request):
    if request.method == "POST":
      payload = await request.json()
      if not isinstance(payload, dict):
        raise web.HTTPBadRequest()
      return web.json_response({"ok": True, "trip": cloud.put_trip(payload)})
    return web.json_response({"trips": cloud.trip_list(_limit(request, 100))})

  async def trip(request):
    detail = cloud.trip_detail(request.match_info["trip_id"])
    return web.json_response(detail) if detail else web.json_response({"error": "not found"}, status=404)

  async def state(_):
    return web.json_response({"state": cloud.state, "vehicleLock": cloud.vehicle_lock()})

  async def feed(_):
    return web.json_response({"state": cloud.state, "vehicleLock": cloud.vehicle_lock(),
                              "trips": cloud.trip_list(JSON_TRIPS), "snapshots": []})

  async def not_found(_):
    return web.json_response({"error": "not found"}, status=404)

  async def capture(request):
    if request.method == "GET":
      meta = cloud.captures.get(_safe_id(request.query.get("id")) or "")
      if not meta:
        return web.json_response({"error": "not found"}, status=404)
      return web.FileResponse(cloud.root / "captures" / f"{meta['id']}.bin",
                              headers={"Content-Type": meta.get("contentType") or "application/octet-stream"})
    data = await request.read()
    h = request.headers
    duration = _num(h.get("X-Wayon-Duration-S"))
    meta = cloud.put_capture(data, {
      "kind": "clip" if h.get("X-Wayon-Capture-Kind") == "clip" else "photo",
      "capturedAt": h.get("X-Wayon-Captured-At") or utc_now(),
      "cameraLayout": h.get("X-Wayon-Camera-Layout") or "",
      "contentType": h.get("Content-Type") or "application/octet-stream",
      "durationS": duration or 0,
    })
    return web.json_response({"ok": True, "id": meta["id"]})

  async def capture_part(request):
    h = request.headers
    upload_id = _safe_id(h.get("X-Wayon-Upload-Id"))
    try:
      index = int(h.get("X-Wayon-Part-Index", ""))
    except ValueError:
      index = -1
    if not upload_id or index < 0:
      raise web.HTTPBadRequest()
    directory = cloud.root / "uploads" / upload_id
    directory.mkdir(parents=True, exist_ok=True)
    with open(directory / f"{index:05d}", "wb") as f:
      f.write(await request.read())
    return web.json_response({"ok": True})

  async def capture_commit(request):
    body = await request.json()
    upload_id = _safe_id(body.get("uploadId"))
    directory = cloud.root / "uploads" / str(upload_id)
    parts = sorted(directory.glob("*")) if upload_id else []
    if not parts or len(parts) != int(body.get("partCount") or 0):
      raise web.HTTPBadRequest(text="missing parts")
    data = b"".join(p.read_bytes() for p in parts)
    shutil.rmtree(directory, ignore_errors=True)
    meta = cloud.put_capture(data, {
      "kind": "clip", "capturedAt": body.get("capturedAt") or utc_now(),
      "cameraLayout": body.get("cameraLayout") or "", "contentType": "application/zip",
      "durationS": _num(body.get("durationS")) or 0,
    })
    return web.json_response({"ok": True, "id": meta["id"]})

  async def captures(request):
    return web.json_response({"captures": cloud.capture_list(_limit(request, 100))})

  async def live_session(request):
    ticket = cloud.new_ticket()
    return web.json_response({"websocketUrl": f"{_ws_base(request)}/api/live/stream?ticket={ticket}",
                              "protocol": LIVE_PROTOCOL})

  async def live_stream(request):
    if not cloud.take_ticket(request.query.get("ticket", "")):
      return web.json_response({"error": "unauthorized"}, status=401)
    try:
      reader, writer = await asyncio.wait_for(asyncio.open_connection(LIVE_HOST, LIVE_PORT), 5)
    except (TimeoutError, OSError):
      return web.json_response({"error": "live server unavailable"}, status=503)
    ws = web.WebSocketResponse(protocols=(LIVE_PROTOCOL,), max_msg_size=MAX_UPLOAD_BYTES, heartbeat=20)
    await ws.prepare(request)

    async def device_to_app():
      while not ws.closed:
        chunk = await reader.read(65536)
        if not chunk:
          break
        await ws.send_bytes(chunk)

    async def app_to_device():
      async for msg in ws:
        if msg.type == WSMsgType.BINARY:
          writer.write(msg.data)
        elif msg.type == WSMsgType.TEXT:
          writer.write(msg.data.encode())
        else:
          break
        await writer.drain()

    tasks = [asyncio.ensure_future(device_to_app()), asyncio.ensure_future(app_to_device())]
    try:
      await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
    finally:
      for task in tasks:
        task.cancel()
      writer.close()
      await ws.close()
    return ws

  async def push_register(request):
    body = await request.json()
    fcm_token = str(body.get("fcmToken") or "")
    if not fcm_token or len(fcm_token) > 4096:
      raise web.HTTPBadRequest()
    tokens = _read_json(cloud.root / "push_tokens.json", {})
    tokens[fcm_token] = {"deviceId": body.get("deviceId"), "platform": body.get("platform"),
                         "appVersion": body.get("appVersion"), "registeredAt": utc_now()}
    _write_json(cloud.root / "push_tokens.json", tokens)
    return web.json_response({"ok": True, "push": cloud.push.configured})

  async def parking_monitor(app):
    async def loop():
      while True:
        await asyncio.sleep(PARKING_CHECK_S)
        alert = cloud.parking_alert_due()
        if alert:
          push_async(alert)
    task = asyncio.ensure_future(loop())
    yield
    task.cancel()

  app = web.Application(middlewares=[cors_and_auth], client_max_size=MAX_UPLOAD_BYTES)
  app.router.add_route("*", "/healthz", healthz)
  app.router.add_post("/api/telemetry", telemetry)
  app.router.add_route("*", "/api/trips", trips)
  app.router.add_get("/api/trips/{trip_id}", trip)
  app.router.add_get("/api/state", state)
  app.router.add_get("/api/json", feed)
  app.router.add_route("*", "/api/live-capture", capture)
  app.router.add_post("/api/live-capture-part", capture_part)
  app.router.add_post("/api/live-capture-commit", capture_commit)
  app.router.add_get("/api/live-captures", captures)
  app.router.add_post("/api/live/session", live_session)
  app.router.add_get("/api/live/stream", live_stream)
  app.router.add_post("/api/push/register", push_register)
  app.router.add_get("/api/snapshot", not_found)
  # Snapshots and impacts were produced by upstream Wayon modules this device does not run.
  for path, key in (("/api/snapshots", "snapshots"), ("/api/impacts", "impacts")):
    app.router.add_get(path, _empty_handler(key))
  app.cleanup_ctx.append(parking_monitor)
  return app


def _empty_handler(key: str):
  async def handler(_):
    return web.json_response({key: []})
  return handler


def main() -> None:
  LOCAL_DIR.mkdir(parents=True, exist_ok=True)
  os.chmod(LOCAL_DIR, 0o700)
  cloud = LocalCloud()
  print(f"wayon_local: serving on {LISTEN_HOST}:{LISTEN_PORT} (push {'on' if cloud.push.configured else 'off'})", flush=True)
  web.run_app(create_app(cloud), host=LISTEN_HOST, port=LISTEN_PORT, print=None, handle_signals=True)


if __name__ == "__main__":
  main()
