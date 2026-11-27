"""MyID4 local cloud (Tailscale-only replacement for the Wayon Cloud Worker), against a real aiohttp server."""
import asyncio
import json

import pytest

aiohttp = pytest.importorskip("aiohttp")
from aiohttp.test_utils import TestClient, TestServer

from openpilot.system import wayon_local_cloud as wlc

DEVICE_TOKEN = "device-token"


class FakePush:
  configured = True

  def __init__(self):
    self.sent = []

  def send(self, data):
    self.sent.append(data)
    return 1


@pytest.fixture
def cloud(tmp_path):
  cloud_config = tmp_path / "wayon_cloud.json"
  cloud_config.write_text(json.dumps({"endpoint": "http://127.0.0.1:8770", "token": DEVICE_TOKEN}))
  return wlc.LocalCloud(root=tmp_path / "local", cloud_config=cloud_config, push=FakePush())


def run(cloud, scenario):
  async def go():
    async with TestClient(TestServer(wlc.create_app(cloud))) as client:
      await scenario(client)
  asyncio.run(go())


def auth(token):
  return {"Authorization": f"Bearer {token}"}


def telemetry(locked=None, onroad=0, lat=37.5, lon=127.0, updated="2026-11-27T01:00:00Z", **vehicle):
  if locked is not None:
    vehicle["door_locked"] = locked
  gps = {"latitude": lat, "longitude": lon, "speedMps": 3.0} if lat is not None else {}
  return {"deviceId": "dev", "updatedAt": updated, "onroad": onroad, "vehicle": vehicle, "gps": gps,
          "vehicleSpeedMps": 12.5}


def test_tokens_are_generated_once_and_kept(cloud, tmp_path):
  again = wlc.LocalCloud(root=tmp_path / "local", cloud_config=cloud.cloud_config, push=FakePush())
  assert again.config["view_token"] == cloud.config["view_token"]
  assert len(cloud.config["view_token"]) >= 24 and cloud.config["view_token"] != cloud.config["live_token"]


def test_auth_and_cors(cloud):
  async def scenario(client):
    assert (await client.get("/api/state")).status == 401
    assert (await client.get("/api/state", headers=auth("wrong"))).status == 401
    # The app token can read but cannot pose as the device.
    assert (await client.post("/api/telemetry", json=telemetry(), headers=auth(cloud.config["view_token"]))).status == 401
    preflight = await client.options("/api/state")
    assert preflight.status == 204 and preflight.headers["Access-Control-Allow-Origin"] == "*"
    assert "Authorization" in preflight.headers["Access-Control-Allow-Headers"]
    r = await client.get("/api/state", headers=auth(cloud.config["live_token"]))
    assert r.status == 200 and r.headers["Access-Control-Allow-Origin"] == "*"
    assert (await client.get("/healthz")).status == 200
  run(cloud, scenario)


def test_telemetry_becomes_worker_state(cloud):
  async def scenario(client):
    r = await client.post("/api/telemetry", json=telemetry(aux_voltage=12.6, onroad=1), headers=auth(DEVICE_TOKEN))
    assert r.status == 200
    body = await (await client.get("/api/state", headers=auth(cloud.config["view_token"]))).json()
    state = body["state"]
    assert state["onroad"] == 1 and state["voltage_v"] == 12.6 and state["speed_mps"] == 12.5
    assert (state["latitude"], state["longitude"]) == (37.5, 127.0)
    assert state["raw_json"]["vehicle"]["aux_voltage"] == 12.6
    # A later sample without a fix keeps the last known position; parked speed is 0.
    await client.post("/api/telemetry", json=telemetry(lat=None, onroad=0), headers=auth(DEVICE_TOKEN))
    state = (await (await client.get("/api/json", headers=auth(cloud.config["view_token"]))).json())["state"]
    assert (state["latitude"], state["longitude"], state["speed_mps"]) == (37.5, 127.0, 0.0)
  run(cloud, scenario)


def test_lock_change_pushes_and_first_observation_does_not(cloud):
  cloud.put_telemetry(telemetry(locked=True))
  assert cloud.lock["locked"] is True

  async def scenario(client):
    await client.post("/api/telemetry", json=telemetry(locked=True), headers=auth(DEVICE_TOKEN))
    await client.post("/api/telemetry", json=telemetry(locked=False, updated="2026-11-27T01:05:00Z"),
                      headers=auth(DEVICE_TOKEN))
    await asyncio.sleep(0.05)
    lock = (await (await client.get("/api/state", headers=auth(cloud.config["view_token"]))).json())["vehicleLock"]
    assert lock == {"locked": False, "occurredAt": "2026-11-27T01:05:00Z"}
  run(cloud, scenario)
  assert [m["type"] for m in cloud.push.sent] == ["wayon_door_lock"]
  assert cloud.push.sent[0]["locked"] == "false" and cloud.push.sent[0]["vehicleEventId"]


def test_parking_unlock_alert_once_per_unlock(cloud):
  cloud.put_telemetry(telemetry(locked=True), now=0.0)
  cloud.put_telemetry(telemetry(locked=False), now=100.0)
  delay = cloud.config["parking_unlock_delay_s"]
  assert cloud.parking_alert_due(now=100.0 + delay - 1) is None
  alert = cloud.parking_alert_due(now=100.0 + delay + 1)
  assert alert["type"] == "wayon_parking_unlocked" and alert["delaySeconds"] == delay and alert["latitude"] == 37.5
  assert cloud.parking_alert_due(now=100.0 + delay * 3) is None
  # Driving off unlocked is not an alert.
  cloud.put_telemetry(telemetry(locked=True), now=500.0)
  cloud.put_telemetry(telemetry(locked=False, onroad=1), now=600.0)
  assert cloud.parking_alert_due(now=600.0 + delay * 2) is None


def test_trips_roundtrip(cloud):
  route = [{"latitude": 37.0, "longitude": 127.0, "speedMps": 5.0, "t": "x"},
           {"latitude": 37.01, "longitude": 127.01, "speedMps": 15.0, "t": "y"}]

  async def scenario(client):
    for i, started in enumerate(("2026-11-25T01:00:00Z", "2026-11-26T01:00:00Z")):
      r = await client.post("/api/trips", headers=auth(DEVICE_TOKEN), json={
        "id": f"trip-{i}", "deviceId": "dev", "startedAt": started, "endedAt": started,
        "durationS": 600, "distanceM": 1500.0, "route": route})
      assert r.status == 200
    trips = (await (await client.get("/api/trips?limit=1000", headers=auth(cloud.config["view_token"]))).json())["trips"]
    assert [t["id"] for t in trips] == ["trip-1", "trip-0"]
    assert "route" not in trips[0] and trips[0]["route_point_count"] == 2
    assert trips[0]["max_speed_mps"] == 15.0 and trips[0]["avg_speed_mps"] == 2.5
    assert (trips[0]["start_lat"], trips[0]["end_lon"]) == (37.0, 127.01)
    detail = await (await client.get("/api/trips/trip-0", headers=auth(cloud.config["view_token"]))).json()
    assert detail["route"] == route
    assert (await client.get("/api/trips/..%2Fconfig", headers=auth(cloud.config["view_token"]))).status == 404
  run(cloud, scenario)


def test_live_captures_single_and_chunked(cloud):
  async def scenario(client):
    r = await client.post("/api/live-capture", data=b"\xff\xd8jpeg", headers={
      **auth(DEVICE_TOKEN), "Content-Type": "image/jpeg", "X-Wayon-Capture-Kind": "photo",
      "X-Wayon-Captured-At": "2026-11-27T02:00:00Z", "X-Wayon-Camera-Layout": "dual"})
    photo_id = (await r.json())["id"]
    for index, part in enumerate((b"zip-", b"data")):
      await client.post("/api/live-capture-part", data=part, headers={
        **auth(DEVICE_TOKEN), "X-Wayon-Upload-Id": "u1", "X-Wayon-Part-Index": str(index), "X-Wayon-Part-Count": "2"})
    r = await client.post("/api/live-capture-commit", headers=auth(DEVICE_TOKEN), json={
      "uploadId": "u1", "partCount": 2, "capturedAt": "2026-11-27T03:00:00Z", "durationS": 12.5, "cameraLayout": "dual"})
    clip_id = (await r.json())["id"]

    view = auth(cloud.config["view_token"])
    captures = (await (await client.get("/api/live-captures?limit=100", headers=view)).json())["captures"]
    assert [(c["id"], c["kind"]) for c in captures] == [(clip_id, "clip"), (photo_id, "photo")]
    assert captures[0]["durationS"] == 12.5 and captures[0]["sizeBytes"] == 8
    clip = await client.get(f"/api/live-capture?id={clip_id}", headers=view)
    assert await clip.read() == b"zip-data" and clip.headers["Content-Type"] == "application/zip"
    photo = await client.get(f"/api/live-capture?id={photo_id}", headers=view)
    assert photo.headers["Content-Type"] == "image/jpeg"
  run(cloud, scenario)


def test_live_session_bridges_websocket_to_live_server(cloud, monkeypatch):
  received = []

  async def fake_live_server(reader, writer):
    received.append(await reader.readexactly(4))   # app heartbeat
    writer.write(b"WLV1-frame")
    await writer.drain()
    received.append(await reader.read(16))          # app binary control
    writer.close()

  async def scenario(client):
    server = await asyncio.start_server(fake_live_server, "127.0.0.1", 0)
    monkeypatch.setattr(wlc, "LIVE_PORT", server.sockets[0].getsockname()[1])
    session = await (await client.post("/api/live/session", headers=auth(cloud.config["live_token"]),
                                       json={"deviceId": "dev"})).json()
    assert session["protocol"] == wlc.LIVE_PROTOCOL and session["websocketUrl"].startswith("ws://")
    path = session["websocketUrl"].split(str(client.server.port), 1)[1]
    async with client.ws_connect(path, protocols=(session["protocol"],)) as ws:
      assert ws.protocol == wlc.LIVE_PROTOCOL
      await ws.send_str("WLP1")
      msg = await ws.receive(timeout=5)
      assert msg.data == b"WLV1-frame"
      await ws.send_bytes(b"WLC1ctl")
      await ws.receive(timeout=5)
    # A ticket works once.
    replay = await client.get(path)
    assert replay.status == 401
    server.close()
  run(cloud, scenario)
  assert received == [b"WLP1", b"WLC1ctl"]


def test_push_register_and_empty_lists(cloud):
  async def scenario(client):
    r = await client.post("/api/push/register", headers=auth(cloud.config["live_token"]),
                          json={"fcmToken": "fcm-abc", "deviceId": "dev", "platform": "android"})
    assert r.status == 200
    for path, key in (("/api/impacts?limit=1", "impacts"), ("/api/snapshots?limit=1000", "snapshots")):
      assert (await (await client.get(path, headers=auth(cloud.config["view_token"]))).json()) == {key: []}
  run(cloud, scenario)
  assert "fcm-abc" in json.loads((cloud.root / "push_tokens.json").read_text())


def test_ws_url_is_secure_behind_tailscale_serve():
  class Req:
    def __init__(self, host, headers=None, secure=False):
      self.host, self.headers, self.secure = host, headers or {}, secure
  assert wlc._ws_base(Req("mici.tail1234.ts.net")) == "wss://mici.tail1234.ts.net"
  assert wlc._ws_base(Req("100.64.0.5:8770", {"X-Forwarded-Proto": "https"})) == "wss://100.64.0.5:8770"
  assert wlc._ws_base(Req("127.0.0.1:8770")) == "ws://127.0.0.1:8770"
