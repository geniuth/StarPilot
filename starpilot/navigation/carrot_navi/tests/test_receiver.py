"""Tests for the Carrot Navi receiver's framing and feed arbitration.

The socket servers are exercised for real over loopback, because the parts most likely to
be wrong are the HTTP body handling and the hand-rolled WebSocket framing, and neither is
visible from a unit test of the parsers.
"""
import json
import socket
import struct
import threading
import time

import pytest

from openpilot.starpilot.navigation.carrot_navi import receiver
from openpilot.starpilot.navigation.carrot_navi.protocol import NaviData
from openpilot.starpilot.navigation.carrot_navi.receiver import (
  NaviStore,
  _read_http_request,
  _ws_recv_frames,
  activity_level,
)


def ws_frame(payload: bytes, opcode=0x1, mask=True):
  """Client-to-server frame, which per RFC 6455 must be masked."""
  head = bytes([0x80 | opcode])
  length = len(payload)
  if length < 126:
    head += bytes([(0x80 if mask else 0) | length])
  elif length < (1 << 16):
    head += bytes([(0x80 if mask else 0) | 126]) + struct.pack(">H", length)
  else:
    head += bytes([(0x80 if mask else 0) | 127]) + struct.pack(">Q", length)
  if not mask:
    return head + payload
  key = b"\x01\x02\x03\x04"
  masked = bytes(b ^ key[i % 4] for i, b in enumerate(payload))
  return head + key + masked


class TestWebSocketFraming:
  def test_single_text_frame(self):
    frames, rest = _ws_recv_frames(None, ws_frame(b"hello"))
    assert frames == [b"hello"]
    assert rest == b""

  def test_two_frames_in_one_read(self):
    frames, _ = _ws_recv_frames(None, ws_frame(b"one") + ws_frame(b"two"))
    assert frames == [b"one", b"two"]

  def test_partial_frame_is_buffered(self):
    whole = ws_frame(b"hello")
    frames, rest = _ws_recv_frames(None, whole[:4])
    assert frames == []
    frames, rest = _ws_recv_frames(None, rest + whole[4:])
    assert frames == [b"hello"]

  def test_extended_length(self):
    payload = b"x" * 300
    frames, _ = _ws_recv_frames(None, ws_frame(payload))
    assert frames == [payload]

  def test_close_frame_ends_the_stream(self):
    frames, rest = _ws_recv_frames(None, ws_frame(b"", opcode=0x8))
    assert rest is None

  def test_oversized_frame_is_refused(self):
    head = bytes([0x81, 127]) + struct.pack(">Q", receiver.MAX_BODY + 1)
    frames, rest = _ws_recv_frames(None, head)
    assert rest == b""


class TestHttpReading:
  def test_reads_path_and_body(self):
    body = json.dumps({"rgdata": {"nRoadLimitSpeed": 50}}).encode()
    raw = (b"POST /api/navi/1 HTTP/1.1\r\nContent-Length: " + str(len(body)).encode()
           + b"\r\n\r\n" + body)
    a, b = socket.socketpair()
    try:
      a.sendall(raw)
      path, got, _ = _read_http_request(b)
      assert path == "/api/navi/1"
      assert json.loads(got)["rgdata"]["nRoadLimitSpeed"] == 50
    finally:
      a.close()
      b.close()

  def test_body_split_across_reads(self):
    body = b'{"rgdata": {"nRoadLimitSpeed": 50}}'
    a, b = socket.socketpair()
    try:
      a.sendall(b"POST /x HTTP/1.1\r\nContent-Length: " + str(len(body)).encode() + b"\r\n\r\n")
      a.sendall(body[:10])
      threading.Timer(0.05, lambda: a.sendall(body[10:])).start()
      path, got, _ = _read_http_request(b)
      assert got == body
    finally:
      a.close()
      b.close()


class TestStoreArbitration:
  def test_first_feed_takes_ownership(self):
    s = NaviStore()
    assert s.put("7713", NaviData(connected=True, source="7713"))
    assert s.owner == "7713"

  def test_other_feed_is_ignored_while_the_owner_is_live(self):
    # carrot's docs say the two ports are not meant to run together, and that last-writer
    # -wins between them is a real defect. Ownership makes that explicit instead.
    s = NaviStore()
    s.put("7713", NaviData(connected=True, source="7713", road_limit_kph=50))
    assert not s.put("7714", NaviData(connected=True, source="7714", road_limit_kph=90))
    assert s.data.road_limit_kph == 50

  def test_other_feed_takes_over_once_the_owner_goes_quiet(self):
    s = NaviStore()
    s.put("7713", NaviData(connected=True, source="7713"))
    s.last_update = time.monotonic() - receiver.FEED_OWNERSHIP_TIMEOUT - 1.0
    assert s.put("7714", NaviData(connected=True, source="7714", road_limit_kph=90))
    assert s.owner == "7714"

  def test_state_is_dropped_when_everything_goes_quiet(self):
    s = NaviStore()
    s.put("7713", NaviData(connected=True, source="7713", road_limit_kph=50))
    s.last_update = time.monotonic() - receiver.CONNECTION_TIMEOUT - 1.0
    assert not s.snapshot().connected


class TestActivityLevel:
  def test_disconnected(self):
    assert activity_level(NaviData()) == 0

  def test_connected_but_idle(self):
    assert activity_level(NaviData(connected=True)) == 2

  def test_camera(self):
    assert activity_level(NaviData(connected=True, sdi_type=1, sdi_speed_limit=50)) == 3

  def test_section(self):
    assert activity_level(NaviData(connected=True, sdi_type=1, sdi_speed_limit=50,
                                   sdi_block_type=2)) == 4

  def test_bump(self):
    assert activity_level(NaviData(connected=True, sdi_type=22)) == 5


class TestLiveHttpServer:
  def test_accepts_a_documented_post(self):
    store = NaviStore()
    stop = threading.Event()
    t = threading.Thread(target=receiver.serve_7713, args=(store, stop), daemon=True)
    t.start()
    try:
      deadline = time.time() + 5.0
      body = json.dumps({"rgdata": {"nRoadLimitSpeed": 50, "nSdiType": 1,
                                    "nSdiSpeedLimit": 50, "nSdiDist": 420,
                                    "roadcate": 8}}).encode()
      while time.time() < deadline:
        try:
          c = socket.create_connection(("127.0.0.1", receiver.HTTP_PORT), timeout=2.0)
        except OSError:
          time.sleep(0.05)
          continue
        with c:
          c.sendall(b"POST /api/navi/1 HTTP/1.1\r\nContent-Length: "
                    + str(len(body)).encode() + b"\r\n\r\n" + body)
          assert b"200 OK" in c.recv(4096)
        break
      else:
        pytest.fail("server never came up")

      for _ in range(50):
        if store.data.connected:
          break
        time.sleep(0.02)
      assert store.data.connected
      assert store.data.sdi_distance == 420.0
      assert store.data.road_category == 8
    finally:
      stop.set()
      t.join(timeout=3.0)
