"""외부 HUD 로 가는 블루투스 RFCOMM 전송.

Navdy 는 WiFi 유저스페이스가 펌웨어에서 빠져 있어(wpa_supplicant/hostapd 없음)
UDP 로는 닿을 수 없다. 대신 기기가 안드로이드이고 루팅돼 있어 우리 앱을 올릴 수
있으므로, 그 앱이 RFCOMM 서버로 듣고 콤마가 클라이언트로 붙는다.

프레임 형식은 Navdy 자체 프로토콜과 같은 모양으로 맞춰 둔다(빅엔디안):

    [타입 2B][길이 4B][내용]

    1  텔레메트리 JSON (remote_hud._packet 결과 그대로)
    2  PING

10Hz 송신 루프가 블루투스 지연에 막히면 안 되므로 실제 전송은 별도 스레드에서
하고, 큐에는 최신 패킷 하나만 남긴다(오래된 주행 상태를 뒤늦게 그려봐야 쓸모없다).
"""
import json
import os
import queue
import select
import socket
import struct
import subprocess
import threading
import time

from openpilot.common.swaglog import cloudlog

# 우리 앱이 등록하는 서비스 UUID. 안드로이드 쪽과 반드시 같아야 한다.
SERVICE_UUID = "b8949674-c91b-4c36-a9d0-c24c644826a0"
SERVICE_NAME = "CommaHUD"

TYPE_TELEMETRY = 1
TYPE_PING = 2

BTPROTO_RFCOMM = 3
PING_INTERVAL_S = 5.0
RECONNECT_MIN_S = 2.0
# 시동이 걸린 동안만 찾으므로 오래 쉴 이유가 없다. Navdy 는 콤마보다 늦게 켜진다.
RECONNECT_MAX_S = 10.0
CONNECT_TIMEOUT_S = 12.0
# 프레임 하나를 끝까지 쓰는 데 허용하는 시간. 쓰기 시작한 뒤에는 중간에 끊지 않는다.
SEND_TIMEOUT_S = 10.0
# 이만큼 기다려도 쓸 수 없으면 이번 프레임은 버린다(링크는 유지).
WRITABLE_WAIT_S = 0.3
# 이 시간 내내 한 프레임도 못 보냈으면 링크가 죽은 것으로 본다.
STALL_LIMIT_S = 15.0
# 시동이 꺼진 뒤 이만큼 지나야 링크를 내리고 찾기를 멈춘다.
INACTIVE_GRACE_S = 60.0


def _bonded_devices():
  """본딩된 기기 MAC 목록. bluetoothctl 출력이 유일하게 안정적인 경로다."""
  try:
    out = subprocess.run(["bluetoothctl", "devices", "Paired"],
                         capture_output=True, text=True, timeout=5).stdout
  except (OSError, subprocess.SubprocessError):
    return []
  macs = []
  for line in out.splitlines():
    parts = line.split()
    if len(parts) >= 2 and parts[0] == "Device" and len(parts[1]) == 17:
      macs.append(parts[1])
  return macs


def _service_channel(mac):
  """SDP 로 우리 서비스의 RFCOMM 채널을 찾는다. 없으면 None.

  sdptool 의 --uuid 는 128비트 UUID 문자열을 받지 않는다("Invalid uuid").
  전체 레코드를 훑어 UUID 128 줄이 우리 것인 블록의 Channel 을 쓴다.
  """
  try:
    out = subprocess.run(["sdptool", "browse", mac],
                         capture_output=True, text=True, timeout=40).stdout
  except (OSError, subprocess.SubprocessError):
    return None
  ours = False
  for line in out.splitlines():
    line = line.strip()
    if line.startswith("Service Name:"):
      # 새 레코드 시작. 이름으로도 한 번 걸러 둔다.
      ours = line.split(":", 1)[1].strip() == SERVICE_NAME
    elif line.lower().startswith("uuid 128:"):
      ours = line.split(":", 1)[1].strip().lower() == SERVICE_UUID
    elif line.startswith("Channel:") and ours:
      try:
        return int(line.split(":", 1)[1])
      except ValueError:
        return None
  return None


# HUD 렌더러가 실제로 읽는 항목만. 원본에는 계기판용 필드(문·타이어공기압·
# 주차센서·날씨 등)가 많은데 이 화면에는 안 그린다.
# HUD 렌더러가 실제로 읽는 항목만(나브디 앱 HudView 에서 opt* 로 읽는 키와 맞춘다).
# gap/gear/alert 는 앱이 읽지 않고, edges(도로 경계선)는 앱에서 그리기가 꺼져 있다
# (HudView.SHOW_EDGES=false). 실측(10/27, 74분)에서 프레임당 평균 1.6KB 를 보내다
# 9.5% 를 버렸다 - 블루투스가 다 못 밀어냈다. 안 쓰는 것부터 뺀다.
KEEP_KEYS = (
  "speed", "limit", "set",
  # 인게이지 여부. 경로 띠 색이 여기서 갈린다.
  "enabled",
  "lanes", "path", "lead", "lead2", "others", "laneL", "laneR",
  "leftBsd", "rightBsd", "leftBlinker", "rightBlinker",
  "camera", "cameraDist", "cameraSection", "bumpDist",
  # turnInfo/turnDist(경로 안내 화살표)는 HUD 에서 뺐다. 안 그리는 것은 안 보낸다.
  "vTurnSpeed", "desiredSpeed", "goTime",
  # TMAP 안내의 신호등(켜진 등, 남은 초, 거리). 모델 추정 신호(trafficState)는
  # 실제 신호가 아니라서 HUD 에 띄우지 않는다.
  "tmapSignal",
)
# 검출 차량에서 앱이 쓰는 필드와 소수 자릿수. src/p 는 앱이 읽지 않는다.
VEHICLE_FIELDS = (("d", 1), ("y", 1), ("v", 0), ("a", 1))

# 폴리라인 점 개수. 앱은 점 사이를 0.75m 간격으로 다시 쪼개 그리므로 원래 점이
# 많을 필요가 없다. 그리는 구간(아래)만 남기면 이 정도로 충분하다.
LINE_POINTS = 10
PATH_POINTS = 8
# 앱이 실제로 그리는 범위. 앱 상수와 맞춰 둔다.
#   MIN_DRAW_X  = Projection.MIN_X   (이보다 가까우면 화면 밖)
#   LANE_MAX_X  = Projection.MAX_X   (차선)
#   PATH_MAX_X  = HudView.PATH_MAX_X (경로 띠)
MIN_DRAW_X = 2.0
LANE_MAX_X = 50.0
PATH_MAX_X = 35.0


def _num(value, digits):
  """반올림하고, 정수가 되면 int 로 둔다(JSON 에서 12.0 대신 12)."""
  r = round(float(value), digits)
  return int(r) if r == int(r) else r


def _pt(point):
  """x 는 0.1m, y 는 0.01m. 높이(z) 등 나머지 성분은 앱이 쓰지 않아 버린다."""
  return [_num(point[0], 1), _num(point[1], 2)]


def _thin(points, count=LINE_POINTS, max_x=LANE_MAX_X, margin=True):
  """그리는 구간만 남기고 솎는다. 가까운 쪽을 촘촘히 둔다.

  modelV2 의 x 간격은 앞쪽이 아주 촘촘하고(0, 0.19, 0.75, 1.69) 뒤로 갈수록
  벌어진다. 인덱스 기준으로 솎으면 그 촘촘한 앞부분만 챙기다가 정작 화면에
  보이는 3~7m 구간을 통째로 건너뛴다(실측: 1.69 다음이 6.75 로 점프해서
  화면 아래 절반이 비었다). 그래서 안 보일 점을 먼저 버리고 솎는다.

  margin 이면 범위 바로 바깥 점을 양쪽에 하나씩 남긴다. 앱은 점 사이 선분을
  쪼개 범위 안쪽만 그리므로, 그래야 선이 화면 아래 끝(2m)과 먼 끝까지 닿는다.
  범위 안 점만 남기면 차선이 3m 에서야 시작해 화면 아래가 비었다.
  """
  if not points:
    return points
  inside = [i for i, p in enumerate(points) if MIN_DRAW_X <= p[0] <= max_x]
  if not inside:
    return []
  lo, hi = inside[0], inside[-1]
  if margin:
    lo = max(0, lo - 1)
    hi = min(len(points) - 1, hi + 1)
  usable = points[lo:hi + 1]
  if len(usable) > count:
    # 모델 격자 자체가 앞쪽이 촘촘하다(x = 192*(i/32)^2). 보이는 구간만 남긴
    # 뒤에는 인덱스를 고르게 뽑아도 가까운 쪽이 촘촘하게 남는다. 예전처럼 다시
    # 앞쪽으로 몰아 뽑으면 앞쪽 인덱스가 겹쳐 점이 줄고 중간 구간이 빈다.
    last = len(usable) - 1
    idx = sorted({int(round(last * i / float(count - 1))) for i in range(count)})
    usable = [usable[i] for i in idx]
  return [_pt(p) for p in usable]


def _vehicle(v):
  if not isinstance(v, dict):
    return v
  return {k: _num(v[k], nd) for k, nd in VEHICLE_FIELDS if k in v}


def slim_packet(packet: dict) -> dict:
  out = {k: packet[k] for k in KEEP_KEYS if k in packet}
  lines = out.get("lanes")
  if isinstance(lines, list):
    # 슬롯 번호가 곧 차선 위치(0 바깥왼쪽 .. 3 바깥오른쪽)라 빈 슬롯도 순서대로 둔다.
    out["lanes"] = [{"p": _thin(ln.get("p")), "c": _num(ln.get("c", 0.0), 2)}
                    if isinstance(ln, dict) else ln for ln in lines]
  if isinstance(out.get("path"), list):
    out["path"] = _thin(out["path"], PATH_POINTS, PATH_MAX_X, margin=False)
  for key in ("lead", "lead2"):
    if key in out:
      out[key] = _vehicle(out[key])
  if isinstance(out.get("others"), list):
    out["others"] = [_vehicle(v) for v in out["others"]]
  return out


class BluetoothLink:
  """최신 패킷 하나만 유지하며 RFCOMM 으로 밀어 넣는다. 끊기면 스스로 다시 붙는다.

  링크가 붙고 끊길 때마다 이유를 cloudlog 로 남긴다. 주행 중에는 rlog 의
  logMessage 로, 주차 중에는 swaglog 파일로 들어간다. journald 는 재부팅하면
  지워지므로 지난 주행의 끊김은 여기서만 볼 수 있다.
  """

  def __init__(self, mac=""):
    self.mac = mac
    self._queue = queue.Queue(maxsize=1)
    self._stop = threading.Event()
    self._connected = threading.Event()
    self._last_error = ""
    # 시동이 꺼진 동안 Navdy 를 찾느라 무선을 두드리지 않도록 메인 루프가 알려 준다.
    # 처음 값은 True 로 둬서 set_active 를 부르지 않는 쪽도 예전처럼 돈다.
    self._wanted = True
    self._inactive_since = None
    # SDP 로 찾은 채널. 재접속할 때마다 sdptool 을 다시 돌리지 않는다.
    self._channels = {}
    self._thread = threading.Thread(target=self._run, daemon=True)
    self._thread.start()

  @property
  def connected(self):
    return self._connected.is_set()

  @property
  def last_error(self):
    return self._last_error

  def set_active(self, active: bool) -> None:
    """HUD 출력이 켜져 있고 시동이 걸려 있을 때만 True."""
    if active:
      self._inactive_since = None
    elif self._inactive_since is None:
      self._inactive_since = time.monotonic()
    self._wanted = active

  def _active(self) -> bool:
    # 시동을 걸 때 점화 신호가 잠깐 꺼지기도 한다. 그때마다 링크를 내렸다
    # 올리면 오히려 우리가 끊김을 만든다. 한동안 꺼져 있어야 내린다.
    if self._wanted:
      return True
    since = self._inactive_since
    return since is not None and time.monotonic() - since < INACTIVE_GRACE_S

  def send_packet(self, packet: dict) -> None:
    """블루투스로 보낼 만큼만 추려서 보낸다.

    UDP(WiFi) 경로는 원본을 그대로 쓰지만 RFCOMM 은 대역폭이 빠듯하다.
    실측 원본이 5,765B 라 10Hz 면 461kbps 로 RFCOMM 실효대역폭에 걸린다.
    HUD 가 실제로 그리는 것만 남기고 폴리라인을 솎으면 그 절반 이하가 된다.
    """
    self.send(json.dumps(slim_packet(packet), separators=(",", ":"),
                         ensure_ascii=False).encode("utf-8"))

  def send(self, payload: bytes) -> None:
    """가장 최근 것만 남긴다. 큐가 차 있으면 이전 것을 버린다."""
    try:
      self._queue.put_nowait(payload)
    except queue.Full:
      try:
        self._queue.get_nowait()
      except queue.Empty:
        pass
      try:
        self._queue.put_nowait(payload)
      except queue.Full:
        pass

  def close(self):
    self._stop.set()

  # ----- 내부 -----

  def _targets(self):
    if self.mac:
      return [self.mac]
    return _bonded_devices()

  def _dial(self):
    """붙으면 (소켓, 채널, SDP 를 새로 했는지). 못 붙으면 None."""
    for mac in self._targets():
      cached = self._channels.get(mac)
      channel = cached if cached is not None else _service_channel(mac)
      if channel is None:
        self._last_error = f"{mac}: service {SERVICE_NAME} not found"
        continue
      sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, BTPROTO_RFCOMM)
      sock.settimeout(CONNECT_TIMEOUT_S)
      try:
        sock.connect((mac, channel))
      except OSError as e:
        self._last_error = f"{mac} ch{channel}: {e.__class__.__name__} {e}"
        # 앱이 다시 뜨면서 채널이 바뀌었을 수 있다. 다음엔 SDP 부터 한다.
        self._channels.pop(mac, None)
        try:
          sock.close()
        except OSError:
          pass
        continue
      sock.settimeout(SEND_TIMEOUT_S)
      self.mac = mac
      self._channels[mac] = channel
      self._last_error = ""
      return sock, channel, cached is None
    return None

  @staticmethod
  def _frame(type_, payload: bytes) -> bytes:
    return struct.pack(">HI", type_, len(payload)) + payload

  @staticmethod
  def _writable(sock) -> bool:
    try:
      return bool(select.select([], [sock], [], WRITABLE_WAIT_S)[1])
    except (OSError, ValueError):
      return False

  def _run(self):
    backoff = RECONNECT_MIN_S
    sock = None
    last_ping = 0.0
    # 링크 하나가 살아 있는 동안의 통계. 끊길 때 한 줄로 남긴다.
    up_at = last_progress = 0.0
    frames = dropped = sent_bytes = 0
    failed_dials = 0
    dial_started = None
    logged_error = None
    was_active = None

    def drop_link(reason):
      nonlocal sock
      self._connected.clear()
      try:
        sock.close()
      except OSError:
        pass
      sock = None
      cloudlog.event("hud_bt_disconnected", mac=self.mac, reason=reason,
                     up_s=round(time.monotonic() - up_at, 1), frames=frames,
                     dropped=dropped, bytes=sent_bytes)

    while not self._stop.is_set():
      active = self._active()
      if active != was_active:
        cloudlog.event("hud_bt_active", active=active)
        was_active = active
        backoff = RECONNECT_MIN_S
      if not active:
        if sock is not None:
          drop_link("inactive")
        dial_started = None
        self._stop.wait(0.5)
        continue

      if sock is None:
        if dial_started is None:
          dial_started = time.monotonic()
        dialed = self._dial()
        if dialed is None:
          self._connected.clear()
          failed_dials += 1
          # 같은 이유로 계속 실패하는 동안은 한 번만 남긴다. 키 이름을 error 로 하면
          # cloudlog 가 ERROR 레벨로 올린다. Navdy 가 아직 안 켜진 건 정상 상황이다.
          if self._last_error != logged_error:
            cloudlog.event("hud_bt_dial_failed", reason=self._last_error, attempts=failed_dials)
            logged_error = self._last_error
          self._stop.wait(backoff)
          backoff = min(RECONNECT_MAX_S, backoff * 2)
          continue
        sock, channel, did_sdp = dialed
        cloudlog.event("hud_bt_connected", mac=self.mac, channel=channel, sdp=did_sdp,
                       dial_s=round(time.monotonic() - dial_started, 1), attempts=failed_dials + 1)
        backoff = RECONNECT_MIN_S
        failed_dials = 0
        dial_started = None
        logged_error = None
        self._connected.set()
        up_at = last_progress = last_ping = time.monotonic()
        frames = dropped = sent_bytes = 0

      try:
        payload = self._queue.get(timeout=0.5)
      except queue.Empty:
        payload = None

      now = time.monotonic()
      frames_out = []
      if payload is not None:
        frames_out.append(self._frame(TYPE_TELEMETRY, payload))
      # PING 은 텔레메트리와 별개로 주기마다 반드시 보낸다(앱이 감시에 쓸 수 있다).
      ping_due = now - last_ping >= PING_INTERVAL_S
      if ping_due:
        frames_out.append(self._frame(TYPE_PING, b"PING"))

      for frame in frames_out:
        # 무선이 잠깐 막혔다고 링크를 끊으면 SDP 부터 다시 해야 해서 수십 초를
        # 잃는다. 지금 못 쓰면 이번 프레임만 버린다. 어차피 큐에는 최신 것만 남는다.
        # 다만 한 프레임을 쓰기 시작했으면 끝까지 쓴다. 중간에 끊으면 스트림이 어긋난다.
        if not self._writable(sock):
          dropped += 1
          if time.monotonic() - last_progress >= STALL_LIMIT_S:
            drop_link(f"stalled {STALL_LIMIT_S:.0f}s")
          break
        try:
          sock.sendall(frame)
        except OSError as e:
          self._last_error = f"send: {e.__class__.__name__} {e}"
          drop_link(self._last_error)
          break
        frames += 1
        sent_bytes += len(frame)
        last_progress = time.monotonic()
      if ping_due and sock is not None:
        last_ping = now

    if sock is not None:
      drop_link("closed")
    self._connected.clear()


def available() -> bool:
  """커널에 블루투스가 없는 기기(구형 AGNOS)에서는 아예 시도하지 않는다."""
  if not os.path.exists("/sys/class/bluetooth"):
    return False
  try:
    s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, BTPROTO_RFCOMM)
    s.close()
  except OSError:
    return False
  return True
