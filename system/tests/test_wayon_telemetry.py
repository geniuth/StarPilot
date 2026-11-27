"""Pure helpers of the MyID4 (Wayon) vehicle telemetry uploader and its process gating."""
import pytest

from openpilot.system import wayon_vehicle_telemetry as wt


@pytest.mark.parametrize("value, expected", [
  (52000, 52000.0),
  ("41000.5", 41000.5),
  (0, None),
  (-5, None),
  (102300, None),          # MEB "invalid" sentinel
  (wt.MAX_VALID_ENERGY_WH + 1, None),
  ("x", None),
  (None, None),
])
def test_valid_battery_wh(value, expected):
  assert wt.valid_battery_wh(value) == expected


def test_haversine_one_degree_latitude():
  assert wt.haversine_m((37.0, 127.0), (38.0, 127.0)) == pytest.approx(111195, rel=1e-3)


def test_charge_eta_ac_and_dc():
  half = wt.USABLE_BATTERY_WH * 0.5
  # AC 7 kW from 50% to 80%: 30% of 64 kWh = 19.2 kWh -> ~164.6 min
  assert wt.estimate_charge_minutes(half, 7000, "ac", 80) == pytest.approx(19200 / 7000 * 60, rel=1e-6)
  assert wt.estimate_charge_minutes(half, 100, "ac", 80) is None   # not charging
  assert wt.estimate_charge_minutes(half, 7000, "ac", 50) == 0.0
  dc = wt.estimate_charge_minutes(half, 120000, "dc", 80)
  assert dc is not None and 5 < dc < 30


def test_wayon_processes_are_gated_on_cloud_config(tmp_path, monkeypatch):
  pc = pytest.importorskip("openpilot.system.manager.process_config")
  monkeypatch.setattr(pc, "WAYON_CLOUD_CONFIG", str(tmp_path / "config.json"))
  monkeypatch.setattr(pc, "WAYON_LIVE_ACTIVE_PATH", str(tmp_path / "live.active"))
  assert not pc.wayon_remote_ready(False, None, None, None)
  assert not pc.wayon_live_streaming(False, None, None, None)
  (tmp_path / "config.json").write_text("{}")
  (tmp_path / "live.active").write_text("1")
  assert pc.wayon_remote_ready(False, None, None, None)
  assert pc.wayon_live_streaming(False, None, None, None)


def test_tailscale_only_mode_swaps_tunnel_for_local_cloud(tmp_path, monkeypatch):
  pc = pytest.importorskip("openpilot.system.manager.process_config")
  config = tmp_path / "config.json"
  monkeypatch.setattr(pc, "WAYON_CLOUD_CONFIG", str(config))
  procs = pc.managed_processes
  args = (False, None, None, None)

  config.write_text('{"endpoint": "https://wayon-cloud.example.workers.dev", "token": "t"}')
  assert procs["wayon_remote"].should_run(*args) and not procs["wayon_local"].should_run(*args)

  config.write_text('{"endpoint": "http://127.0.0.1:8770", "token": "t"}')
  import os
  os.utime(config, (1, 2))  # new mtime so the cached endpoint is re-read
  assert procs["wayon_local"].should_run(*args) and not procs["wayon_remote"].should_run(*args)
