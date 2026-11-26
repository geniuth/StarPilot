"""Auxiliary processes must not add msgq readers to services that are already near the reader limit.

msgq allows NUM_READERS (15) subscribers per service. One more and it evicts *every* subscriber of that
service; they reconnect, evict again, and drop messages. On 2026-11-26 the NDA receiver, Navdy HUD and
Wayon telemetry each subscribed to carState on top of StarPilot's ~14 readers, which made calibrationd,
radard, torqued, lagd, ... publish invalid and selfdrived raise commIssue for the whole drive.
"""
import ast
import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
NUM_READERS = int(re.search(r"#define NUM_READERS (\d+)", (ROOT / "msgq_repo/msgq/msgq.h").read_text()).group(1))

# Services with many always-on readers. Only the budget listed per process may be used.
BUSY = {"carState", "deviceState"}
AUX_MODULES = {
  "selfdrive/road_speed_limiter.py": set(),
  "system/wayon_vehicle_telemetry.py": set(),
  "system/wayon_live_stream.py": set(),
  "system/wayon_remote.py": set(),
  "selfdrive/eon_cluster/navdy_hud.py": {"carState"},  # the only extra carState reader
}


def _string_constants(node):
  return {n.value for n in ast.walk(node) if isinstance(n, ast.Constant) and isinstance(n.value, str)}


def _subscribed(path: Path) -> set[str]:
  tree = ast.parse(path.read_text(encoding="utf-8"))
  names: set[str] = set()
  # Module-level lists passed by name (e.g. SERVICES = [...]) count as subscriptions too.
  assigned = {t.id: n.value for n in ast.walk(tree) if isinstance(n, ast.Assign)
              for t in n.targets if isinstance(t, ast.Name)}
  for call in ast.walk(tree):
    if not isinstance(call, ast.Call):
      continue
    fn = call.func.attr if isinstance(call.func, ast.Attribute) else getattr(call.func, "id", "")
    if fn not in ("SubMaster", "sub_sock") or not call.args:
      continue
    arg = call.args[0]
    if isinstance(arg, ast.Name) and arg.id in assigned:
      arg = assigned[arg.id]
    names |= _string_constants(arg)
  return names


def test_reader_limit_is_what_this_test_assumes():
  assert NUM_READERS == 15


@pytest.mark.parametrize("module", sorted(AUX_MODULES))
def test_aux_process_stays_off_busy_services(module):
  busy_used = _subscribed(ROOT / module) & BUSY
  assert busy_used <= AUX_MODULES[module], f"{module} subscribes to {sorted(busy_used - AUX_MODULES[module])}"
