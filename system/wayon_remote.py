#!/usr/bin/env python3
"""Wayon 원격 터널 supervisor 래퍼.

/data/wayon-remote/wayon_remote_supervisor.sh (cloudflared 터널, 오프로드 전용)를
openpilot manager 관리 프로세스로 상시 유지한다. supervisor 스크립트 자체가
IsOnroad 폴링으로 주행 시 터널을 내리므로 여기선 생존만 책임진다.
flock으로 중복 실행(수동 기동 잔재 등)을 방지한다.
"""
import fcntl
import os
import subprocess
import sys
import time

ROOT = "/data/wayon-remote"
SUPERVISOR = os.path.join(ROOT, "wayon_remote_supervisor.sh")
LOCK_PATH = os.path.join(ROOT, "supervisor.lock")


def main() -> None:
  while True:
    if not os.path.isfile(SUPERVISOR):
      time.sleep(30)
      continue

    os.makedirs(ROOT, exist_ok=True)
    lock = open(LOCK_PATH, "w")
    try:
      fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
      # 다른 supervisor 인스턴스가 이미 떠 있음 (수동 실행 등)
      lock.close()
      time.sleep(60)
      continue

    try:
      proc = subprocess.Popen(["sh", SUPERVISOR])
      ret = proc.wait()
      print(f"wayon_remote: supervisor exited {ret}", flush=True)
    finally:
      fcntl.flock(lock, fcntl.LOCK_UN)
      lock.close()
    time.sleep(10)


if __name__ == "__main__":
  sys.exit(main())
