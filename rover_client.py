"""Thin HTTP wrapper around the Frodobots Earth Rovers SDK (v5.x).

The SDK itself runs a local FastAPI server (`hypercorn main:app`) that holds your
SDK_API_TOKEN + BOT_SLUG and talks to the bot. This client just calls that local
server, so no token lives here. Default base URL: http://localhost:8000.

Endpoints used (see SDK README):
  GET  /data             -> battery, gps (latitude/longitude), orientation, imu, ...
  GET  /v2/front         -> {front_frame: base64, timestamp}
  POST /control          -> {command:{linear,angular,lamp}}, each -1..1 (lamp 0/1)
  POST /start-mission
  GET  /checkpoints-list -> {checkpoints_list:[{sequence,latitude,longitude}], latest_scanned_checkpoint}
  POST /checkpoint-reached
  POST /end-mission
"""
import base64
import requests


def _clamp(v, lo=-1.0, hi=1.0):
    return max(lo, min(hi, float(v)))


class RoverClient:
    def __init__(self, base_url="http://localhost:8000", timeout=5.0):
        self.base = base_url.rstrip("/")
        self.timeout = timeout

    def get_data(self):
        r = requests.get(f"{self.base}/data", timeout=self.timeout)
        r.raise_for_status()
        return r.json()

    def get_front_frame(self):
        """Return (jpeg_bytes | None, timestamp)."""
        r = requests.get(f"{self.base}/v2/front", timeout=self.timeout)
        r.raise_for_status()
        j = r.json()
        b64 = j.get("front_frame")
        return (base64.b64decode(b64) if b64 else None), j.get("timestamp")

    def control(self, linear, angular, lamp=0):
        payload = {"command": {"linear": _clamp(linear),
                               "angular": _clamp(angular),
                               "lamp": int(lamp)}}
        r = requests.post(f"{self.base}/control", json=payload, timeout=self.timeout)
        r.raise_for_status()
        return r.json()

    # --- Missions API ---
    def start_mission(self):
        r = requests.post(f"{self.base}/start-mission", timeout=self.timeout)
        return r.json()

    def checkpoints(self):
        r = requests.get(f"{self.base}/checkpoints-list", timeout=self.timeout)
        r.raise_for_status()
        return r.json()

    def checkpoint_reached(self):
        r = requests.post(f"{self.base}/checkpoint-reached", json={}, timeout=self.timeout)
        return r.status_code, r.json()

    def end_mission(self):
        r = requests.post(f"{self.base}/end-mission", timeout=self.timeout)
        return r.json()
