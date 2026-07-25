"""Thin, resilient HTTP wrapper around the Frodobots Earth Rovers SDK (v5.x).

The SDK itself runs a local FastAPI server (`hypercorn main:app`) that holds your
SDK_API_TOKEN + BOT_SLUG and talks to the bot. This client just calls that local
server, so no token lives here. Default base URL: http://localhost:8000.

Resilience matters: the bot is on 4G, so transient request failures are normal.
Every call retries with backoff; `get_data`/`control` raise only after all retries
fail (the control loop turns that into a safe stop). `checkpoint_reached` is
special — a 400 "not within X m" is a normal answer, not an error, so it returns
(ok, detail) instead of raising.

Endpoints (verified against the SDK's main.py):
  GET  /data             -> latitude, longitude, orientation, battery, imu, ...
  GET  /v2/front         -> {front_frame: base64, timestamp}
  POST /control          -> {command:{linear,angular,lamp}}, each -1..1 (lamp 0/1)
  POST /start-mission
  GET  /checkpoints-list -> {checkpoints_list:[{sequence,latitude,longitude}], ...}
  POST /checkpoint-reached -> 200 {..next_checkpoint_sequence} | 400 {detail:{error,proximate_distance_to_checkpoint}}
  POST /end-mission
"""
import base64
import time

import requests


def _clamp(v, lo=-1.0, hi=1.0):
    return max(lo, min(hi, float(v)))


def server_distance(detail):
    """Metres to the current checkpoint per the SERVER, from a `checkpoint_reached`
    payload, or None if it does not carry one.

    The SDK answers a refusal with FastAPI's envelope around its own error object:
        {"detail": {"error": "Bot is not within 15 meters from the checkpoint",
                    "proximate_distance_to_checkpoint": 12.5}}
    This is the authoritative answer to "would this count", so it is worth far more
    than our own haversine — and it was being thrown away.
    """
    if not isinstance(detail, dict):
        return None
    inner = detail.get("detail")
    if not isinstance(inner, dict):
        return None
    try:
        return float(inner["proximate_distance_to_checkpoint"])
    except (KeyError, TypeError, ValueError):
        return None


class RoverClient:
    def __init__(self, base_url="http://localhost:8000", timeout=5.0,
                 retries=3, backoff=0.3):
        self.base = base_url.rstrip("/")
        self.timeout = timeout
        self.retries = max(1, retries)
        self.backoff = backoff
        self.s = requests.Session()

    def _req(self, method, path, **kw):
        """Retry transient failures with exponential backoff; raise after the last."""
        kw.setdefault("timeout", self.timeout)
        last = None
        for i in range(self.retries):
            try:
                r = self.s.request(method, f"{self.base}{path}", **kw)
                r.raise_for_status()
                return r
            except requests.RequestException as e:
                last = e
                if i < self.retries - 1:
                    time.sleep(self.backoff * (2 ** i))
        raise last

    def get_data(self):
        return self._req("GET", "/data").json()

    def get_front_frame(self):
        """Return (jpeg_bytes | None, timestamp)."""
        j = self._req("GET", "/v2/front").json()
        b64 = j.get("front_frame")
        return (base64.b64decode(b64) if b64 else None), j.get("timestamp")

    def control(self, linear, angular, lamp=0):
        payload = {"command": {"linear": _clamp(linear),
                               "angular": _clamp(angular),
                               "lamp": int(lamp)}}
        return self._req("POST", "/control", json=payload).json()

    # --- Missions API ---
    def start_mission(self):
        try:
            return self._req("POST", "/start-mission").json()
        except requests.RequestException:
            return {}      # mission may already be running; not fatal

    def checkpoints(self):
        return self._req("GET", "/checkpoints-list").json()

    def checkpoint_reached(self):
        """Return (ok, detail). A 400 'not within X m' is a normal answer, not an error."""
        try:
            r = self.s.post(f"{self.base}/checkpoint-reached", json={}, timeout=self.timeout)
        except requests.RequestException as e:
            return False, {"error": str(e)}
        try:
            detail = r.json()
        except ValueError:
            detail = {}
        return (r.status_code == 200), detail

    def end_mission(self):
        try:
            return self._req("POST", "/end-mission").json()
        except requests.RequestException:
            return {}
