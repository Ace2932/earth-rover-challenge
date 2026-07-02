"""Fake Frodobots SDK server — stdlib only, no bot required.

Mimics the real SDK's local HTTP surface (verified against the SDK's main.py) so
the REAL rover_client + waypoint_follower can run end-to-end over HTTP before you
ever get a bot. Simulates a moving rover: /control integrates a kinematic model,
/data reports GPS + a 0..255 `orientation` (like the real bot, units unknown ->
exercises heading calibration), Missions API returns a short route.

Run:  python3 fake_sdk_server.py 8777
Then: SDK_BASE_URL=http://localhost:8777 python3 waypoint_follower.py
      SDK_BASE_URL=http://localhost:8777 python3 calibrate_heading.py
"""
import base64
import json
import math
import os
import random
import sys
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

FAIL_RATE = float(os.getenv("FAKE_FAIL_RATE", "0"))   # prob of a transient 503 per request

START = (37.8719, -122.2585, 0.0)   # near UC Berkeley
MAX_SPEED = 1.5                     # m/s at linear=1
MAX_YAW = 90.0                      # deg/s at angular=1
state = {"lat": START[0], "lon": START[1], "heading": START[2], "last": None}
TINY_FRAME = base64.b64encode(b"\xff\xd8\xff\xd9").decode()  # placeholder, not a real image


def route():
    b = (START[0], START[1])
    pts = [(0.0002, 0.0001), (0.0003, 0.0004), (0.0000, 0.0005)]
    return [{"sequence": i + 1, "latitude": str(b[0] + dla), "longitude": str(b[1] + dlo)}
            for i, (dla, dlo) in enumerate(pts)]


def step(linear, angular):
    now = time.time()
    last = state["last"]
    state["last"] = now
    dt = 0.2 if last is None else min(1.0, now - last)
    state["heading"] = (state["heading"] + angular * MAX_YAW * dt) % 360.0
    speed = linear * MAX_SPEED
    dn = speed * dt * math.cos(math.radians(state["heading"]))
    de = speed * dt * math.sin(math.radians(state["heading"]))
    state["lat"] += dn / 111111.0
    state["lon"] += de / (111111.0 * math.cos(math.radians(state["lat"])))


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _maybe_fail(self):
        if FAIL_RATE and random.random() < FAIL_RATE:
            self._send({"detail": "transient error"}, 503)
            return True
        return False

    def do_GET(self):
        if self._maybe_fail():
            return
        p = self.path.split("?")[0]
        if p == "/data":
            o = int(state["heading"] % 360 / 360 * 255)   # 0..255, like the real bot
            self._send({"battery": 88, "signal_level": 5, "orientation": o, "lamp": 0,
                        "speed": 0, "gps_signal": 31.0, "latitude": state["lat"],
                        "longitude": state["lon"], "vibration": 0.1, "timestamp": time.time(),
                        "accels": [], "gyros": [], "mags": [], "rpms": []})
        elif p == "/checkpoints-list":
            self._send({"checkpoints_list": route(), "latest_scanned_checkpoint": 0})
        elif p in ("/v2/front", "/v2/rear"):
            self._send({"front_frame": TINY_FRAME, "timestamp": time.time()})
        else:
            self._send({"detail": "not found"}, 404)

    def do_POST(self):
        if self._maybe_fail():
            return
        p = self.path.split("?")[0]
        ln = int(self.headers.get("Content-Length", 0) or 0)
        raw = self.rfile.read(ln) if ln else b"{}"
        try:
            body = json.loads(raw or b"{}")
        except json.JSONDecodeError:
            body = {}
        if p == "/control":
            cmd = body.get("command", {})
            step(float(cmd.get("linear", 0)), float(cmd.get("angular", 0)))
            self._send({"message": "Command sent successfully"})
        elif p == "/start-mission":
            self._send({"message": "Mission started successfully"})
        elif p == "/checkpoints-list":
            self._send({"checkpoints_list": route(), "latest_scanned_checkpoint": 0})
        elif p == "/checkpoint-reached":
            self._send({"message": "Checkpoint reached successfully", "next_checkpoint_sequence": 0})
        elif p == "/end-mission":
            self._send({"message": "Mission ended successfully"})
        else:
            self._send({"detail": "not found"}, 404)


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8777
    print(f"fake SDK on http://localhost:{port}  "
          f"(/data /control /v2/front /start-mission /checkpoints-list /checkpoint-reached)")
    HTTPServer(("127.0.0.1", port), H).serve_forever()
