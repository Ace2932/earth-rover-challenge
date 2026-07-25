"""Fake Frodobots SDK server — stdlib only, no bot required.

Mimics the real SDK's local HTTP surface (verified against the SDK's main.py) so
the REAL rover_client + waypoint_follower can run end-to-end over HTTP before you
ever get a bot.

It models the parts of a real bot that break navigation code, all off by default
so the deterministic quick-start still passes:

  * `/checkpoint-reached` does a real proximity check and returns the SDK's actual
    400 body — `{"detail": {"error": ..., "proximate_distance_to_checkpoint": f}}` —
    computed from the same (noisy) fix `/data` reports, exactly like the real server.
  * GPS error: white noise (`FAKE_GPS_SIGMA_M`) plus a slowly rotating bias
    (`FAKE_GPS_BIAS_M` / `FAKE_GPS_BIAS_PERIOD_S`). The bias is the realistic urban
    failure mode: it does not average out.
  * Telemetry latency (`FAKE_TELEMETRY_LATENCY_S`) — `/data` reports where the bot
    *was*, like a real 4G link.
  * Silently dropped control messages (`FAKE_CONTROL_DROP_RATE`) — the SDK sends
    commands over Agora RTM unacked, so an HTTP 200 does NOT mean the bot heard you.
  * Battery drain (`FAKE_BATTERY_DRAIN_PCT_PER_MIN`) and a settable `gps_signal`.
  * `FAKE_START_MISSION_UNAVAILABLE=1` reproduces 400 "Bot unavailable for SDK".
  * `FAKE_FAIL_RATE` still injects transient 503s to exercise client retry.

Run:  python3 fake_sdk_server.py 8777
Then: SDK_BASE_URL=http://localhost:8777 python3 waypoint_follower.py
      SDK_BASE_URL=http://localhost:8777 python3 calibrate_heading.py

Realistic urban run:
      FAKE_GPS_SIGMA_M=1.5 FAKE_GPS_BIAS_M=8 FAKE_TELEMETRY_LATENCY_S=0.5 \
      FAKE_CONTROL_DROP_RATE=0.05 python3 fake_sdk_server.py 8777
"""
import base64
import json
import math
import os
import random
import sys
import time
from dataclasses import dataclass, fields
from http.server import BaseHTTPRequestHandler, HTTPServer

M_PER_DEG = 111111.0
TINY_FRAME = base64.b64encode(b"\xff\xd8\xff\xd9").decode()  # placeholder, not a real image


@dataclass
class SimConfig:
    start_lat: float = 37.8719          # near UC Berkeley
    start_lon: float = -122.2585
    start_heading: float = 0.0
    max_speed: float = 1.5              # m/s at linear=1
    max_yaw: float = 90.0               # deg/s at angular=1

    accept_radius_m: float = 15.0       # challenge Urban tolerance
    gps_sigma_m: float = 0.0            # per-fix white noise
    gps_bias_m: float = 0.0             # slow drift magnitude
    gps_bias_period_s: float = 120.0    # how long the bias takes to rotate once
    telemetry_latency_s: float = 0.0
    control_drop_rate: float = 0.0      # fraction of commands the bot never hears
    fail_rate: float = 0.0              # transient 503s on any request
    battery_start: float = 88.0
    battery_drain_pct_per_min: float = 0.0
    gps_signal: float = 31.0
    start_mission_unavailable: bool = False
    seed: int = 0

    @classmethod
    def from_env(cls):
        c = cls()
        for f in fields(c):
            env = os.getenv("FAKE_" + f.name.upper())
            if env is None:
                continue
            cur = getattr(c, f.name)
            if isinstance(cur, bool):
                setattr(c, f.name, env not in ("", "0", "false", "False"))
            else:
                setattr(c, f.name, type(cur)(env))
        return c


ROUTE_OFFSETS = [(0.0002, 0.0001), (0.0003, 0.0004), (0.0000, 0.0005)]


class Sim:
    """Ground-truth rover state plus the error model layered on top of it.

    `true_lat`/`true_lon` are where the bot physically is. Everything the HTTP
    surface reports goes through `_reported()`, which applies latency, bias and
    noise — so tests can assert on the gap between truth and what a client sees.
    """

    def __init__(self, cfg=None, clock=time.time):
        self.cfg = cfg or SimConfig()
        self.clock = clock
        self.rng = random.Random(self.cfg.seed)
        self.t0 = self.clock()
        self.true_lat = self.cfg.start_lat
        self.true_lon = self.cfg.start_lon
        self.heading = self.cfg.start_heading
        self.cmd = (0.0, 0.0)           # command currently in force on the bot
        self.last_step_t = self.t0
        self.history = [(self.t0, self.true_lat, self.true_lon)]
        self.latest_scanned_checkpoint = 0

    # ---------------- physics ----------------

    def _integrate(self, now):
        dt = max(0.0, min(1.0, now - self.last_step_t))
        self.last_step_t = now
        if dt <= 0.0:
            return
        linear, angular = self.cmd
        self.heading = (self.heading + angular * self.cfg.max_yaw * dt) % 360.0
        speed = linear * self.cfg.max_speed
        dn = speed * dt * math.cos(math.radians(self.heading))
        de = speed * dt * math.sin(math.radians(self.heading))
        self.true_lat += dn / M_PER_DEG
        self.true_lon += de / (M_PER_DEG * math.cos(math.radians(self.true_lat)))
        self.history.append((now, self.true_lat, self.true_lon))
        if len(self.history) > 20000:
            del self.history[:10000]

    def apply_control(self, linear, angular):
        """Advance the sim, then latch the new command. Returns False if the bot
        never heard it (the RTM message vanished) — the HTTP layer still 200s."""
        now = self.clock()
        self._integrate(now)
        if self.cfg.control_drop_rate and self.rng.random() < self.cfg.control_drop_rate:
            return False
        self.cmd = (float(linear), float(angular))
        return True

    def teleport_to_checkpoint(self, index, offset_m=0.0):
        """Test helper: place the bot `offset_m` north of checkpoint `index`."""
        lat, lon = self.checkpoint_coords(index)
        self.true_lat = lat + offset_m / M_PER_DEG
        self.true_lon = lon
        now = self.clock()
        self.last_step_t = now
        self.history = [(now, self.true_lat, self.true_lon)]

    # ---------------- error model ----------------

    def _bias(self, now):
        if not self.cfg.gps_bias_m:
            return 0.0, 0.0
        ang = 2 * math.pi * (now - self.t0) / max(1e-9, self.cfg.gps_bias_period_s)
        return self.cfg.gps_bias_m * math.cos(ang), self.cfg.gps_bias_m * math.sin(ang)

    def _delayed_position(self, now):
        target = now - self.cfg.telemetry_latency_s
        lat, lon = self.history[0][1], self.history[0][2]
        for t, la, lo in self.history:
            if t > target:
                break
            lat, lon = la, lo
        return lat, lon

    def _reported(self):
        now = self.clock()
        self._integrate(now)
        lat, lon = self._delayed_position(now)
        dn, de = self._bias(now)
        if self.cfg.gps_sigma_m:
            dn += self.rng.gauss(0.0, self.cfg.gps_sigma_m)
            de += self.rng.gauss(0.0, self.cfg.gps_sigma_m)
        lat += dn / M_PER_DEG
        lon += de / (M_PER_DEG * math.cos(math.radians(lat)))
        return lat, lon

    def battery(self):
        drained = self.cfg.battery_drain_pct_per_min * (self.clock() - self.t0) / 60.0
        return max(0.0, self.cfg.battery_start - drained)

    # ---------------- telemetry ----------------

    def data(self):
        lat, lon = self._reported()
        linear, angular = self.cmd
        return {
            "battery": self.battery(),
            "signal_level": 5,
            "orientation": int(self.heading % 360 / 360 * 255),   # 0..255, units undocumented
            "lamp": 0,
            "speed": linear * self.cfg.max_speed,
            "gps_signal": self.cfg.gps_signal,
            "latitude": lat,
            "longitude": lon,
            "vibration": 0.1,
            "timestamp": self.clock(),
            "accels": [[0.0, 0.0, 9.81]],
            "gyros": [[0.0, 0.0, angular * self.cfg.max_yaw]],    # deg/s, yaw last
            "mags": [],
            "rpms": [[linear * 100] * 4],
        }

    # ---------------- mission ----------------

    def checkpoint_coords(self, index):
        dla, dlo = ROUTE_OFFSETS[index]
        return self.cfg.start_lat + dla, self.cfg.start_lon + dlo

    def checkpoints(self):
        pts = [{"id": i + 1, "sequence": i + 1,
                "latitude": str(self.cfg.start_lat + dla),
                "longitude": str(self.cfg.start_lon + dlo)}
               for i, (dla, dlo) in enumerate(ROUTE_OFFSETS)]
        return {"checkpoints_list": pts,
                "latest_scanned_checkpoint": self.latest_scanned_checkpoint}

    def distance_to_checkpoint(self, lat, lon, index=None):
        idx = self.latest_scanned_checkpoint if index is None else index
        idx = min(idx, len(ROUTE_OFFSETS) - 1)
        clat, clon = self.checkpoint_coords(idx)
        dn = (clat - lat) * M_PER_DEG
        de = (clon - lon) * M_PER_DEG * math.cos(math.radians(lat))
        return math.hypot(dn, de)

    def checkpoint_reached(self):
        """(status_code, body) — mirrors the SDK exactly, including the 400 shape."""
        if self.latest_scanned_checkpoint >= len(ROUTE_OFFSETS):
            return 400, {"detail": "Mission already complete"}
        lat, lon = self._reported()
        dist = self.distance_to_checkpoint(lat, lon)
        if dist > self.cfg.accept_radius_m:
            return 400, {"detail": {
                "error": f"Bot is not within {self.cfg.accept_radius_m:.0f} meters "
                         f"from the checkpoint",
                "proximate_distance_to_checkpoint": dist}}
        self.latest_scanned_checkpoint += 1
        return 200, {"message": "Checkpoint reached successfully",
                     "next_checkpoint_sequence": self.latest_scanned_checkpoint + 1}


def make_handler(sim):
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
            if sim.cfg.fail_rate and sim.rng.random() < sim.cfg.fail_rate:
                self._send({"detail": "transient error"}, 503)
                return True
            return False

        def do_GET(self):
            if self._maybe_fail():
                return
            p = self.path.split("?")[0]
            if p == "/data":
                self._send(sim.data())
            elif p == "/checkpoints-list":
                self._send(sim.checkpoints())
            elif p in ("/v2/front", "/v2/rear"):
                key = "front_frame" if p.endswith("front") else "rear_frame"
                self._send({key: TINY_FRAME, "timestamp": sim.clock()})
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
                # NOTE: 200 even when the message is dropped — the real SDK acks the
                # local HTTP hop, not delivery to the bot over RTM.
                sim.apply_control(float(cmd.get("linear", 0)), float(cmd.get("angular", 0)))
                self._send({"message": "Command sent successfully"})
            elif p == "/start-mission":
                if sim.cfg.start_mission_unavailable:
                    self._send({"detail": "Bot unavailable for SDK"}, 400)
                else:
                    self._send({"message": "Mission started successfully"})
            elif p == "/checkpoints-list":
                self._send(sim.checkpoints())
            elif p == "/checkpoint-reached":
                code, payload = sim.checkpoint_reached()
                self._send(payload, code)
            elif p == "/end-mission":
                self._send({"message": "Mission ended successfully"})
            else:
                self._send({"detail": "not found"}, 404)
    return H


def make_server(port, cfg=None, clock=time.time):
    """Return (HTTPServer, Sim). port=0 lets the OS pick — read srv.server_address[1]."""
    sim = Sim(cfg or SimConfig(), clock=clock)
    return HTTPServer(("127.0.0.1", port), make_handler(sim)), sim


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8777
    cfg = SimConfig.from_env()
    srv, sim = make_server(port, cfg)
    print(f"fake SDK on http://localhost:{port}  "
          f"(accept_radius={cfg.accept_radius_m}m gps_sigma={cfg.gps_sigma_m}m "
          f"bias={cfg.gps_bias_m}m latency={cfg.telemetry_latency_s}s "
          f"drop={cfg.control_drop_rate})")
    srv.serve_forever()
