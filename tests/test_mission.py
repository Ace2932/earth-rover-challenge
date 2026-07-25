"""Mission start / ordering / resume semantics (issues #5 and #6).

Two ways a live run silently goes wrong before the rover even moves:

  #5  `start_mission` swallowed every error, so 400 "Bot unavailable for SDK" —
      the most likely failure on call day — produced an empty checkpoint list,
      an empty control loop, and the message "COMPLETE — 0/0 waypoints".
  #6  The checkpoint list was consumed in payload order with `latest_scanned_checkpoint`
      ignored, so a crash-restart drove all the way back to checkpoint 1.
"""
import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import pytest

from rover_client import RoverClient
from waypoint_follower import Config, LiveIO, MissionUnavailable, run


# ---------------- a stub SDK, so these tests do not depend on the fake server ----------------

def stub_server(routes):
    """routes: {(method, path): (status, body)} -> (base_url, shutdown)"""
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _handle(self, method):
            status, body = routes.get((method, self.path), (404, {"detail": "not found"}))
            raw = json.dumps(body).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def do_GET(self):
            self._handle("GET")

        def do_POST(self):
            length = int(self.headers.get("Content-Length", 0) or 0)
            if length:
                self.rfile.read(length)
            self._handle("POST")

    srv = HTTPServer(("127.0.0.1", 0), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return f"http://127.0.0.1:{srv.server_address[1]}", srv.shutdown


class StubClient:
    """Just the two calls LiveIO.waypoints() makes."""

    def __init__(self, checkpoints, start_ok=True, start_body=None):
        self._checkpoints = checkpoints
        self._start = (start_ok, start_body or {})
        self.started = False

    def start_mission(self):
        self.started = True
        return self._start

    def checkpoints(self):
        return self._checkpoints


def live_io(client):
    io = LiveIO(Config())
    io.c = client
    return io


CHECKPOINTS = {
    "checkpoints_list": [
        {"sequence": 3, "latitude": "37.8722", "longitude": "-122.2580"},
        {"sequence": 1, "latitude": "37.8720", "longitude": "-122.2584"},
        {"sequence": 2, "latitude": "37.8721", "longitude": "-122.2582"},
    ],
    "latest_scanned_checkpoint": 0,
}


# ---------------- #5: fail loud ----------------

def test_start_mission_reports_bot_unavailable_instead_of_swallowing_it():
    base, shutdown = stub_server({("POST", "/start-mission"):
                                  (400, {"detail": "Bot unavailable for SDK"})})
    try:
        ok, body = RoverClient(base_url=base).start_mission()
        assert ok is False
        assert body["detail"] == "Bot unavailable for SDK"
    finally:
        shutdown()


def test_start_mission_reports_success():
    base, shutdown = stub_server({("POST", "/start-mission"):
                                  (200, {"message": "Mission started successfully"})})
    try:
        ok, _ = RoverClient(base_url=base).start_mission()
        assert ok is True
    finally:
        shutdown()


def test_waypoints_raise_when_the_bot_is_unavailable():
    io = live_io(StubClient(CHECKPOINTS, start_ok=False,
                            start_body={"detail": "Bot unavailable for SDK"}))
    with pytest.raises(MissionUnavailable):
        io.waypoints(None)


def test_waypoints_raise_on_an_empty_checkpoint_list():
    io = live_io(StubClient({"checkpoints_list": [], "latest_scanned_checkpoint": 0}))
    with pytest.raises(MissionUnavailable):
        io.waypoints(None)


def test_run_does_not_report_complete_with_zero_waypoints():
    class EmptyIO:
        hsrc = "stub"

        def waypoints(self, route_file):
            return [], 0

        def control(self, linear, angular):
            pass

    assert run(EmptyIO(), Config()) is False


# ---------------- #6: ordering and resume ----------------

def test_waypoints_are_ordered_by_sequence_not_payload_order():
    io = live_io(StubClient(CHECKPOINTS))
    wps, _ = io.waypoints(None)
    assert [round(lat, 4) for lat, _ in wps] == [37.8720, 37.8721, 37.8722]


def test_resume_starts_at_the_latest_scanned_checkpoint():
    payload = dict(CHECKPOINTS, latest_scanned_checkpoint=2)
    io = live_io(StubClient(payload))
    wps, start = io.waypoints(None)
    assert len(wps) == 3
    assert start == 2                       # checkpoints 1 and 2 already confirmed


def test_resume_index_is_clamped_to_the_route_length():
    payload = dict(CHECKPOINTS, latest_scanned_checkpoint=99)
    io = live_io(StubClient(payload))
    _, start = io.waypoints(None)
    assert start == 3


def test_missing_progress_counter_starts_from_the_beginning():
    payload = {"checkpoints_list": CHECKPOINTS["checkpoints_list"]}
    io = live_io(StubClient(payload))
    _, start = io.waypoints(None)
    assert start == 0


def test_explicit_route_file_never_starts_a_mission(tmp_path):
    route = tmp_path / "route.json"
    route.write_text(json.dumps([{"latitude": 1.0, "longitude": 2.0}]))
    client = StubClient(CHECKPOINTS)
    io = live_io(client)
    wps, start = io.waypoints(str(route))
    assert wps == [(1.0, 2.0)]
    assert start == 0
    assert client.started is False


def test_run_resumes_from_the_start_index_and_reports_absolute_progress(capsys):
    class ResumeIO:
        hsrc = "stub"

        def __init__(self):
            self.stopped = False

        def waypoints(self, route_file):
            return [(0.0, 0.0), (0.0, 0.001), (0.0, 0.002)], 2

        def get_pose(self):
            return 0.0, 0.002, 0.0          # sitting on the last checkpoint

        def control(self, linear, angular):
            self.stopped = linear == 0 and angular == 0

        def front_frame(self):
            return None

        def reached(self):
            return True, {}

    assert run(ResumeIO(), Config()) is True
    out = capsys.readouterr().out
    assert "reached wp 3/3" in out           # not "wp 1/3"
