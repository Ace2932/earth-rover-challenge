"""Guarding the destructive endpoint (issue #15).

The SDK documents /end-mission as:

    "This endpoint should only be used in case of emergency. If you run this
     endpoint you will lose all your progress."

It was exposed as `RoverClient.end_mission()` — a plain, inviting name sitting in
the public surface next to routine calls like `start_mission` and `checkpoints`,
one autocomplete away from destroying a mission in progress.
"""
import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import pytest

from rover_client import RoverClient


def stub_server():
    calls = []

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_POST(self):
            calls.append(self.path)
            body = json.dumps({"message": "Mission ended successfully"}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    srv = HTTPServer(("127.0.0.1", 0), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return f"http://127.0.0.1:{srv.server_address[1]}", calls, srv.shutdown


def test_the_destructive_call_refuses_without_explicit_confirmation():
    base, calls, shutdown = stub_server()
    try:
        client = RoverClient(base_url=base)
        with pytest.raises(RuntimeError):
            client.emergency_abort_lose_all_progress()
        assert calls == [], "sent the request anyway"
    finally:
        shutdown()


def test_it_goes_through_when_confirmed():
    base, calls, shutdown = stub_server()
    try:
        RoverClient(base_url=base).emergency_abort_lose_all_progress(confirm=True)
        assert calls == ["/end-mission"]
    finally:
        shutdown()


def test_the_old_inviting_name_is_gone():
    """`end_mission()` sat next to start_mission/checkpoints and read as routine."""
    assert not hasattr(RoverClient, "end_mission")
