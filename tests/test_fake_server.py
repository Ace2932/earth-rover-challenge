"""Fidelity tests for the fake SDK server (issue #14).

The offline harness only helps if it reproduces the responses a real bot gives:
a 400 with the server's own distance when you are not close enough, GPS that is
noisy and biased, control messages that can silently vanish, and a battery that
runs down. These tests pin that behavior.
"""
import json
import math
import threading
import urllib.request
import urllib.error

import pytest

from fake_sdk_server import Sim, SimConfig, make_server

M_PER_DEG = 111111.0


def sim_at_start(**kw):
    """Sim with a controllable clock, parked at the mission start point."""
    t = {"now": 1000.0}
    cfg = SimConfig(**kw)
    s = Sim(cfg, clock=lambda: t["now"])
    return s, t


def offset_m(sim, lat, lon):
    """Metres between a reported fix and the sim's true position."""
    dn = (lat - sim.true_lat) * M_PER_DEG
    de = (lon - sim.true_lon) * M_PER_DEG * math.cos(math.radians(sim.true_lat))
    return math.hypot(dn, de)


# ---------------- checkpoint semantics ----------------

def test_checkpoint_reached_400_carries_server_distance():
    s, _ = sim_at_start(accept_radius_m=15.0)
    code, body = s.checkpoint_reached()
    assert code == 400
    detail = body["detail"]
    assert "not within" in detail["error"]
    assert detail["proximate_distance_to_checkpoint"] > 15.0


def test_checkpoint_reached_200_within_accept_radius():
    s, _ = sim_at_start(accept_radius_m=15.0)
    s.teleport_to_checkpoint(0, offset_m=10.0)
    code, body = s.checkpoint_reached()
    assert code == 200
    assert body["next_checkpoint_sequence"] == 2


def test_accept_radius_is_configurable():
    s, _ = sim_at_start(accept_radius_m=5.0)
    s.teleport_to_checkpoint(0, offset_m=10.0)      # inside 15 m, outside 5 m
    assert s.checkpoint_reached()[0] == 400


def test_latest_scanned_checkpoint_advances_only_on_success():
    s, _ = sim_at_start(accept_radius_m=15.0)
    assert s.checkpoints()["latest_scanned_checkpoint"] == 0
    s.checkpoint_reached()                           # too far -> no advance
    assert s.checkpoints()["latest_scanned_checkpoint"] == 0
    s.teleport_to_checkpoint(0, offset_m=1.0)
    s.checkpoint_reached()
    assert s.checkpoints()["latest_scanned_checkpoint"] == 1


def test_checkpoint_reached_uses_the_next_checkpoint_after_success():
    s, _ = sim_at_start(accept_radius_m=15.0)
    s.teleport_to_checkpoint(0, offset_m=1.0)
    s.checkpoint_reached()
    code, body = s.checkpoint_reached()              # still parked on cp 1
    assert code == 400                               # cp 2 is far away
    assert body["detail"]["proximate_distance_to_checkpoint"] > 15.0


# ---------------- GPS error model ----------------

def test_gps_noise_scatters_reports_around_the_true_position():
    s, _ = sim_at_start(gps_sigma_m=2.0, seed=1)
    fixes = [s.data() for _ in range(400)]
    offs = [offset_m(s, f["latitude"], f["longitude"]) for f in fixes]
    assert max(offs) > 2.0                                    # actually scatters
    mean_n = sum((f["latitude"] - s.true_lat) * M_PER_DEG for f in fixes) / len(fixes)
    assert abs(mean_n) < 0.6                                  # zero-mean


def test_gps_bias_is_a_slow_drift_not_per_sample_noise():
    s, t = sim_at_start(gps_bias_m=8.0, gps_bias_period_s=600.0, seed=2)
    a = s.data()
    t["now"] += 0.2
    b = s.data()
    assert offset_m(s, a["latitude"], a["longitude"]) == pytest.approx(8.0, abs=0.5)
    # 0.2 s apart the bias has barely moved -> consecutive fixes nearly identical
    assert offset_m(s, b["latitude"], b["longitude"] ) == pytest.approx(8.0, abs=0.5)
    step = math.hypot((b["latitude"] - a["latitude"]) * M_PER_DEG,
                      (b["longitude"] - a["longitude"]) * M_PER_DEG)
    assert step < 0.2


def test_gps_bias_moves_over_a_full_period():
    s, t = sim_at_start(gps_bias_m=8.0, gps_bias_period_s=600.0, seed=3)
    a = s.data()
    t["now"] += 300.0                                          # half a period
    b = s.data()
    step = math.hypot((b["latitude"] - a["latitude"]) * M_PER_DEG,
                      (b["longitude"] - a["longitude"]) * M_PER_DEG)
    assert step > 8.0


def test_server_distance_is_computed_from_the_reported_fix():
    """The real server sees the same biased GPS the client does, so its distance
    tracks the reported position, not the true one."""
    s, _ = sim_at_start(accept_radius_m=15.0, gps_bias_m=20.0, gps_bias_period_s=1e9)
    s.teleport_to_checkpoint(0, offset_m=0.0)                  # physically ON the checkpoint
    fix = s.data()
    reported = s.distance_to_checkpoint(fix["latitude"], fix["longitude"])
    code, body = s.checkpoint_reached()
    # sitting on the checkpoint, but a 20 m bias makes both sides believe otherwise
    assert code == 400
    assert reported == pytest.approx(20.0, abs=1.0)
    assert body["detail"]["proximate_distance_to_checkpoint"] == pytest.approx(reported, abs=0.5)


# ---------------- telemetry latency ----------------

def test_telemetry_lags_the_true_position_by_the_configured_latency():
    s, t = sim_at_start(telemetry_latency_s=1.0)
    for _ in range(10):                                        # drive north 1 s
        s.apply_control(1.0, 0.0)
        t["now"] += 0.1
    lagged = s.data()
    assert offset_m(s, lagged["latitude"], lagged["longitude"]) > 1.0
    s.apply_control(0.0, 0.0)                                  # park it (commands latch)
    t["now"] += 1.0                                            # let the lag catch up
    caught_up = s.data()
    assert offset_m(s, caught_up["latitude"], caught_up["longitude"]) < 0.3


# ---------------- unacked control ----------------

def test_dropped_control_messages_do_not_move_the_rover():
    s, t = sim_at_start(control_drop_rate=1.0, seed=4)
    for _ in range(10):
        assert s.apply_control(1.0, 0.0) is False              # reports the drop
        t["now"] += 0.1
    assert s.true_lat == pytest.approx(SimConfig().start_lat, abs=1e-9)


def test_control_moves_the_rover_when_not_dropped():
    s, t = sim_at_start()
    for _ in range(10):
        assert s.apply_control(1.0, 0.0) is True
        t["now"] += 0.1
    assert (s.true_lat - SimConfig().start_lat) * M_PER_DEG > 0.5


# ---------------- battery / signal ----------------

def test_battery_drains_over_time():
    s, t = sim_at_start(battery_start=100.0, battery_drain_pct_per_min=6.0)
    assert s.data()["battery"] == pytest.approx(100.0, abs=0.1)
    t["now"] += 600.0                                          # 10 minutes
    assert s.data()["battery"] == pytest.approx(40.0, abs=0.5)


def test_gps_signal_is_configurable():
    s, _ = sim_at_start(gps_signal=3.0)
    assert s.data()["gps_signal"] == pytest.approx(3.0)


# ---------------- HTTP surface ----------------

@pytest.fixture
def server():
    srv, sim = make_server(0, SimConfig(accept_radius_m=15.0))
    th = threading.Thread(target=srv.serve_forever, daemon=True)
    th.start()
    yield f"http://127.0.0.1:{srv.server_address[1]}", sim
    srv.shutdown()
    srv.server_close()


def _post(url, payload=None):
    req = urllib.request.Request(url, data=json.dumps(payload or {}).encode(),
                                 headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def _get(url):
    with urllib.request.urlopen(url, timeout=5) as r:
        return r.status, json.loads(r.read())


def test_http_checkpoint_reached_returns_400_with_sdk_detail_shape(server):
    base, _ = server
    code, body = _post(base + "/checkpoint-reached")
    assert code == 400
    assert "proximate_distance_to_checkpoint" in body["detail"]


def test_http_checkpoint_reached_returns_200_when_close(server):
    base, sim = server
    sim.teleport_to_checkpoint(0, offset_m=2.0)
    code, body = _post(base + "/checkpoint-reached")
    assert code == 200
    assert body["next_checkpoint_sequence"] == 2


def test_http_start_mission_can_report_bot_unavailable():
    srv, sim = make_server(0, SimConfig(start_mission_unavailable=True))
    th = threading.Thread(target=srv.serve_forever, daemon=True)
    th.start()
    try:
        code, body = _post(f"http://127.0.0.1:{srv.server_address[1]}/start-mission")
        assert code == 400
        assert body["detail"] == "Bot unavailable for SDK"
    finally:
        srv.shutdown()
        srv.server_close()


def test_http_checkpoints_list_is_ordered_and_reports_progress(server):
    base, sim = server
    code, body = _get(base + "/checkpoints-list")
    assert code == 200
    seqs = [c["sequence"] for c in body["checkpoints_list"]]
    assert seqs == sorted(seqs)
    assert body["latest_scanned_checkpoint"] == 0


# ---------------- the bot does not turn exactly as commanded (issue #56) ----------------

def test_yaw_is_exact_by_default():
    """The deterministic quick-start must stay deterministic."""
    s, t = sim_at_start()
    s.apply_control(0.0, 1.0)
    t["now"] += 1.0
    s.apply_control(0.0, 1.0)
    assert s.heading == pytest.approx(SimConfig().max_yaw, abs=0.01)


def test_a_yaw_scale_error_makes_dead_reckoning_wrong():
    """An uncalibrated YAW_RATE_DPS looks exactly like this: the rover turns more (or
    less) than commanded, and open-loop dead reckoning drifts without bound."""
    s, t = sim_at_start(yaw_scale=1.2)
    s.apply_control(0.0, 1.0)
    t["now"] += 1.0
    s.apply_control(0.0, 1.0)
    assert s.heading == pytest.approx(SimConfig().max_yaw * 1.2, abs=0.01)


def test_yaw_noise_accumulates_over_time():
    """Slip and surface: individually small, unbounded once integrated."""
    s, t = sim_at_start(yaw_noise_dps=10.0, seed=5)
    for _ in range(50):
        s.apply_control(0.5, 0.0)          # driving straight, no turn commanded
        t["now"] += 0.2
    assert abs(wrap(s.heading)) > 1.0, "no drift at all from yaw noise"


def test_yaw_noise_is_deterministic_for_a_seed():
    def run(seed):
        s, t = sim_at_start(yaw_noise_dps=10.0, seed=seed)
        for _ in range(20):
            s.apply_control(0.5, 0.0)
            t["now"] += 0.2
        return s.heading

    assert run(11) == run(11)
    assert run(11) != run(12)


def wrap(deg):
    return (deg + 180.0) % 360.0 - 180.0
