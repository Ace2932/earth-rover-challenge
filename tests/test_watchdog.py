"""Out-of-process runaway protection (#2).

The follower's `try/finally` covers Ctrl-C and exceptions. It does not cover
`kill -9`, an OOM, a wedged interpreter, or the laptop going to sleep — and the
rover latches its last command, because `/control` is a fire-and-forget RTM
message with no bot-side watchdog documented anywhere in the SDK.

So a second process holds the heartbeat the Commander refreshes. Heartbeat goes
stale -> stop the rover, repeatedly. Heartbeat deleted -> the follower exited
cleanly, so exit too.
"""
from watchdog import Watchdog


class Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t


class Stopper:
    def __init__(self):
        self.stops = 0

    def __call__(self):
        self.stops += 1


def wd(clock, mtime, timeout=1.0):
    """Watchdog reading a heartbeat whose mtime the test controls."""
    stopper = Stopper()
    w = Watchdog(stop=stopper, heartbeat_mtime=lambda: mtime[0],
                 clock=clock, timeout_s=timeout)
    return w, stopper


def test_a_fresh_heartbeat_leaves_the_rover_alone():
    clock = Clock()
    mtime = [1000.0]
    w, stopper = wd(clock, mtime)
    assert w.tick() is True                      # keep watching
    assert stopper.stops == 0


def test_a_stale_heartbeat_stops_the_rover():
    clock = Clock()
    mtime = [1000.0]
    w, stopper = wd(clock, mtime)
    w.tick()
    clock.t += 2.0                               # controller went away
    assert w.tick() is True
    assert stopper.stops == 1


def test_it_keeps_stopping_while_the_heartbeat_stays_stale():
    """One stop is not enough: the command may not have reached the bot, and a
    latched throttle is exactly the failure being defended against."""
    clock = Clock()
    mtime = [1000.0]
    w, stopper = wd(clock, mtime)
    w.tick()
    for _ in range(5):
        clock.t += 2.0
        w.tick()
    assert stopper.stops == 5


def test_a_recovered_heartbeat_stops_the_stopping():
    clock = Clock()
    mtime = [1000.0]
    w, stopper = wd(clock, mtime)
    w.tick()
    clock.t += 2.0
    w.tick()
    assert stopper.stops == 1
    clock.t += 0.1
    mtime[0] = clock.t                           # controller is alive again
    w.tick()
    assert stopper.stops == 1


def test_a_deleted_heartbeat_is_a_clean_shutdown():
    clock = Clock()
    mtime = [1000.0]
    w, stopper = wd(clock, mtime)
    w.tick()
    mtime[0] = None                              # Commander.close() removed it
    assert w.tick() is False                     # stop watching
    assert stopper.stops == 0


def test_it_waits_for_a_heartbeat_that_has_not_appeared_yet():
    """Started before the follower: absence is not yet a crash."""
    clock = Clock()
    mtime = [None]
    w, stopper = wd(clock, mtime)
    for _ in range(5):
        clock.t += 1.0
        assert w.tick() is True
    assert stopper.stops == 0
