"""What to do when the rover stops making progress.

Stuck detection used to end the run. On a sidewalk course that means the first
curb, pole, wet leaf or GPS glitch costs the whole mission — and the score is the
sum of completed missions, so an abort is worth zero.

The ladder, in order of how much it costs to be wrong:

  1. Back up and turn. Cheap, fixes the common cases: a wheel against a lip, a
     nose in a hedge, a corner cut too tight. Alternates direction between
     attempts, because if turning left did not free it, left is not the answer.
  2. Approach from a different angle (`waypoint_follower` drives to an offset
     point that is NOT a checkpoint). Fixes the cases where the direct line is
     blocked but the checkpoint is reachable.
  3. Give up honestly: record an intervention and stop.

`Recovery` owns only step 1 — it is a small time-based state machine so it can be
tested without a rover, a clock, or a network.
"""


class Recovery:
    PAUSE, REVERSE, YAW, DONE = "pause", "reverse", "yaw", "done"

    def __init__(self, cfg):
        self.cfg = cfg
        self.phase = self.DONE
        self.phase_t = 0.0
        self.attempts = 0
        self._yaw_sign = 1.0

    @property
    def active(self):
        return self.phase != self.DONE

    @property
    def exhausted(self):
        return self.attempts >= self.cfg.recovery_tries

    def begin(self, now):
        """Start one recovery attempt."""
        self.attempts += 1
        self.phase = self.PAUSE
        self.phase_t = now
        self._yaw_sign = -self._yaw_sign      # the other way this time

    def step(self, now):
        """Return (linear, angular, active) for this tick."""
        c = self.cfg
        if self.phase == self.DONE:
            return 0.0, 0.0, False
        elapsed = now - self.phase_t
        if self.phase == self.PAUSE:
            if elapsed >= c.recovery_pause_s:
                self.phase, self.phase_t = self.REVERSE, now
            return 0.0, 0.0, True
        if self.phase == self.REVERSE:
            if elapsed >= c.recovery_reverse_s:
                self.phase, self.phase_t = self.YAW, now
                return 0.0, 0.0, True
            return -abs(c.recovery_reverse_throttle), 0.0, True
        # YAW: turn in place, so the next approach starts from a new heading
        if elapsed >= c.recovery_yaw_s:
            self.phase = self.DONE
            return 0.0, 0.0, False
        return 0.0, self._yaw_sign * abs(c.recovery_yaw), True
