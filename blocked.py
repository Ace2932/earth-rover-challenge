""""Is something in the way?" — the decision, and the weak label it is trained on.

The Urban track is sidewalks, crossings and pedestrians, and the policy regressed
steering only. Nothing in the stack could say "do not go forward", so nothing
could stop for anything.

Two pieces, deliberately separated from the model:

`BlockedGate` is the decision. It has hysteresis in both directions, because at
5 Hz a raw threshold produces stop/go/stop chatter, which is worse for a
pedestrian than either a clean stop or no stop at all. It also fails open: no
probability (no head, failed inference) never brakes the rover.

`blocked_labels` turns teleop into supervision. There is no obstacle annotation
in FrodoBots-2K, but there is a human who stopped. "Was moving, now stopped" is a
real signal about the scene ahead — and a WEAK one: humans also stop for red
lights, to get their bearings, or because they got bored. Treat the resulting
model as a prior, never as a safety certificate.

Latency matters too: ~500 ms of video latency plus the control period means the
rover travels ~0.5 m at cruise between a hazard appearing and any reaction.
"""


class BlockedGate:
    def __init__(self, cfg):
        self.cfg = cfg
        self.run = 0                 # consecutive frames over the threshold
        self.held_until = None

    def update(self, probability, now):
        """Return True if forward motion should be suppressed right now."""
        c = self.cfg
        if c.blocked_p >= 1.0:                      # disabled
            return False
        if self.held_until is not None and now < self.held_until:
            return True                             # hold, do not chatter
        if probability is None:
            self.run = 0                            # no opinion -> never brake
            return False
        if probability >= c.blocked_p:
            self.run += 1
        else:
            self.run = 0
        if self.run >= c.blocked_frames:
            self.held_until = now + c.blocked_hold_s
            return True
        return False


def blocked_labels(linear, hz=10.0, moving=0.3, stopped=0.05, lookback_s=1.0):
    """Weak supervision from teleop: 1 where the human was moving and has just
    stopped, 0 elsewhere.

    Not "there is an obstacle" — nobody labelled that. It is "the driver braked
    here", which correlates with obstacles and also with traffic lights, hesitation
    and boredom. The long tail is excluded by `lookback_s`: a rover parked for ten
    seconds is waiting, not braking.
    """
    span = max(1, int(lookback_s * hz))
    out = []
    for i, v in enumerate(linear):
        if abs(v) > stopped:
            out.append(0)
            continue
        window = linear[max(0, i - span):i]
        out.append(1 if any(abs(w) > moving for w in window) else 0)
    return out
