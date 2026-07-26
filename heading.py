"""Heading estimation for a GPS + magnetometer + 4G rover.

Why this is not just `bearing(previous_fix, current_fix)`:

GPS error is metres. Over a short baseline the difference between two fixes is
dominated by that error, so the "course" it yields is close to random — and
because noise alone moves a *parked* rover a metre, a naive estimator reports a
confident GPS heading while standing still. Measured on the previous
implementation (0.7 m baseline, sigma=1.5 m): ~81 deg median error while driving,
~88 deg while parked, and it preferred that number over the magnetometer on 95%
of steps.

So this is a complementary filter:

  predict   integrate yaw between fixes — gyro when the bot reports one,
            otherwise the commanded angular times a yaw-rate model
  correct   only from a course computed over a LONG baseline, only while the
            rover is demonstrably moving, only if it did not turn much while
            covering that baseline, and then only as a partial correction

The magnetometer seeds the filter before the first course fix and is never
trusted again after it — the SDK's `orientation` units are undocumented, which is
exactly why the old design tried to avoid it.

Source strings, reported for the run log: "mag" (pre-lock), "dr" (dead
reckoning between fixes), "gps" (a course correction landed this step).
"""
from geo import haversine_m, bearing_deg, wrap180


class HeadingEstimator:
    """Config is duck-typed on `waypoint_follower.Config` (all HEADING_* env knobs)."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.heading = None          # current estimate, degrees, 0=N 90=E
        self.locked = False          # has a GPS course ever corrected us?
        self.anchor = None           # (lat, lon, t) the current baseline started from
        self.turned_deg = 0.0        # |yaw| integrated since the anchor
        self.turn_signed = 0.0       # signed yaw since the anchor, for de-biasing
        self.odo_m = 0.0             # distance travelled since the anchor, from wheels
        self.n_fix = 0               # course corrections applied so far
        self.last_fix_t = None       # when the last correction landed
        self.last_t = None

    # ---------------- internals ----------------

    def _mag(self, orientation):
        return (self.cfg.heading_sign * orientation * self.cfg.heading_scale
                + self.cfg.heading_offset) % 360.0

    def _moving(self, cmd_linear, speed):
        """Demand commanded motion AND, when telemetry offers one, real wheel motion.
        A dropped RTM command or a high-centred rover looks 'moving' otherwise."""
        if abs(cmd_linear) < self.cfg.heading_min_linear:
            return False
        if speed is not None and abs(speed) < self.cfg.heading_min_speed:
            return False
        return True

    def _reset_anchor(self, lat, lon, now):
        self.anchor = (lat, lon, now)
        self.turned_deg = 0.0
        self.turn_signed = 0.0
        self.odo_m = 0.0

    def _gain(self):
        """Snap to the first course (it is the best absolute reference we have), then
        average harder as fixes accumulate, down to a floor. Fast lock-on without
        paying for it in steady-state jitter."""
        return max(self.cfg.heading_gain, 1.0 / (self.n_fix + 1))

    # ---------------- update ----------------

    def update(self, lat, lon, orientation=0.0, now=None, cmd_linear=0.0,
               cmd_angular=0.0, gyro_z_dps=None, speed=None):
        """Fold one telemetry sample in. Returns (heading_deg, source)."""
        if now is None:
            import time
            now = time.time()
        dt = 0.0 if self.last_t is None else max(0.0, min(2.0, now - self.last_t))
        self.last_t = now

        if self.heading is None:
            self.heading = self._mag(orientation)
            self._reset_anchor(lat, lon, now)
            self.last_fix_t = now
            return self.heading, "mag"

        # ---- predict ----
        yaw_rate = gyro_z_dps if gyro_z_dps is not None else cmd_angular * self.cfg.yaw_rate_dps
        self.heading = (self.heading + yaw_rate * dt) % 360.0
        self.turned_deg += abs(yaw_rate) * dt
        self.turn_signed += yaw_rate * dt
        # The magnetometer seeds the filter and is never read again: re-reading a
        # miscalibrated `orientation` every step is what makes a rover turn in place
        # forever. Dead reckoning from a wrong seed still terminates the turn, drives
        # forward, and lets the GPS course fix the absolute offset.
        source = "dr"

        # ---- correct ----
        if not self._moving(cmd_linear, speed):
            self._reset_anchor(lat, lon, now)          # never span a stop
            return self.heading, source

        # Odometry, not GPS, measures the baseline. Gating on the GPS-measured
        # displacement selects for samples where noise inflated it — precisely the
        # samples whose direction is least trustworthy.
        travelled = abs(speed) if speed is not None else abs(cmd_linear) * self.cfg.max_speed_mps
        self.odo_m += travelled * dt

        alat, alon, at = self.anchor
        if now - at > self.cfg.heading_anchor_max_age_s:
            self._reset_anchor(lat, lon, now)
            return self.heading, source

        if self.odo_m < self.cfg.heading_min_move_m:
            return self.heading, source

        # A turn does not make the chord useless, it makes it BIASED. Over a
        # constant-rate turn the chord bearing is the AVERAGE heading across the
        # window, so the heading at the end is chord + turn/2. De-bias rather than
        # discard: rejecting every curved sample is how the filter went blind on a
        # course made of curves (#44).
        # Measured from the last ACCEPTED fix, not from the anchor — the anchor
        # resets on every rejection, so anchoring the clock to it means a filter
        # that rejects forever also starves forever without noticing.
        starved = (now - self.last_fix_t) > self.cfg.heading_max_blind_s

        # Gate on NET rotation, not on accumulated |yaw| (#54). What breaks the
        # "chord bearing == heading" relationship is turning that the de-bias below
        # cannot account for. Back-and-forth wobble — which is what a controller does
        # when GPS noise makes the bearing jitter — averages out: it contributes ~0 to
        # the de-bias and does not bend the path. Summing |yaw| counted that wobble as
        # hard turning and starved the filter exactly when the noise made corrections
        # most valuable. The absolute sum survives only as a loose sanity bound.
        net_turn = abs(self.turn_signed)
        too_curved = (net_turn > self.cfg.heading_max_turn_deg
                      or self.turned_deg > self.cfg.heading_max_abs_turn_deg)
        if too_curved and not starved:
            self._reset_anchor(lat, lon, now)          # past this the chord IS meaningless
            return self.heading, source

        chord = haversine_m(alat, alon, lat, lon)
        if chord < self.cfg.heading_slip_ratio * self.odo_m:
            self._reset_anchor(lat, lon, now)          # wheels turning, rover isn't
            return self.heading, source

        course = (bearing_deg(alat, alon, lat, lon) + self.turn_signed / 2.0) % 360.0
        # Starved samples are the ones we would normally reject; trust them less.
        gain = self._gain() * (0.5 if starved else 1.0)
        self.heading = (self.heading + gain * wrap180(course - self.heading)) % 360.0
        self.n_fix += 1
        self.last_fix_t = now
        self.locked = True
        self._reset_anchor(lat, lon, now)
        return self.heading, "gps"
