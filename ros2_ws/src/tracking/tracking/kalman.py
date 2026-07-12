"""
kalman.py — constant-velocity filtering for one track.

Why there is no 4x4 matrix here
-------------------------------
The textbook constant-velocity model has state [x, y, vx, vy] and a 4x4
transition. But for that model F, Q, H and R are all block-diagonal with
IDENTICAL 2x2 blocks, and the measurement touches only position. x never
influences y. So the 4-state filter is exactly two independent 2-state filters,
and the 4x4 form carries nothing but zeros and an excuse to depend on numpy.

Running two 2-state filters is not an approximation. It is the same algebra with
the structural zeros removed. It keeps `tracking/` on the standard library,
which is what lets track.py be copied into go2-social-nav untouched.

Per axis the state is [p, v]:

    predict:  p <- p + v*dt              F = [[1, dt],
              v <- v                          [0,  1]]
              P <- F P F' + Q(dt)

    update:   z is a position measurement with variance r
              (a scalar update, so "inverting" S is a division)

Process noise is the white-noise-acceleration model: an unmodelled acceleration
of intensity q (m^2/s^3) acting over dt. This is what lets the filter believe a
person can change speed, and it is the knob that trades smoothness against lag.

Units: metres, seconds.
"""
from __future__ import annotations

import math

#: Unmodelled acceleration intensity, m^2/s^3. A walking person accelerates at
#: well under 1 m/s^2 in normal gait; this admits that without letting the
#: filter chase centroid noise.
#:
#: Swept on the validation bag across q in 0.05..1.0 and R in 0.03..0.25. The
#: reported speeds barely move: walkers land at 0.9-1.4 m/s and standers under
#: 0.12 m/s everywhere in that box. 0.2 is picked from the middle of a flat
#: region, not from a sharp optimum — worth knowing, because it means this
#: number does not need re-tuning for a slightly different rig.
DEFAULT_PROCESS_NOISE = 0.2

#: 1-sigma of the RANDOM, frame-to-frame part of a centroid measurement, metres.
#:
#: Measured on the validation bag: over a 4.2 s plateau where a person stood
#: genuinely still, the fused centroid wobbled with an RMS of only 0.016 m. It
#: is small because a 0.8 s aggregation window sampled every 0.2 s shares 75% of
#: its LiDAR points with its neighbour, so successive centroids are heavily
#: smoothed and correlated (lag-1 autocorrelation +0.54 / +0.61).
#:
#: 0.08 rather than 0.016: a motionless torso is the best case. Once a person
#: turns or is partly occluded, DBSCAN latches onto a different subset of the
#: body and the centroid steps by several centimetres for reasons that have
#: nothing to do with walking. R must cover that, not just the still-torso floor.
DEFAULT_MEASUREMENT_STD = 0.08

#: 1-sigma of the SYSTEMATIC part of the centroid error, metres.
#:
#: The deskewed cloud aggregates returns over the preceding few hundred ms, so a
#: person reads ~0.2-0.4 m behind their true position; the independent RealSense
#: depth cross-check puts the median disagreement at ~0.28 m. This error does not
#: resample frame to frame, so the Kalman filter structurally cannot average it
#: away, and it never appears in P.
#:
#: Left out, `pos_std` would converge toward ~0.05 m and tell a safety-critical
#: consumer that a person's position is known an order of magnitude better than
#: it is. It is added in quadrature to the published `pos_std`, which therefore
#: has a floor at this value.
#:
#: A property of THIS sensor pipeline. HuNavSim ground truth passes 0.0 and the
#: floor disappears.
DEFAULT_POSITION_BIAS_STD = 0.29

#: 1-sigma of the initial velocity estimate, m/s. A newborn track has no idea
#: how fast the object is moving, so start humble and wide: this must comfortably
#: cover a brisk walk, or the filter will be slow to accept real motion.
DEFAULT_INITIAL_VEL_STD = 1.5


class _Axis:
    """A 2-state [position, velocity] Kalman filter along one axis."""

    __slots__ = ("p", "v", "p00", "p01", "p10", "p11", "q")

    def __init__(self, p0: float, meas_std: float, vel_std: float,
                 process_noise: float) -> None:
        self.p = p0
        self.v = 0.0
        # Covariance P, row-major. Position is known as well as one measurement
        # allows; velocity is wide open and uncorrelated with position.
        self.p00 = meas_std * meas_std
        self.p01 = 0.0
        self.p10 = 0.0
        self.p11 = vel_std * vel_std
        self.q = process_noise

    def predict(self, dt: float) -> None:
        if dt <= 0.0:
            return
        # State: constant velocity.
        self.p += self.v * dt

        # P <- F P F' with F = [[1, dt], [0, 1]]
        p00 = self.p00 + dt * (self.p10 + self.p01) + dt * dt * self.p11
        p01 = self.p01 + dt * self.p11
        p10 = self.p10 + dt * self.p11
        p11 = self.p11

        # + Q(dt), white-noise acceleration of intensity q.
        dt2 = dt * dt
        dt3 = dt2 * dt
        self.p00 = p00 + self.q * dt3 / 3.0
        self.p01 = p01 + self.q * dt2 / 2.0
        self.p10 = p10 + self.q * dt2 / 2.0
        self.p11 = p11 + self.q * dt

    def update(self, z: float, r: float) -> None:
        """Correct with a position measurement `z` of variance `r`."""
        y = z - self.p              # innovation
        s = self.p00 + r            # innovation variance (scalar)
        k0 = self.p00 / s           # Kalman gain
        k1 = self.p10 / s

        self.p += k0 * y
        self.v += k1 * y

        # P <- (I - K H) P, with H = [1, 0]
        p00, p01 = self.p00, self.p01
        self.p00 = p00 - k0 * p00
        self.p01 = p01 - k0 * p01
        self.p10 = self.p10 - k1 * p00
        self.p11 = self.p11 - k1 * p01

    @property
    def pos_var(self) -> float:
        return self.p00

    @property
    def vel_var(self) -> float:
        return self.p11


class ConstantVelocityFilter:
    """Planar constant-velocity filter: two decoupled per-axis filters.

    Position z is NOT filtered. Nothing in the motion model predicts height, and
    a person's centroid height is near-constant, so z is passed through from the
    latest measurement rather than smoothed with a model that does not apply.
    """

    def __init__(self, x: float, y: float, z: float,
                 measurement_std: float = DEFAULT_MEASUREMENT_STD,
                 initial_vel_std: float = DEFAULT_INITIAL_VEL_STD,
                 process_noise: float = DEFAULT_PROCESS_NOISE,
                 position_bias_std: float = DEFAULT_POSITION_BIAS_STD) -> None:
        self._r = measurement_std * measurement_std
        self._bias_var = position_bias_std * position_bias_std
        self._x = _Axis(x, measurement_std, initial_vel_std, process_noise)
        self._y = _Axis(y, measurement_std, initial_vel_std, process_noise)
        self.z = z

    # -- state ---------------------------------------------------------------
    @property
    def x(self) -> float:
        return self._x.p

    @property
    def y(self) -> float:
        return self._y.p

    @property
    def vx(self) -> float:
        return self._x.v

    @property
    def vy(self) -> float:
        return self._y.v

    @property
    def speed(self) -> float:
        return math.hypot(self.vx, self.vy)

    @property
    def filter_pos_std(self) -> float:
        """What the Kalman filter alone believes, ignoring correlated error."""
        return math.sqrt(0.5 * (self._x.pos_var + self._y.pos_var))

    @property
    def pos_std(self) -> float:
        """Isotropic 1-sigma position uncertainty, published on the contract.

        The RMS of the two axes' filter variance, PLUS the correlated centroid
        bias in quadrature. The axes stay near-equal because both see the same
        measurement and process noise, so collapsing them to one scalar loses
        little; the bias term is what stops this number from being a lie.

        Consequence: pos_std has a floor at `position_bias_std`. No number of
        observations drives it lower, because the error being averaged over does
        not resample. That is the physically correct statement.
        """
        return math.sqrt(0.5 * (self._x.pos_var + self._y.pos_var)
                         + self._bias_var)

    @property
    def vel_std(self) -> float:
        return math.sqrt(0.5 * (self._x.vel_var + self._y.vel_var))

    # -- cycle ---------------------------------------------------------------
    def predict(self, dt: float) -> None:
        """Propagate forward by `dt` seconds of REAL elapsed time.

        Called every frame, including frames where this track got no detection —
        that is exactly what makes a coasting track drift forward along its last
        known heading instead of freezing in place.
        """
        self._x.predict(dt)
        self._y.predict(dt)

    def update(self, x: float, y: float, z: float) -> None:
        """Correct with a measured position."""
        self._x.update(x, self._r)
        self._y.update(y, self._r)
        self.z = z
