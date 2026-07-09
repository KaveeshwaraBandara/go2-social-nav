#!/usr/bin/env python3
"""Interval Type-2 Fuzzy Logic System (IT2-FLS) for social navigation.

This module is DELIBERATELY ROS-FREE. It is plain Python + math so you can:

    python3 it2_fls.py          # prints a control surface + a few sanity checks

on your host, unit-test it, and reason about the fuzzy logic on its own before it
ever touches Gazebo. The ROS 2 node (`it2fls_brain.py`) imports `SocialFLS` from
here and does nothing but wire it to /people, /odom and /cmd_vel.

------------------------------------------------------------------------------
WHAT AN INTERVAL TYPE-2 FUZZY SYSTEM IS (the 90-second version)
------------------------------------------------------------------------------
A *type-1* fuzzy set assigns each input x a single membership number mu(x) in
[0, 1] ("this distance is 0.7 NEAR"). A *type-2* set says even that membership is
uncertain: instead of one number you get a *band* [mu_lower(x), mu_upper(x)].
The area between the two curves is the FOOTPRINT OF UNCERTAINTY (FOU) - it is how
the maths represents "I am not even sure how NEAR this is", which is exactly the
sensor noise + human unpredictability the proposal talks about.

An *interval* type-2 system is the tractable special case where, inside that band,
every membership value is treated as equally possible (a flat interval). That buys
us a clean, well-known inference pipeline:

    fuzzify -> each rule fires with an INTERVAL [f_lower, f_upper]
            -> TYPE REDUCTION collapses the type-2 output back to a type-1
               interval [y_left, y_right]   (we use the Karnik-Mendel algorithm)
            -> defuzzify: crisp output = (y_left + y_right) / 2

If you delete the FOU (set lower == upper) this whole file degenerates to an
ordinary type-1 Mamdani system - which is a good way to convince yourself the
extra machinery is doing something.

------------------------------------------------------------------------------
THE CONTROLLER WE BUILD HERE
------------------------------------------------------------------------------
Inputs  (per person the robot can see):
    distance       d   [m]      how far away the person is
    closing_speed  vc  [m/s]    how fast the gap is shrinking (+ = approaching,
                                - = walking away).  This is the "predict where
                                they're heading" signal from the proposal.

Output:
    caution        c   in [0, 1]  how strongly to avoid this person right now.

The node turns `caution` into (a) how hard to steer away from that person and
(b) how much to slow down - i.e. a smooth, context-sensitive separation
distance, which is precisely Layer 1 of the WSO2 proposal.
"""
import math


# =============================================================================
# 1.  INTERVAL TYPE-2 MEMBERSHIP FUNCTIONS
# =============================================================================
# We use the textbook FOU: a Gaussian with an *uncertain mean*. The set's mean is
# not a point but an interval [m1, m2]; the standard deviation `sigma` is fixed.
#
#   upper MF  = 1                       for m1 <= x <= m2   (the "plateau")
#             = gaussian(x, m1, sigma)  for x < m1
#             = gaussian(x, m2, sigma)  for x > m2
#   lower MF  = gaussian(x, m2, sigma)  for x <= (m1+m2)/2
#             = gaussian(x, m1, sigma)  otherwise
#
# Widen [m1, m2] -> fatter FOU -> more uncertainty absorbed. Set m1 == m2 -> the
# band closes and you are back to a plain type-1 Gaussian.

def _gauss(x, mean, sigma):
    return math.exp(-0.5 * ((x - mean) / sigma) ** 2)


class IT2Gaussian:
    """One interval type-2 fuzzy set: Gaussian, uncertain mean [m1, m2]."""

    def __init__(self, m1, m2, sigma):
        self.m1 = min(m1, m2)
        self.m2 = max(m1, m2)
        self.sigma = sigma

    def upper(self, x):
        if x < self.m1:
            return _gauss(x, self.m1, self.sigma)
        if x > self.m2:
            return _gauss(x, self.m2, self.sigma)
        return 1.0

    def lower(self, x):
        mid = 0.5 * (self.m1 + self.m2)
        # The lower MF takes the *farther* mean, so it is the smaller of the two
        # Gaussians on each side - the pessimistic edge of the FOU.
        if x <= mid:
            return _gauss(x, self.m2, self.sigma)
        return _gauss(x, self.m1, self.sigma)

    def membership(self, x):
        """Return the interval [lower, upper] at x."""
        return self.lower(x), self.upper(x)


# =============================================================================
# 2.  KARNIK-MENDEL TYPE REDUCTION
# =============================================================================
# After every rule has fired with an interval [f_lower, f_upper] and carries a
# consequent whose centroid is an interval [c_left, c_right], we must collapse the
# type-2 output to a crisp number. KM finds the two endpoints of the type-reduced
# set:
#     y_left  = the SMALLEST possible weighted average of the consequent centroids
#     y_right = the LARGEST  possible weighted average
# by choosing, for each rule, whether to use its lower or upper firing strength.
# The crisp output is their midpoint.
#
# This is the heart of "why type-2": the spread (y_right - y_left) is a direct,
# usable measure of how uncertain the decision is.

def _km_endpoint(centroids, f_low, f_up, find_left):
    """One KM endpoint.

    centroids : consequent centroid to use for THIS endpoint (c_left for the
                left endpoint, c_right for the right endpoint), one per rule.
    f_low/f_up: per-rule firing-strength interval.
    find_left : True  -> compute y_left  (minimise)
                False -> compute y_right (maximise)
    """
    n = len(centroids)
    if n == 0:
        return 0.0

    # Sort rules by their consequent centroid (KM requires ascending order).
    order = sorted(range(n), key=lambda i: centroids[i])
    c = [centroids[i] for i in order]
    fl = [f_low[i] for i in order]
    fu = [f_up[i] for i in order]

    # Start from the mid firing strength and iterate to the switch point.
    f = [0.5 * (fl[i] + fu[i]) for i in range(n)]

    def weighted(fs):
        num = sum(fs[i] * c[i] for i in range(n))
        den = sum(fs[i] for i in range(n))
        return num / den if den > 1e-12 else 0.0

    y = weighted(f)
    for _ in range(100):  # KM converges in <= n steps; cap is just paranoia.
        # Find switch index k: c[k] <= y <= c[k+1].
        k = 0
        for i in range(n - 1):
            if c[i] <= y <= c[i + 1]:
                k = i
                break
        else:
            k = n - 1

        # To MINIMISE (left): put max weight on small centroids -> upper firing
        # left of the switch, lower firing right of it. To MAXIMISE, flip it.
        if find_left:
            f = [fu[i] if i <= k else fl[i] for i in range(n)]
        else:
            f = [fl[i] if i <= k else fu[i] for i in range(n)]

        y_new = weighted(f)
        if abs(y_new - y) < 1e-6:
            return y_new
        y = y_new
    return y


def karnik_mendel(f_low, f_up, c_left, c_right):
    """Type-reduce + defuzzify. Returns (crisp, y_left, y_right)."""
    y_left = _km_endpoint(c_left, f_low, f_up, find_left=True)
    y_right = _km_endpoint(c_right, f_low, f_up, find_left=False)
    return 0.5 * (y_left + y_right), y_left, y_right


# =============================================================================
# 3.  THE SOCIAL-NAVIGATION FLS
# =============================================================================
class SocialFLS:
    """2-input, 1-output interval type-2 Mamdani controller.

    inputs  : distance d [m], closing_speed vc [m/s]
    output  : caution in [0, 1]

    The rule base (9 rules) reads:

                         vc: RECEDING   STEADY     APPROACHING
        distance NEAR :      MED         HIGH       HIGH
        distance MED  :      LOW         MED        HIGH
        distance FAR  :      LOW         LOW        MED

    Near + approaching -> maximum caution; far + receding -> ignore. Everything
    in between is a smooth blend, which is the whole point.
    """

    def __init__(self):
        # --- Antecedent sets (tune the means/sigmas + FOU width here) ---------
        # distance [m]: the uncertain-mean interval is the FOU.
        self.d_near = IT2Gaussian(0.0, 0.5, 0.7)
        self.d_med = IT2Gaussian(1.3, 1.9, 0.7)
        self.d_far = IT2Gaussian(3.2, 4.0, 0.9)

        # closing speed [m/s]: negative = receding, positive = approaching.
        self.v_recede = IT2Gaussian(-0.9, -0.5, 0.5)
        self.v_steady = IT2Gaussian(-0.15, 0.15, 0.35)
        self.v_approach = IT2Gaussian(0.5, 0.9, 0.5)

        # --- Consequent sets: interval centroids [c_left, c_right] in [0,1] ----
        # The width of each interval is the output-side FOU.
        self.c_low = (0.00, 0.20)
        self.c_med = (0.40, 0.60)
        self.c_high = (0.80, 1.00)

        # --- Rule base: (distance set, vc set) -> consequent -------------------
        D = {"near": self.d_near, "med": self.d_med, "far": self.d_far}
        V = {"rec": self.v_recede, "steady": self.v_steady, "app": self.v_approach}
        C = {"low": self.c_low, "med": self.c_med, "high": self.c_high}
        table = [
            ("near", "rec", "med"), ("near", "steady", "high"), ("near", "app", "high"),
            ("med", "rec", "low"), ("med", "steady", "med"), ("med", "app", "high"),
            ("far", "rec", "low"), ("far", "steady", "low"), ("far", "app", "med"),
        ]
        self.rules = [(D[d], V[v], C[c]) for (d, v, c) in table]

    def infer(self, distance, closing_speed):
        """Run the full IT2 pipeline. Returns (caution, uncertainty).

        caution     : crisp output in [0, 1]
        uncertainty : y_right - y_left, the FOU-driven spread of the decision
                      (handy to log / visualise - it is what type-1 cannot give).
        """
        f_low, f_up, c_left, c_right = [], [], [], []
        for d_set, v_set, cons in self.rules:
            # Fuzzify each antecedent into its [lower, upper] membership interval.
            dl, du = d_set.membership(distance)
            vl, vu = v_set.membership(closing_speed)
            # PRODUCT t-norm to combine the two antecedents -> firing interval.
            f_low.append(dl * vl)
            f_up.append(du * vu)
            c_left.append(cons[0])
            c_right.append(cons[1])

        # If nothing fires (person off every scale) default to zero caution.
        if sum(f_up) < 1e-9:
            return 0.0, 0.0

        crisp, y_l, y_r = karnik_mendel(f_low, f_up, c_left, c_right)
        return max(0.0, min(1.0, crisp)), (y_r - y_l)


# =============================================================================
# 4.  STANDALONE SELF-TEST  (run: python3 it2_fls.py)
# =============================================================================
def _demo():
    fls = SocialFLS()

    print("Sanity checks (caution should rise as distance falls / closing rises):")
    cases = [
        ("far & walking away ", 4.0, -0.8),
        ("far & steady       ", 4.0, 0.0),
        ("medium & steady    ", 1.6, 0.0),
        ("medium & approaching", 1.6, 0.8),
        ("near & steady      ", 0.4, 0.0),
        ("near & approaching ", 0.4, 0.9),
    ]
    for label, d, vc in cases:
        c, unc = fls.infer(d, vc)
        print(f"  {label}: d={d:>4.1f} vc={vc:>+4.1f} -> caution={c:0.3f}  (uncertainty {unc:0.3f})")

    # Monotonicity spot-check: for fixed closing speed, closer must be >= caution.
    prev = -1.0
    ok = True
    for d in (4.0, 3.0, 2.0, 1.0, 0.3):
        c, _ = fls.infer(d, 0.3)
        if c < prev - 1e-6:
            ok = False
        prev = c
    print(f"\nMonotone-in-distance check @ vc=0.3: {'PASS' if ok else 'FAIL'}")

    print("\nControl surface  (rows = distance, cols = closing speed):")
    vcs = [-0.8, -0.4, 0.0, 0.4, 0.8]
    header = "  d\\vc |" + "".join(f"{v:>7.1f}" for v in vcs)
    print(header)
    print("  " + "-" * (len(header) - 2))
    for d in [0.3, 0.8, 1.5, 2.5, 3.5, 4.5]:
        row = "".join(f"{fls.infer(d, v)[0]:>7.2f}" for v in vcs)
        print(f"  {d:>4.1f} |{row}")


if __name__ == "__main__":
    _demo()
