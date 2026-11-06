"""Generic discrete transfer function (difference equation) block.

Realizes, in direct form I, any LTI discrete transfer function

    Y(z)   b[0] + b[1] z^-1 + ... + b[m] z^-m
    ---- = -----------------------------------
    X(z)   a[0] + a[1] z^-1 + ... + a[n] z^-n

as the recursive difference equation

    a[0]*y[n] = sum_i b[i]*x[n-i] - sum_{j>=1} a[j]*y[n-j]

Coefficients are typically obtained by discretizing a continuous-domain
design C(s) at a chosen sample time Ts (e.g. via Tustin/bilinear, ZOH, or
matched-Z transform) -- see discretize_helper.py for an optional scipy-based
convenience script. This class only evaluates the resulting b/a coefficients;
it has no opinion on how they were derived.

Reused, unmodified, as the base of both SensorModel (feedback path) and
MotorModel (actuation path).
"""

from collections import deque


class DifferenceEquationBlock:

    def __init__(self, b, a, sat_min=None, sat_max=None):
        if not a or a[0] == 0.0:
            raise ValueError('a[0] must be a nonzero leading denominator coefficient')
        if not b:
            raise ValueError('b must have at least one coefficient')

        a0 = a[0]
        self.b = [bi / a0 for bi in b]
        self.a = [ai / a0 for ai in a]
        self.sat_min = sat_min
        self.sat_max = sat_max

        self._x_hist = deque([0.0] * len(self.b), maxlen=len(self.b))
        self._y_hist = deque([0.0] * (len(self.a) - 1), maxlen=max(len(self.a) - 1, 0))

    def reset(self):
        self._x_hist = deque([0.0] * len(self.b), maxlen=len(self.b))
        self._y_hist = deque([0.0] * (len(self.a) - 1), maxlen=max(len(self.a) - 1, 0))

    def update(self, x_n: float) -> float:
        self._x_hist.appendleft(x_n)

        y_n = sum(bi * xi for bi, xi in zip(self.b, self._x_hist))
        y_n -= sum(aj * yj for aj, yj in zip(self.a[1:], self._y_hist))

        if self.sat_min is not None or self.sat_max is not None:
            lo = self.sat_min if self.sat_min is not None else float('-inf')
            hi = self.sat_max if self.sat_max is not None else float('inf')
            y_n = max(lo, min(hi, y_n))

        if self._y_hist.maxlen:
            self._y_hist.appendleft(y_n)

        return y_n
