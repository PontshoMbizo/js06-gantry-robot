"""Position feedback model: transfer-function dynamics and sensor non-idealities.

Converts Gazebo joint position to the controller unit and can apply
quantization, delay and noise. Current configuration approximates the
project slot-sensor resolution; it does not claim a measured noise model.
"""

import numpy as np

from gantry_control.difference_equation import DifferenceEquationBlock


class SensorModel(DifferenceEquationBlock):

    def __init__(self, b, a, noise_std=0.0, quantization=0.0,
                 delay_samples=0, rng=None):
        super().__init__(b, a, sat_min=None, sat_max=None)
        self.noise_std = noise_std
        self.quantization = quantization
        self._delay_samples = delay_samples
        self._delay_buf = [0.0] * delay_samples if delay_samples > 0 else None
        self._rng = rng if rng is not None else np.random.default_rng()

    def reset(self):
        super().reset()
        if self._delay_buf is not None:
            self._delay_buf = [0.0] * self._delay_samples

    def update(self, x_n: float) -> float:
        y = super().update(x_n)

        if self._delay_buf is not None:
            self._delay_buf.append(y)
            y = self._delay_buf.pop(0)

        if self.noise_std:
            y += self._rng.normal(0.0, self.noise_std)

        if self.quantization:
            y = round(y / self.quantization) * self.quantization

        return y
