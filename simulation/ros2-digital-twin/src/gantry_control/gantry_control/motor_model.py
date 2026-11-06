"""First-order voltage-to-linear-velocity model from unloaded motor ID."""

from gantry_control.difference_equation import DifferenceEquationBlock


class MotorModel:
    def __init__(self, b, a, deadband=0.0, velocity_min=-0.1, velocity_max=0.1):
        self._block = DifferenceEquationBlock(b, a)
        self.deadband = float(deadband)
        self.velocity_min = float(velocity_min)
        self.velocity_max = float(velocity_max)

    def reset(self):
        self._block.reset()

    def update(self, voltage):
        magnitude = abs(voltage)
        if magnitude <= self.deadband:
            effective_voltage = 0.0
        else:
            effective_voltage = (1.0 if voltage >= 0 else -1.0) * (magnitude - self.deadband)
        velocity = self._block.update(effective_voltage)
        return max(self.velocity_min, min(self.velocity_max, velocity))
