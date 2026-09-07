"""ROS-independent acceleration limiting for final drive commands."""


class AccelerationRamp:
    """Limit throttle increases while preserving immediate safe stops."""

    def __init__(self, acceleration_rate=2.0):
        self.acceleration_rate = float(acceleration_rate)
        if self.acceleration_rate <= 0.0:
            raise ValueError('acceleration_rate must be positive')
        self.output = 0.0
        self.last_time = None

    def apply(self, target, now):
        """Return a rate-limited target; exact zero is always immediate."""
        target = float(target)
        now = float(now)

        if target == 0.0:
            self.reset(now)
            return 0.0

        if self.last_time is None:
            self.last_time = now
            self.output = 0.0
            return 0.0

        elapsed = max(0.0, now - self.last_time)
        self.last_time = now

        if self.output != 0.0 and target * self.output < 0.0:
            self.output = 0.0
            return 0.0

        if abs(target) <= abs(self.output):
            self.output = target
            return self.output

        maximum_increase = self.acceleration_rate * elapsed
        output_magnitude = min(
            abs(target),
            abs(self.output) + maximum_increase,
        )
        self.output = output_magnitude if target > 0.0 else -output_magnitude
        return self.output

    def reset(self, now=None):
        """Force exact zero and optionally establish a new time origin."""
        self.output = 0.0
        self.last_time = None if now is None else float(now)
