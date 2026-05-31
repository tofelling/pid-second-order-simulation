"""Second-order dynamic system implemented with explicit Euler integration."""


class SecondOrderSystem:
    """Numerical model of a standard second-order plant.

    The transfer function is:

        G(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)

    State variables:
        x1 = y       (system output)
        x2 = dy/dt   (output velocity)

    Continuous-time state equations:
        dx1/dt = x2
        dx2/dt = wn^2*u - 2*zeta*wn*x2 - wn^2*x1
    """

    def __init__(self, wn=2.0, zeta=0.3):
        """Create a second-order system with zero initial conditions."""
        self.wn = wn
        self.zeta = zeta
        self.x1 = 0.0
        self.x2 = 0.0

    @property
    def output(self):
        """Return the current system output y."""
        return self.x1

    def reset(self):
        """Reset the system state to zero initial conditions."""
        self.x1 = 0.0
        self.x2 = 0.0

    def step(self, control_input, dt):
        """Advance the system by one time step.

        Args:
            control_input: Controller output u applied to the plant.
            dt: Simulation time step in seconds.

        Returns:
            The updated system output y.
        """
        dx1 = self.x2
        dx2 = (
            self.wn**2 * control_input
            - 2.0 * self.zeta * self.wn * self.x2
            - self.wn**2 * self.x1
        )

        # Explicit Euler update: next_state = current_state + derivative * dt.
        self.x1 += dx1 * dt
        self.x2 += dx2 * dt

        return self.x1

