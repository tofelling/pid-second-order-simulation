"""PID controller implementation for closed-loop simulations."""


class PIDController:
    """A simple PID controller that can also act as P or PI controller.

    Control law:

        u(t) = Kp*e(t) + Ki*integral(e) + Kd*de/dt

    The derivative term is initialized to zero on the first step. This avoids
    a large artificial derivative kick caused only by the lack of previous
    error history at t = 0.
    """

    def __init__(self, kp, ki=0.0, kd=0.0, name="PID"):
        """Create a PID controller with the given gains."""
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.name = name
        self.integral = 0.0
        self.previous_error = None

    def reset(self):
        """Clear stored integral and previous error values."""
        self.integral = 0.0
        self.previous_error = None

    def compute(self, setpoint, measurement, dt):
        """Compute the controller output for one simulation step.

        Args:
            setpoint: Desired reference value.
            measurement: Current measured system output.
            dt: Simulation time step in seconds.

        Returns:
            Controller output u.
        """
        error = setpoint - measurement

        proportional = self.kp * error

        self.integral += error * dt
        integral = self.ki * self.integral

        if self.previous_error is None:
            derivative_error = 0.0
        else:
            derivative_error = (error - self.previous_error) / dt
        derivative = self.kd * derivative_error

        self.previous_error = error

        return proportional + integral + derivative

