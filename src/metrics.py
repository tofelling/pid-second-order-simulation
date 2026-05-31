"""Performance metrics for step-response analysis."""

import numpy as np


def calculate_overshoot(response, setpoint):
    """Calculate percent overshoot relative to the setpoint.

    Returns 0 when the response never exceeds the setpoint.
    """
    peak_value = float(np.max(response))
    if setpoint == 0:
        return 0.0
    overshoot = max(0.0, (peak_value - setpoint) / abs(setpoint) * 100.0)
    return overshoot


def calculate_final_window_tracking_error(response, setpoint, sample_count=100):
    """Estimate final tracking behavior over a finite simulation window.

    This is the absolute difference between the setpoint and the average of
    the final samples. It is a practical finite-horizon approximation, not a
    strict t -> infinity steady-state error.
    """
    if len(response) == 0:
        return 0.0

    final_window = response[-min(sample_count, len(response)) :]
    final_value = float(np.mean(final_window))
    return abs(setpoint - final_value)


def calculate_settling_time(time, response, setpoint, tolerance=0.02):
    """Calculate settling time using a percentage error band.

    The settling time is the first time after which the response remains inside
    +/- tolerance of the setpoint for the rest of the simulation.
    """
    if len(time) == 0:
        return 0.0

    band = abs(setpoint) * tolerance
    if band == 0:
        band = tolerance

    error = np.abs(response - setpoint)
    outside_band = np.where(error > band)[0]

    if len(outside_band) == 0:
        return float(time[0])

    last_outside_index = int(outside_band[-1])
    if last_outside_index >= len(time) - 1:
        return None

    return float(time[last_outside_index + 1])


def calculate_metrics(time, response, setpoint):
    """Return a dictionary of common step-response performance metrics."""
    return {
        "overshoot_percent": calculate_overshoot(response, setpoint),
        "final_window_tracking_error": calculate_final_window_tracking_error(
            response,
            setpoint,
        ),
        "settling_time_seconds": calculate_settling_time(time, response, setpoint),
    }
