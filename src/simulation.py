"""Closed-loop simulation helpers for P, PI, and PID controller comparison."""

import numpy as np

from src.second_order_system import SecondOrderSystem


def simulate_closed_loop(controller, setpoint, simulation_time, dt, wn=2.0, zeta=0.3):
    """Simulate one controller connected to the second-order plant.

    Args:
        controller: PIDController instance.
        setpoint: Step reference value.
        simulation_time: Total simulation duration in seconds.
        dt: Simulation time step in seconds.
        wn: Natural frequency of the second-order plant.
        zeta: Damping ratio of the second-order plant.

    Returns:
        A dictionary containing controller name, time, response, control signal,
        and setpoint arrays.
    """
    plant = SecondOrderSystem(wn=wn, zeta=zeta)
    controller.reset()

    time = np.arange(0.0, simulation_time + dt, dt)
    response = np.zeros_like(time)
    control_signal = np.zeros_like(time)
    setpoint_signal = np.full_like(time, setpoint)

    for index, _ in enumerate(time):
        measurement = plant.output
        control_input = controller.compute(setpoint, measurement, dt)
        output = plant.step(control_input, dt)

        control_signal[index] = control_input
        response[index] = output

    return {
        "name": controller.name,
        "time": time,
        "response": response,
        "control_signal": control_signal,
        "setpoint": setpoint_signal,
    }


def simulate_controller_comparison(
    controllers,
    setpoint,
    simulation_time,
    dt,
    wn=2.0,
    zeta=0.3,
):
    """Run closed-loop simulations for a list of controllers."""
    results = []
    for controller in controllers:
        result = simulate_closed_loop(
            controller=controller,
            setpoint=setpoint,
            simulation_time=simulation_time,
            dt=dt,
            wn=wn,
            zeta=zeta,
        )
        results.append(result)
    return results

