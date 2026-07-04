"""Fully worked example dynamics for a 1D mass-spring-damper -- NOT a
pendulum, NOT graded content. See docs/example_workflow/README.md for the
derivation this implements.

    m * x_ddot = -k * x - b * x_dot + F

This mirrors the exact function-signature style used in
single_inverted/dynamics.py and double_inverted/dynamics.py, so you can
see the target code shape for translating a derivation into a Python
function. Everything here is filled in on purpose -- copy the *shape*,
not the content, into your graded work.
"""
from __future__ import annotations

import numpy as np


def x_ddot(x: float, x_dot: float, F: float, params: dict) -> float:
    """Nonlinear (here, actually linear) equation of motion.

    params must contain keys: m, k, b.
    """
    m = params['m']
    k = params['k']
    b = params['b']
    return (-k * x - b * x_dot + F) / m


def rk4_step(x: float, x_dot: float, F: float, params: dict, dt: float):
    """One RK4 integration step -- same pattern used in
    single_inverted/sim_node.py, shown here standalone for the sanity-test
    step (docs/example_workflow/README.md Step 5).
    """
    def deriv(xx, xxd):
        return xxd, x_ddot(xx, xxd, F, params)

    k1 = deriv(x, x_dot)
    k2 = deriv(x + 0.5 * dt * k1[0], x_dot + 0.5 * dt * k1[1])
    k3 = deriv(x + 0.5 * dt * k2[0], x_dot + 0.5 * dt * k2[1])
    k4 = deriv(x + dt * k3[0], x_dot + dt * k3[1])
    x_next = x + (dt / 6.0) * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
    x_dot_next = x_dot + (dt / 6.0) * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])
    return x_next, x_dot_next


if __name__ == '__main__':
    # Sanity check: undamped free response should oscillate at
    # omega_n = sqrt(k/m), period T = 2*pi/omega_n.
    params = {'m': 1.0, 'k': 4.0, 'b': 0.0}
    dt = 0.001
    x, x_dot = 1.0, 0.0
    expected_period = 2 * np.pi / np.sqrt(params['k'] / params['m'])

    t = 0.0
    prev_x = x
    zero_crossings = []
    while t < 2 * expected_period:
        x, x_dot = rk4_step(x, x_dot, 0.0, params, dt)
        if prev_x > 0 and x <= 0:
            zero_crossings.append(t)
        prev_x = x
        t += dt

    if len(zero_crossings) >= 2:
        measured_half_period = zero_crossings[1] - zero_crossings[0]
        print(f'expected period: {expected_period:.4f}s, '
              f'measured half-period x2: {2 * measured_half_period:.4f}s')
    else:
        print('not enough zero crossings captured -- lengthen the sim window')
