"""Nonlinear dynamics for the single inverted pendulum. GIVEN -- do not modify.

theta = 0 is upright (unstable equilibrium), theta = pi is hanging
straight down (stable equilibrium). theta increases counter-clockwise.

    I * theta_ddot = m * g * l * sin(theta) - b * theta_dot + tau
"""
from __future__ import annotations

import numpy as np


def theta_ddot(theta: float, theta_dot: float, tau: float, params: dict) -> float:
    """Nonlinear equation of motion. GIVEN -- do not modify.

    params must contain keys: m, l, I, b, g.
    """
    m = params['m']
    l = params['l']
    I = params['I']
    b = params['b']
    g = params['g']
    return (m * g * l * np.sin(theta) - b * theta_dot + tau) / I

