"""Nonlinear dynamics for the double (2-link) inverted pendulum. TODO (trainee).

The equations of motion are GIVEN to you -- see
docs/eom_derivation.md for the fully worked 2-link Lagrangian derivation
(fully-actuated, both joints driven). Your job here is TRANSLATION, not
derivation: turn the given closed-form equations into a correct Python
implementation. This is still real work -- matching state/sign/variable
conventions exactly, structuring the mass-matrix/Coriolis/gravity terms
correctly, and avoiding transcription errors -- but it is not a from-
scratch derivation exercise. See README.md Sec 5.1.

State convention (must match custom_msgs/PendulumState2.msg):
    state = [theta1, theta1_dot, theta2, theta2_dot]
    theta1: base joint angle, 0 = upright
    theta2: elbow joint angle, relative to link 1
    tau = [tau1, tau2]
"""
from __future__ import annotations

import numpy as np


def theta_ddot(state: np.ndarray, tau: np.ndarray, params: dict) -> np.ndarray:
    """TODO (trainee): translate the equations in docs/eom_derivation.md.

    params should contain keys for both links: m1, l1, I1, m2, l2, I2,
    b1, b2, g (extend as docs/eom_derivation.md requires -- e.g. lc1/lc2
    if center-of-mass distance is tracked separately from link length).

    Return np.array([theta1_ddot, theta2_ddot]).
    """
    raise NotImplementedError(
        'double_inverted/dynamics.py::theta_ddot is a trainee deliverable -- '
        'translate the given equations in docs/eom_derivation.md, see README.md Sec 5.1')

