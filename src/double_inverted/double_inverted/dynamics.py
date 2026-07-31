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
    # 1. Extract State and Controls
    theta1, theta1_dot, theta2, theta2_dot = state
    tau1, tau2 = tau

    # 2. Extract Parameters
    # Explicit modeling choice: default lc to l/2 if not explicitly provided in params
    m1 = params['m1']
    l1 = params['l1']
    lc1 = params.get('lc1', l1 / 2.0) 
    I1 = params['I1']
    b1 = params['b1']

    m2 = params['m2']
    l2 = params['l2']
    lc2 = params.get('lc2', l2 / 2.0)
    I2 = params['I2']
    b2 = params['b2']

    g = params['g']

    # Precompute trigonometric terms
    sin_theta1 = np.sin(theta1)
    cos_theta1 = np.cos(theta1)
    sin_theta2 = np.sin(theta2)
    cos_theta2 = np.cos(theta2)
    sin_theta12 = np.sin(theta1 + theta2)

    # 3. Mass Matrix (M) components
    alpha = I1 + I2 + m1 * lc1**2 + m2 * (l1**2 + lc2**2)
    beta = m2 * l1 * lc2
    delta = I2 + m2 * lc2**2

    M11 = alpha + 2 * beta * cos_theta2
    M12 = delta + beta * cos_theta2
    M21 = M12
    M22 = delta

    # Closed-form 2x2 Matrix Inversion (M^-1)
    det_M = M11 * M22 - M12 * M21
    
    M_inv_11 = M22 / det_M
    M_inv_12 = -M12 / det_M
    M_inv_21 = -M21 / det_M
    M_inv_22 = M11 / det_M

    # 4. Coriolis/Centrifugal Matrix (C) components
    C11 = -beta * sin_theta2 * theta2_dot
    C12 = -beta * sin_theta2 * (theta1_dot + theta2_dot)
    C21 = beta * sin_theta2 * theta1_dot
    C22 = 0.0

    # Matrix multiplication: C @ theta_dot
    C_theta_dot_1 = C11 * theta1_dot + C12 * theta2_dot
    C_theta_dot_2 = C21 * theta1_dot + C22 * theta2_dot

    # 5. Gravity Vector (G) components
    G1 = -(m1 * lc1 + m2 * l1) * g * sin_theta1 - m2 * lc2 * g * sin_theta12
    G2 = -m2 * lc2 * g * sin_theta12

    # 6. Damping Vector (b_vec)
    damp1 = b1 * theta1_dot
    damp2 = b2 * theta2_dot

    # 7. Solve for Accelerations
    # theta_ddot = M^-1 @ (tau - C @ theta_dot - G - b_vec)
    RHS1 = tau1 - C_theta_dot_1 - G1 - damp1
    RHS2 = tau2 - C_theta_dot_2 - G2 - damp2

    theta1_ddot = M_inv_11 * RHS1 + M_inv_12 * RHS2
    theta2_ddot = M_inv_21 * RHS1 + M_inv_22 * RHS2

    return np.array([theta1_ddot, theta2_ddot])