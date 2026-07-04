"""Symbolic mechanics helper toolkit -- GIVEN, sanctioned yellow-zone tool.

This is a general-purpose Lagrangian-mechanics utility, not a pendulum-
specific answer key. It does NOT know anything about pendulums and will
not tell you whether your physics *model* (what forces/energies you
included) is correct. What it does do:

  1. `euler_lagrange_eom(L, q, t)` -- given a Lagrangian L = T - V that
     YOU build symbolically (in terms of generalized coordinates you
     choose), mechanically applies the Euler-Lagrange equation
     d/dt(dL/dq_dot) - dL/dq = 0 for each coordinate. You still have to:
       - derive T and V yourself (this is the actual physics thinking)
       - add generalized forces (torque, damping) to the result yourself
       - solve the resulting (possibly coupled) equations for the
         accelerations yourself
     Use this to cross-check the *algebra* of a derivation you already
     did by hand, not to skip doing it.

  2. `compare_numeric(your_func, reference_expr, state_vars, input_vars,
     params, param_symbols, n_samples)` -- randomly samples states/inputs
     and compares YOUR numeric Python implementation against a symbolic
     reference expression YOU built (e.g. from step 1, or transcribed
     from a given derivation like double_inverted/docs/eom_derivation.md),
     reporting the max absolute difference. Useful for sanity-checking
     dynamics.py functions.

  3. `linearize_symbolic(f_exprs, state_vars, input_vars, x_eq, u_eq)` --
     given symbolic nonlinear state-derivative expressions x_dot =
     f(x, u), computes the Jacobians (A, B) at an equilibrium you
     specify. Not required by any current exercise (LQR/linearization
     were dropped from this track), but left in as a general-purpose
     utility in case you want it for your own understanding or a stretch
     goal.

This tool is scoped to README.md Sec 7 (yellow zone): "verifying a
hand-derived EOM against a symbolic tool." Using it does not remove the
requirement to submit your by-hand derivation (single pendulum, Week 1
swing-up argument) or your MPC formulation writeup, and its use should be
logged in docs/ai_log.md like any other yellow-zone tool.

See docs/example_workflow/ for a fully worked (non-pendulum) example of
the whole paper -> symbolic-check -> code pipeline using this toolkit.
"""
from __future__ import annotations

import random

import numpy as np
import sympy as sp


def euler_lagrange_eom(L: sp.Expr, q: list, t: sp.Symbol) -> list:
    """Apply the Euler-Lagrange equation to a Lagrangian you built.

    Args:
        L: Lagrangian expression (T - V), built from your own T and V, in
           terms of sympy Function(t) generalized coordinates and their
           time derivatives.
        q: list of generalized coordinates, e.g. [theta1(t), theta2(t)]
           (each a sympy Function of t).
        t: the sympy Symbol representing time.

    Returns:
        List of expressions, one per coordinate, equal to
        d/dt(dL/dq_dot) - dL/dq. These equal your generalized forces
        (torque, damping, etc.) once you add those in and set the
        expression to zero -- that step is yours to do by hand.
    """
    eoms = []
    for qi in q:
        qdot = qi.diff(t)
        dL_dqdot = sp.diff(L, qdot)
        d_dt_dL_dqdot = sp.diff(dL_dqdot, t)
        dL_dq = sp.diff(L, qi)
        eoms.append(sp.simplify(d_dt_dL_dqdot - dL_dq))
    return eoms


def compare_numeric(your_func, reference_expr, state_vars: list, input_vars: list,
                     params: dict, param_symbols: dict, n_samples: int = 50,
                     state_range=(-3.14, 3.14), input_range=(-5.0, 5.0), seed: int = 0):
    """Compare a numeric Python function against a symbolic reference.

    Args:
        your_func: callable your_func(*state_values, *input_values, params)
            -> array-like, matching your dynamics.py function's signature
            style (adapt the lambda you pass in to match your actual
            function's argument order).
        reference_expr: sympy Matrix/expr (or list of exprs) for the same
            quantity, in terms of state_vars, input_vars, and
            param_symbols.
        state_vars, input_vars: sympy Symbols appearing in reference_expr.
        params: dict of numeric parameter values (e.g. m, l, I, b, g).
        param_symbols: dict mapping the same keys to sympy Symbols used in
            reference_expr.
        n_samples: number of random (state, input) samples to check.
        state_range, input_range: uniform sampling ranges.
        seed: RNG seed for reproducibility.

    Returns:
        dict with 'max_abs_diff' and the sample that produced it -- does
        NOT tell you which side is "right," only whether they agree.
    """
    rng = random.Random(seed)
    ref_lamb = sp.lambdify(
        list(state_vars) + list(input_vars) + list(param_symbols.values()),
        reference_expr, 'numpy')

    max_diff = -1.0
    worst_sample = None
    for _ in range(n_samples):
        state_sample = [rng.uniform(*state_range) for _ in state_vars]
        input_sample = [rng.uniform(*input_range) for _ in input_vars]
        param_values = [params[k] for k in param_symbols]

        ref_val = np.array(ref_lamb(*state_sample, *input_sample, *param_values),
                            dtype=float).flatten()
        your_val = np.array(your_func(*state_sample, *input_sample, params),
                             dtype=float).flatten()

        diff = float(np.max(np.abs(ref_val - your_val)))
        if diff > max_diff:
            max_diff = diff
            worst_sample = {'state': state_sample, 'input': input_sample}

    return {'max_abs_diff': max_diff, 'worst_sample': worst_sample}


def linearize_symbolic(f_exprs: list, state_vars: list, input_vars: list,
                        x_eq: list, u_eq: list):
    """Jacobian-linearize x_dot = f(x, u) about an equilibrium you choose.

    Not required by any current exercise in this track -- provided as a
    general-purpose utility only.

    Args:
        f_exprs: list of sympy expressions for x_dot, in terms of
            state_vars and input_vars (symbols, not Functions -- this is
            the algebraic/already-substituted form, not the Function(t)
            form used in euler_lagrange_eom).
        state_vars: list of sympy Symbols, e.g. [theta, theta_dot].
        input_vars: list of sympy Symbols, e.g. [tau].
        x_eq: numeric equilibrium values for state_vars.
        u_eq: numeric equilibrium values for input_vars.

    Returns:
        (A, B) as sympy Matrices, evaluated numerically at the given
        equilibrium.
    """
    f = sp.Matrix(f_exprs)
    A = f.jacobian(state_vars)
    B = f.jacobian(input_vars)
    subs = dict(zip(state_vars, x_eq))
    subs.update(dict(zip(input_vars, u_eq)))
    return A.subs(subs), B.subs(subs)
