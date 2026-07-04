# MPC primer (GIVEN background reading)

You have not implemented MPC before. This document gives you the
conceptual background you need before attempting the Week 2/3 MPC
formulation exercises -- it is **not** the formulation exercise itself,
and it contains no pendulum-specific content. The cost function,
constraints, and control-loop-frequency justification you write for
`controller_mpc.py` are still 100% your own reasoning (README.md Sec 7,
red zone).

## What MPC is

Model Predictive Control repeatedly solves a finite-horizon optimal
control problem online. At each control step:

1. Measure (or estimate) the current state `x0`.
2. Solve: find the control sequence `u_0, u_1, ..., u_{N-1}` over the
   next `N` steps that minimizes a cost function, subject to the system
   dynamics and any constraints (actuator limits, state limits).
3. Apply only the **first** control `u_0` to the real system.
4. Discard the rest of the planned sequence, move one step forward, and
   re-solve from the new measured state at the next control step.

This "solve, apply first step, re-solve" pattern is called **receding
horizon control** -- it's what makes MPC robust to model mismatch and
disturbances despite planning with an imperfect model: you never
committed to the whole plan, only the first step of it.

## Why a horizon, and what "N" means

`N` is the number of discrete steps the optimizer looks ahead. A longer
horizon can produce better-informed decisions (e.g. it "sees" that
slowing down now avoids overshoot later) but costs more to solve. Horizon
length and discretization step `dt` together determine how far into the
future (in seconds) the controller is actually planning: `N * dt`.

## Discretization

The pendulum dynamics you have (`theta_ddot(...)`) are continuous-time
(an ODE). The optimizer needs a discrete-time model: given `x_k`, predict
`x_{k+1}` after one step of size `dt`. You already know one way to do
this -- the same RK4 integration used in `sim_node.py` -- just applied
symbolically inside the optimization instead of numerically inside a ROS
timer callback.

## Multiple shooting vs. single shooting

Two common ways to wire the discretized dynamics into the optimization:

- **Single shooting:** only the initial state and the control sequence
  are decision variables; every future state is computed by repeatedly
  applying the discrete dynamics starting from `x0`. Simple, but for
  nonlinear/unstable systems (like an upright pendulum) small control
  changes can produce wildly different long-horizon state predictions,
  which makes the optimization numerically harder.
- **Multiple shooting:** every predicted state `x_0, x_1, ..., x_N` is
  *also* a decision variable, and you add equality constraints forcing
  each `x_{k+1}` to match the discrete-dynamics prediction from `x_k` and
  `u_k`. This is what `controller_mpc.py`'s scaffold (single pendulum) and
  the given shooting skeleton (double pendulum) already set up for you --
  it's generally better-conditioned for unstable nonlinear systems, which
  is why it's the required approach here.

## Cost function

Typically a sum over the horizon of a **stage cost** (how far is the
state from where you want it, how much control effort are you spending)
plus a **terminal cost** (an extra penalty on the final predicted state,
often weighted more heavily, to encourage the plan to actually reach the
target by the end of the horizon rather than just "eventually"). The
specific weights and structure you choose are the graded formulation
exercise -- this primer is not telling you what to pick.

## Constraints

Anything the real system can't violate: actuator saturation
(`|tau| <= tau_max`), and optionally state limits (e.g. angular rate
bounds) if your plant/actuators need them. Constraints are added directly
to the optimization problem, not enforced after the fact by clipping --
clipping after solving can produce an inconsistent/infeasible plan.

## Why a general NLP solver (IPOPT), not a hand-rolled one

Once you discretize nonlinear dynamics via multiple shooting, the
resulting optimization problem is a nonlinear program (NLP): the
equality constraints (dynamics) are nonlinear in the decision variables.
CasADi's `Opti()` interface lets you describe decision variables,
constraints, and a cost function symbolically, then hands the whole
problem to a solver -- IPOPT (interior-point solver) is a mature, general
NLP solver well-suited to this. Writing a correct, robust NLP solver from
scratch is a significant undertaking on its own and is explicitly out of
scope here -- the formulation (what problem you're posing) is the
exercise, not the numerical solve.

## Real-time considerations

Solving an NLP takes measurable time -- sometimes much longer than a
single sim timestep. A controller that "works" in an offline test but
takes 200ms to solve while your control loop expects a new command every
20ms is not a working real-time controller. Part of your Week 2/3
writeup is measuring your actual solve time and justifying (or fixing)
your chosen control-loop frequency in light of it.

## Where to read more

- CasADi's own `Opti()` documentation and examples: https://web.casadi.org/
- Any standard MPC textbook/course material your team already uses is
  fair game for green-zone background reading -- this primer intentionally
  stays generic and pendulum-agnostic.
