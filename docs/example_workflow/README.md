# Worked example: paper -> symbolic check -> code (non-pendulum toy)

This walks through the full pipeline you'll use for the graded exercises
-- deriving physics on paper, optionally cross-checking it symbolically,
then translating it into working Python -- using a **different toy
system** (a 1D mass-spring-damper) so nothing here is directly reusable
in the graded pendulum exercises. The goal is to show you the *process*
and the *code shape*, not to hand you an answer.

System: a mass `m` on a horizontal track, connected to a wall by a spring
(stiffness `k`) and a damper (coefficient `b`), driven by an external
horizontal force `F`.

## Step 1 -- Kinetic and potential energy (by hand)

Generalized coordinate: `x` (displacement from the spring's natural
length).

```
T = 0.5 * m * x_dot**2
V = 0.5 * k * x**2
L = T - V
```

## Step 2 -- Euler-Lagrange (by hand)

```
d/dt(dL/dx_dot) - dL/dx = Q
```

where `Q` is the generalized force (external force minus damping):
`Q = F - b*x_dot`.

```
d/dt(m*x_dot) - (-k*x) = F - b*x_dot
m*x_ddot + k*x = F - b*x_dot
m*x_ddot = -k*x - b*x_dot + F
```

## Step 3 -- Optional symbolic cross-check

Using `tools/mechanics_check.py::euler_lagrange_eom`, you'd build the
same Lagrangian symbolically and confirm the mechanical
Euler-Lagrange step matches your by-hand algebra:

```python
import sympy as sp
from tools.mechanics_check import euler_lagrange_eom

t = sp.Symbol('t')
m, k = sp.symbols('m k')
x = sp.Function('x')(t)

L = sp.Rational(1, 2) * m * x.diff(t)**2 - sp.Rational(1, 2) * k * x**2
eom = euler_lagrange_eom(L, [x], t)
print(eom)  # -> [-k*x(t) - m*x(t).diff(t, 2)] (sign convention: this equals -Q)
```

This only checks the calculus bookkeeping (the derivative/algebra step),
not whether your `T`/`V` model the physics correctly, and it does not add
damping or external force for you -- those are generalized forces you add
by hand after this step, same as in Step 2.

## Step 4 -- Translate into a `dynamics.py`-style function

This is the code shape you'll use in `single_inverted/dynamics.py` and
`double_inverted/dynamics.py`: a plain function taking state, input, and
a `params` dict, returning the acceleration(s). See
`example_dynamics.py` in this folder for the fully worked version of this
step (safe to look at in full, since this toy system isn't graded).

## Step 5 -- Sanity-test via integration

Before trusting a `theta_ddot`/`x_ddot`-style function, integrate it
(e.g. with the same RK4 pattern used in `single_inverted/sim_node.py`)
from a known initial condition and check it against physical intuition:
does an undamped mass-spring system oscillate at the expected natural
frequency `sqrt(k/m)`? Does adding damping make it decay? These are cheap
checks that catch sign errors and transcription mistakes before you ever
touch a controller.

## Applying this to the graded exercises

- **Single pendulum:** the nonlinear EOM is already given to you in
  `single_inverted/dynamics.py` -- you don't need Steps 1-2 for it, but
  the *swing-up energy argument* you derive and defend in Week 1 follows
  the same "energy on paper first" spirit as Step 1 here.
- **Double pendulum:** the EOM is given to you in
  `double_inverted/docs/eom_derivation.md` (already through Steps 1-2 and
  written up as closed-form equations) -- your job is Step 4 (translate
  into `dynamics.py`) and, if you want the extra confidence, Step 3
  (symbolic cross-check against the given Lagrangian) and Step 5 (sanity
  test via integration) before wiring up any controller against it.
- **MPC (both pendulums):** the "paper first" step is your cost/
  constraint formulation, not a new derivation -- see
  `docs/mpc_primer.md` for the background you need before attempting
  that formulation.
