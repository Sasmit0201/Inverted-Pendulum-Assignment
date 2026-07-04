# Double pendulum equations of motion (GIVEN)

This document contains the fully worked Lagrangian derivation of the
equations of motion for the fully-actuated 2-link double pendulum used in
`double_inverted/`. **This derivation is given to you** -- unlike the
single pendulum in Week 1/2, you are not deriving these equations
yourself. Your job is to **translate** the closed-form equations below
into a correct `theta_ddot()` implementation in `dynamics.py`.

That translation is still real work: you have to match the state/sign
conventions exactly, structure the mass-matrix/Coriolis/gravity terms
correctly, invert a 2x2 matrix, and avoid transcription errors. Treat it
the way you'd treat implementing a paper's equations in a research
codebase -- get it right, don't just eyeball it.

---

## 1. Setup and generalized coordinates

Two rigid links, each with its own mass, length, and moment of inertia
about its own center of mass:

- Link 1: mass `m1`, full length `l1`, center-of-mass distance from joint
  1 `lc1`, moment of inertia about its own COM `I1`, joint damping `b1`.
- Link 2: mass `m2`, full length `l2`, center-of-mass distance from joint
  2 `lc2`, moment of inertia about its own COM `I2`, joint damping `b2`.
- `g`: gravitational acceleration.

Generalized coordinates (matching `custom_msgs/PendulumState2.msg` and
`dynamics.py`'s state convention):

- `theta1`: angle of link 1 from the upward vertical (absolute, 0 = upright).
- `theta2`: angle of link 2 **relative to link 1** (0 = link 2 is a
  straight extension of link 1, both pointing the same way).

The absolute angle of link 2 is therefore `theta1 + theta2`.

## 2. Kinematics

Center-of-mass positions (y measured upward from the base joint):

```
x1 = lc1 * sin(theta1)
y1 = lc1 * cos(theta1)

x2 = l1 * sin(theta1) + lc2 * sin(theta1 + theta2)
y2 = l1 * cos(theta1) + lc2 * cos(theta1 + theta2)
```

Differentiate to get velocities, then form `v1^2 = x1_dot^2 + y1_dot^2`
and `v2^2 = x2_dot^2 + y2_dot^2`. After using the identity
`cos(A)cos(B) + sin(A)sin(B) = cos(A - B)`:

```
v1^2 = lc1^2 * theta1_dot^2

v2^2 = l1^2 * theta1_dot^2
     + lc2^2 * (theta1_dot + theta2_dot)^2
     + 2 * l1 * lc2 * theta1_dot * (theta1_dot + theta2_dot) * cos(theta2)
```

## 3. Kinetic and potential energy

```
T = 0.5*m1*v1^2 + 0.5*I1*theta1_dot^2
  + 0.5*m2*v2^2 + 0.5*I2*(theta1_dot + theta2_dot)^2

V = m1*g*lc1*cos(theta1) + m2*g*(l1*cos(theta1) + lc2*cos(theta1 + theta2))
```

`L = T - V`.

## 4. Euler-Lagrange -> equations of motion

Applying `d/dt(dL/dtheta_i_dot) - dL/dtheta_i = generalized_force_i` for
`i = 1, 2` and adding joint torque + linear viscous damping as the
generalized forces gives the standard manipulator-equation form:

```
M(theta) @ theta_ddot + C(theta, theta_dot) @ theta_dot + G(theta) + b_vec = tau
```

where `theta = [theta1, theta2]`, `theta_dot = [theta1_dot, theta2_dot]`,
`tau = [tau1, tau2]`, `b_vec = [b1*theta1_dot, b2*theta2_dot]`, and:

**Mass matrix** (symmetric):
```
alpha = I1 + I2 + m1*lc1**2 + m2*(l1**2 + lc2**2)
beta  = m2*l1*lc2
delta = I2 + m2*lc2**2

M11 = alpha + 2*beta*cos(theta2)
M12 = delta + beta*cos(theta2)
M21 = M12
M22 = delta
```

**Coriolis/centrifugal matrix:**
```
C11 = -beta*sin(theta2)*theta2_dot
C12 = -beta*sin(theta2)*(theta1_dot + theta2_dot)
C21 =  beta*sin(theta2)*theta1_dot
C22 = 0
```

**Gravity vector:**
```
G1 = -(m1*lc1 + m2*l1)*g*sin(theta1) - m2*lc2*g*sin(theta1 + theta2)
G2 = -m2*lc2*g*sin(theta1 + theta2)
```

## 5. Solving for the accelerations

`theta_ddot = M^-1 @ (tau - C @ theta_dot - G - b_vec)`

For a 2x2 matrix, the inverse has a closed form -- no need for a general
linear-algebra solve:

```
det(M) = M11*M22 - M12*M21

M_inv = (1/det(M)) * [[ M22, -M12],
                       [-M21,  M11]]
```

## 6. What's left for you to do

- Implement `theta_ddot(state, tau, params)` in `dynamics.py` using the
  equations above. `params` needs `m1, l1, lc1, I1, b1, m2, l2, lc2, I2,
  b2, g` (add `lc1`/`lc2` to `config/params.yaml` if not already present
  -- they default sensibly to `l1/2`, `l2/2` for a uniform rod, but you
  should treat that as a modeling choice worth a one-line note in your
  writeup, not something to silently assume).
- Double-check units and signs against the kinematics above before
  trusting any control result built on top of this function -- a sign
  error here will silently break every controller downstream.
- You're welcome (yellow zone, log it in `docs/ai_log.md`) to cross-check
  your `theta_ddot()` implementation against a symbolic version built with
  `tools/mechanics_check.py` (feed it the same `L = T - V` above and
  compare `euler_lagrange_eom(...)` output against what you coded). This
  does not remove the translation exercise -- it's a sanity check on your
  own transcription, not a replacement for doing it carefully.
