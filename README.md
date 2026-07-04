# Inverted Pendulum — Control on ROS2

Implementation of PID and MPC on a simulated single and double inverted pendulum, on ROS2, with live visualization in Foxglove Studio.

This document is both your assignment brief and the build spec for this repo. Sections marked **GIVEN** are provided/scaffolded for you. Sections marked **TODO** are your deliverable. Read the whole thing before writing code — the derivation requirements in each week gate whether your code is even allowed to be graded.

---

## 0. Why this exists

You'll use AI tools while working through this — that's expected and fine, see §7. What isn't fine is a controller you can't explain. Every week ends in a live checkpoint where you defend your derivation from memory and then retune your controller live against a plant configuration you haven't seen before. A chat transcript can't do that part for you. The exercises are designed so the code is the easy 40%; understanding why it's stable is the other 60%.

---

## 1. Prerequisites

- ROS2 (Humble or Jazzy). If you're on native Windows, set up WSL2 first — do this before Week 1 starts, not during it.
- Python 3.10+
- `pip install casadi numpy scipy sympy pyyaml`
- `sudo apt install ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-foxglove-bridge ros-$ROS_DISTRO-xacro`
- [Foxglove Studio](https://foxglove.dev/) (desktop app or web) — free.

---

## 2. Repo structure

```
inverted-pendulum-assignment/
├── custom_msgs/msg/
│   ├── PendulumState.msg
│   ├── PendulumState2.msg
│   ├── TorqueCmd.msg
│   └── TorqueCmd2.msg
├── single_inverted/
│   ├── config/{params.yaml, eval_thresholds.yaml}
│   ├── launch/single_inverted_pendulum.launch.py
│   ├── urdf/single_pendulum.urdf.xacro
│   ├── single_inverted/{sim_node.py, dynamics.py, controller_pid.py, controller_mpc.py}
│   ├── scripts/evaluate.py
│   └── test/
├── double_inverted/
│   ├── (mirrors single_inverted)
│   ├── docs/eom_derivation.md
│   ├── urdf/double_pendulum.urdf.xacro
│   └── double_inverted/{sim_node.py, dynamics.py, controller_pid.py, controller_mpc.py}
├── tools/mechanics_check.py
├── foxglove/layout_notes.md
├── docs/{derivations/, ai_log.md, rubric.md, mpc_primer.md, example_workflow/}
└── illustrations/
```

---

## 3. Message definitions (GIVEN, exact)

`custom_msgs/msg/PendulumState.msg`
```
std_msgs/Header header
float64 theta       # rad, 0 = upright, measured CCW positive
float64 theta_dot    # rad/s
```

`custom_msgs/msg/TorqueCmd.msg`
```
std_msgs/Header header
float64 torque       # N*m, applied at the pivot
```

`custom_msgs/msg/PendulumState2.msg`
```
std_msgs/Header header
float64 theta1        # rad, base joint, 0 = upright
float64 theta1_dot     # rad/s
float64 theta2        # rad, elbow joint, relative to link 1
float64 theta2_dot     # rad/s
```

`custom_msgs/msg/TorqueCmd2.msg`
```
std_msgs/Header header
float64 torque1       # N*m at joint 1
float64 torque2       # N*m at joint 2
```

## Topics (GIVEN, exact)

| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| `/single_inverted/state` | `custom_msgs/PendulumState` | `sim_node` | controller node |
| `/single_inverted/torque_cmd` | `custom_msgs/TorqueCmd` | controller node | `sim_node` |
| `/single_inverted/joint_states` | `sensor_msgs/JointState` | `sim_node` | `robot_state_publisher` |
| `/double_inverted/state` | `custom_msgs/PendulumState2` | `sim_node` | controller node |
| `/double_inverted/torque_cmd` | `custom_msgs/TorqueCmd2` | controller node | `sim_node` |
| `/double_inverted/joint_states` | `sensor_msgs/JointState` | `sim_node` | `robot_state_publisher` |

Only one controller node runs at a time per pendulum; which one launches is a launch argument (§6).

---

## 4. Single inverted pendulum

### 4.1 Plant (GIVEN)

Simple pendulum, point mass `m` at distance `l` from a fixed pivot, moment of inertia `I` about the pivot, viscous damping `b`, driven by torque `tau` at the pivot. `theta = 0` is upright (unstable equilibrium), `theta = π` is hanging down (stable equilibrium).

$$I\ddot\theta = m g l \sin\theta - b\dot\theta + \tau$$

`single_inverted/dynamics.py` — **GIVEN:**
```python
def theta_ddot(theta: float, theta_dot: float, tau: float, params: dict) -> float:
    """Nonlinear EOM. params has keys: m, l, I, b, g. GIVEN — do not modify."""
    ...
```

`single_inverted/sim_node.py` — **GIVEN.** RK4 integration of `theta_ddot` at a fixed sim rate (default 200 Hz, configurable), publishes `/single_inverted/state` and `/single_inverted/joint_states`, subscribes `/single_inverted/torque_cmd`, applies torque saturation from `params.yaml`, and supports configurable initial conditions (fixed near-upright, fixed downward, or randomized within a range — set via `config/params.yaml`).

### 4.2 Week 1 — Interface + PID

**Exercise A — Interface.** `controller_pid.py`: subscribe to `/single_inverted/state`, publish to `/single_inverted/torque_cmd`. Confirm nonzero torque moves the pendulum before writing any control law.

**Exercise B — Balance.** IC near upright (small `theta0`, small `theta_dot0`, see `params.yaml`). Write a PID (or PD) balance law. This is the easy case — don't overbuild it.

**Exercise C — Swing-up.** IC at the downward stable equilibrium (`theta0 ≈ π`). Required approach: **energy shaping**, not a large-gain PID that happens to work. Pump energy toward the upright equilibrium's energy level, then hand off to your balance controller once near upright (a simple angle-threshold switch is fine — a smooth blend is a nice-to-have, not required).

```python
# controller_pid.py — TODO (trainee)
class PIDController(Node):
    def __init__(self):
        # subscribe /single_inverted/state, publish /single_inverted/torque_cmd
        # load Kp, Kd, Ki and swingup params from config/params.yaml
        ...

    def state_callback(self, msg: PendulumState):
        # mode switch: if |theta| < upright_threshold -> balance law
        #              else -> energy-shaping swing-up law
        ...
```

**Checkpoint 1:** defend the energy argument for swing-up from memory (no notes), then retune live against a perturbed `params.yaml` you haven't seen (heavier mass, added actuation latency).

### 4.3 Week 2 — MPC

You have not implemented MPC before. Read `docs/mpc_primer.md` first — it's given background reading on multiple shooting, receding horizon control, discretization, and why an NLP solver is needed. It is not the formulation exercise itself and contains no pendulum-specific content.

Once you've read the primer: formulate on paper first — cost function, constraints, discretization scheme — before touching CasADi.

**Required formulation:**
- State `x = [theta, theta_dot]`, control `u = tau`.
- Discretize the *nonlinear* EOM (reuse `dynamics.theta_ddot`) via RK4 at your chosen `dt`.
- Multiple-shooting NLP: minimize $\sum_{k=0}^{N-1} (x_k - x_{ref})^T Q (x_k - x_{ref}) + u_k^T R u_k + \text{terminal cost}$, subject to the shooting/dynamics constraints and `|tau| <= tau_max`.
- Solve with CasADi `Opti()` + IPOPT. Do not hand-roll the NLP solver — the formulation is the exercise, not the solve.

```python
# controller_mpc.py — TODO (trainee)
class MPCController(Node):
    def __init__(self):
        # build the CasADi Opti() problem once at init: decision vars, dynamics
        # constraints (multiple shooting, RK4), cost, bounds. N and dt are
        # config/params.yaml values.
        ...

    def state_callback(self, msg: PendulumState):
        # set x0 (warm-start from previous solution), solve, publish tau_0
        ...
```

**Required in your writeup:** measured NLP solve time, and a justification for your chosen control-loop frequency given that solve time. A controller that "works" with a 200 ms solve and a 20 ms sim timestep is not actually working — say why your chosen rate is or isn't a problem.

**Checkpoint 2:** defend your MPC formulation (why this cost weighting, why these constraints), report and justify your measured solve time vs. control-loop frequency, then live-retune (cost weights, horizon) against a perturbed plant.

---

## 5. Double inverted pendulum

### 5.1 Setup (base requirement: fully actuated at both joints)

Now that you've built PID and MPC for the simpler single-pendulum system, apply the same two control techniques to the double pendulum — same logic, more states and a coupled, more complex plant.

**The equations of motion are given to you this time** — see [`double_inverted/docs/eom_derivation.md`](double_inverted/docs/eom_derivation.md) for the fully worked 2-link Lagrangian derivation (fully actuated, both joints). Your job is **translation, not derivation**: implement the given closed-form equations correctly in `dynamics.py`. This is still real work — matching state/sign conventions, structuring the mass-matrix/Coriolis/gravity terms, inverting a 2x2 matrix, avoiding transcription errors — just not a from-scratch derivation.

`sim_node.py` is scaffolded — message wiring, integration loop, and publishing are given; the EOM call is the TODO.

```python
# double_inverted/dynamics.py — TODO (trainee)
def theta_ddot(state: np.ndarray, tau: np.ndarray, params: dict) -> np.ndarray:
    """
    state = [theta1, theta1_dot, theta2, theta2_dot]
    tau = [tau1, tau2]
    Implement the equations given in docs/eom_derivation.md. Return
    [theta1_ddot, theta2_ddot].
    """
    raise NotImplementedError
```

### 5.2 Week 3, part A — PID (per-joint, independent)

Same spirit as Week 1, decentralized across two joints: treat `theta1` and `theta2` as two independent single-pendulum-style error signals — `tau1` from a PID/PD on `theta1`, `tau2` from a PID/PD on `theta2`. This deliberately ignores the cross-joint coupling in the real dynamics; that's expected — it's a fast sanity check that your translated EOM and sim are behaving correctly, and a simple baseline before MPC.

**Exercise A — Balance.** IC near upright, independent PID/PD per joint.

**Exercise B — Swing-up.** IC at the downward equilibrium, independent energy-shaping per joint (apply the same idea from Week 1's swing-up, separately, to each joint). Perfect coordination between joints is **not** required or expected — this is intentionally an approximate baseline, not the optimal solution. State whether you switch modes per joint independently or on a combined norm of both angles — either is fine, just justify the choice.

### 5.3 Week 3, part B — MPC (scaffolded)

`double_inverted/controller_mpc.py` ships with the CasADi shooting skeleton (decision variables, dynamics-constraint wiring against your `dynamics.theta_ddot`) already built. Your TODO is the cost function and constraint bounds — same formulation exercise as §4.3, extended to 4 states / 2 controls.

**Checkpoint 3 (final):** walk through your EOM translation (correctness, not derivation — you're explaining how you turned the given equations into code and how you validated it), defend your MPC formulation, then live-retune under a perturbed, noisy, latency-added plant (20–25 min time box, since MPC retuning means adjusting cost weights/horizon rather than just gains).

### 5.4 Stretch goal — acrobot swing-up (optional)

Only joint 2 (elbow) actuated, joint 1 passive. This is a genuinely harder, underactuated problem — not required, opens up once you're ahead of pace on the base double-pendulum requirement. Ask before starting it; it changes the controllability picture and needs a different control strategy (partial feedback linearization or energy-based swing-up for underactuated systems), not just a bigger version of what you already built.

---

## 6. How to build and run

```bash
# from your ROS2 workspace src/ folder
git clone <this repo>
cd ..
colcon build --symlink-install
source install/setup.bash
```

Launch (single pendulum, PID controller, Foxglove enabled):
```bash
ros2 launch single_inverted single_inverted_pendulum.launch.py controller:=pid ic_mode:=downward enable_foxglove:=true
```

Launch args:
- `controller:=pid|mpc`
- `ic_mode:=upright|downward|random`
- `enable_foxglove:=true|false`

Double pendulum follows the same pattern via `double_inverted double_inverted_pendulum.launch.py`.

Run the objective grader against a running sim:
```bash
ros2 run single_inverted evaluate.py --exercise swingup --duration 15
ros2 run double_inverted evaluate.py --exercise mpc_balance --duration 15
```

### Foxglove setup

1. Launch with `enable_foxglove:=true` (this starts `foxglove_bridge` on the default port alongside `robot_state_publisher`, which is fed your `urdf/*.urdf.xacro`).
2. In Foxglove Studio, connect to `ws://localhost:8765`.
3. Add a **3D** panel — it will pick up the robot description and TF automatically once the sim is publishing `joint_states`.
4. Add **Plot** panels for `/single_inverted/state.theta`, `.theta_dot`, and `/single_inverted/torque_cmd.torque` — Foxglove plots any numeric field by message path directly, no extra topics needed.
5. Save your panel layout (`Layout → Export`) into `foxglove/` if you want to reuse it across sessions.

---

## 7. Using AI tools — scoped, not banned

You'll use these tools on the actual car. Learn to use them for the right things now.

**Green — use freely:** ROS2/launch/message boilerplate, CasADi API syntax, reading tracebacks, refactors and style, reading `docs/mpc_primer.md`-style background material.

**Yellow — use to check your own work, log it in `docs/ai_log.md` (one line: what you asked, why):** verifying your translated double-pendulum EOM against a symbolic reference (`tools/mechanics_check.py` — see below), sanity-checking an MPC cost/constraint choice, debugging why a swing-up controller isn't pumping enough energy.

**Red — do not offload:** the swing-up energy argument (Week 1), the reasoning behind MPC cost/constraint design, and the checkpoint defense itself.

`tools/mechanics_check.py` is a sanctioned, general-purpose (not pendulum-specific) sympy toolkit for cross-checking mechanics work — it mechanically applies Euler-Lagrange to a Lagrangian you build, or numerically compares your function against a symbolic reference. It's yellow zone: useful for catching algebra/sign errors, but it doesn't tell you whether your physical model (what `T`/`V`/generalized forces to use) is correct, and it doesn't do the translation-into-code step for you. See `docs/example_workflow/` for a full non-pendulum worked example of the paper → symbolic-check → code pipeline before you apply it to the double-pendulum EOM translation.

Your derivation submission is checked *before* `evaluate.py` is allowed to run against your controller — this is what stops "get it working with AI help, write the derivation after." The live retune at each checkpoint, against a plant you haven't seen, is the real check: it can't have been pre-solved by anything you asked beforehand.

---

## 8. Grading

| Component | Weight |
|---|---|
| `evaluate.py` pass/fail per exercise | 40% |
| Derivation + checkpoint defense | 35% |
| Live retune under perturbed plant | 15% |
| Code quality / ROS2 idiom | 10% |

Full rubric detail in `docs/rubric.md`.

---

## 9. Build checklist (repo scaffolding reference)

For whoever/whatever is scaffolding this repo — build in this order, since messages and given-files are fully specified (zero ambiguity, should one-shot) while stub files are signatures + docstrings only (do not implement the control laws or dynamics — those are the trainee deliverable):

1. `custom_msgs/msg/*.msg` — exact content in §3.
2. `single_inverted`: `dynamics.py` (nonlinear `theta_ddot` given, no linearization), `sim_node.py` (full RK4 sim + pub/sub per §4.1), `urdf/single_pendulum.urdf.xacro` (single revolute joint, matches `theta` convention), `launch/single_inverted_pendulum.launch.py` (args: `controller`, `ic_mode`, `enable_foxglove`; conditionally launches `robot_state_publisher` + `foxglove_bridge`), `config/params.yaml` (physical params, noise, torque limits, IC ranges, gain placeholders).
3. `single_inverted` controller stubs — `controller_pid.py`, `controller_mpc.py` — class + method signatures and docstrings per §4.2–4.3, `raise NotImplementedError` or `TODO` bodies only.
4. `scripts/evaluate.py` — full implementation: subscribes state + torque_cmd, computes settling time / overshoot / steady-state error / saturation violations against `config/eval_thresholds.yaml`, prints pass/fail.
5. `double_inverted` — mirror steps 2–4, but with `dynamics.py::theta_ddot` a translation stub (equations GIVEN in `docs/eom_derivation.md`, per §5.1), a `controller_pid.py` stub (independent per-joint PID + swing-up, per §5.2), and `controller_mpc.py` shipping the shooting/constraint-wiring skeleton with cost/bounds as the stub (per §5.3).
6. `double_inverted/docs/eom_derivation.md` — GIVEN, fully worked Lagrangian derivation for the fully-actuated 2-link double pendulum.
7. `tools/mechanics_check.py` — GIVEN, general-purpose sympy Euler-Lagrange/comparison/linearization utility (not pendulum-specific).
8. `docs/mpc_primer.md` — GIVEN background reading on MPC concepts (receding horizon, multiple shooting, discretization, solver choice) — no pendulum-specific formulation content.
9. `docs/example_workflow/{README.md, example_dynamics.py}` — GIVEN, fully worked non-pendulum (mass-spring-damper) example of the paper → symbolic-check → code pipeline.
10. `foxglove/layout_notes.md` — panel setup instructions per §6, no exact byte-for-byte layout JSON required (Foxglove layouts are saved/exported from the app itself).
11. `docs/` scaffolding — `derivations/README.md` (required-derivations table per §8), `ai_log.md` template, `rubric.md` mirroring §8.

Do not implement any TODO-marked function body beyond `raise NotImplementedError` or an explanatory comment — those are graded trainee deliverables, not scaffolding.