# Derivation submissions

Drop your hand-derived work here, **timestamped**, before the
corresponding code is allowed to be graded (see README.md Sec 7 -- this is
the AI usage enforcement mechanism, not busywork).

Accepted formats: a clear photo/scan of paper work, or a typed doc/PDF.
Either way, the file's timestamp (or an explicit date in the filename) is
what matters -- it must predate the working code, not follow it.

Suggested naming (adjust as needed, just keep it identifiable):

```
docs/derivations/
├── week1_swingup_energy_argument_2026-07-02.pdf
├── week2_mpc_single_pendulum_cost_constraints_2026-07-09.pdf
└── week3_mpc_double_pendulum_cost_constraints_2026-07-16.pdf
```

Only two kinds of derivation/reasoning work are graded deliverables in
this track (LQR and linearization are not part of this assignment --
PID and MPC are the only two control techniques used, see README.md):

| Derivation | Week | README section |
|---|---|---|
| Swing-up energy argument (single pendulum) | 1 | Sec 4.2 |
| MPC cost/constraint formulation (single pendulum) | 2 | Sec 4.3 |
| MPC cost/constraint formulation (double pendulum) | 3 | Sec 5.2 |

The double-pendulum equations of motion are **given** to you (see
`double_inverted/docs/eom_derivation.md`) -- translating them into
`dynamics.py` is a coding exercise, not a derivation, and does not need a
submission here.

