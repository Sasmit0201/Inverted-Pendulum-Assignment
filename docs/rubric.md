# Grading rubric

Mirrors README.md Sec 8. This file exists to give a slightly more detailed
breakdown of what each weighted component checks -- the weights themselves
are the source of truth in README.md, keep them in sync if either changes.

| Component | Weight |
|---|---|
| `evaluate.py` pass/fail per exercise | 40% |
| Derivation + checkpoint defense | 35% |
| Live retune under perturbed plant | 15% |
| Code quality / ROS2 idiom | 10% |

## `evaluate.py` pass/fail per exercise (40%)

Objective, threshold-based grading against `config/eval_thresholds.yaml`.
No partial credit within this component per exercise -- either the
controller meets the thresholds (settling time, overshoot, steady-state
error, torque-saturation violations, and for swing-up, time-to-catch) or
it doesn't. See README.md Sec 8 (single) / Sec 5 (double) for the exact
metrics.

## Derivation + checkpoint defense (35%)

- Derivation artifact quality: is the Lagrangian/linearization/EOM work
  correct, complete, and actually submitted to `docs/derivations/`
  *before* the corresponding code is graded (per the AI-usage enforcement
  mechanism in README.md Sec 7)?
- Live defense at each checkpoint: can the trainee explain their own
  derivation from memory, without notes? This is what separates "the
  code works" from "the trainee understands why it's stable."

## Live retune under perturbed plant (15%)

Each checkpoint ends with a live retune against a plant configuration
(mass, latency, noise, etc.) the trainee has not seen before. This is the
part a chat transcript cannot pre-solve. Graded on whether the trainee
can diagnose what changed and adjust gains/weights/horizon appropriately
within the checkpoint's time box.

## Code quality / ROS2 idiom (10%)

Standard software-engineering bar: correct use of ROS2 node/pub/sub
patterns, no busy-waiting where a callback/timer is appropriate, params
loaded from `config/*.yaml` rather than hardcoded, reasonable naming and
structure. Not a style nitpick exercise -- this is the smallest-weight
component intentionally.
