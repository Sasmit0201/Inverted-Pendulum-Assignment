# AI usage log

One line per **yellow-zone** use (see README.md Sec 7 for the
green/yellow/red policy). Format:

```
YYYY-MM-DD | what you asked | why
```

Green-zone usage (boilerplate, syntax lookups, traceback reading, style)
does not need to be logged. Red-zone usage (first-pass derivations, MPC
cost/constraint reasoning, checkpoint defense) must not happen at all.

---

<!-- Example entry -- delete once you add your own:
2026-07-02 | Verified my by-hand linearization of theta_ddot against sympy | wanted a second check before submitting the derivation
-->
