# Foxglove Studio panel setup

Companion notes to README.md Sec 6. Foxglove layouts are saved/exported
from the app itself (no committed layout JSON here) -- this file just
documents how to reconstruct a useful layout quickly.

## Connect

1. Launch your package with `enable_foxglove:=true` (starts
   `foxglove_bridge` on port 8765 alongside `robot_state_publisher`).
2. Open Foxglove Studio (desktop or [web](https://studio.foxglove.dev/)).
3. **Open connection** -> **Foxglove WebSocket** -> `ws://localhost:8765`.

## Recommended panels

**3D panel**
- Add a 3D panel. It auto-discovers the robot description (URDF) and TF
  once `sim_node` is publishing `joint_states` and `robot_state_publisher`
  is running -- no manual topic wiring needed.

**Plot panels** (one per signal, or combine related ones on one plot)
- Single pendulum:
  - `/single_inverted/state.theta`
  - `/single_inverted/state.theta_dot`
  - `/single_inverted/torque_cmd.torque`
- Double pendulum:
  - `/double_inverted/state.theta1`, `.theta1_dot`
  - `/double_inverted/state.theta2`, `.theta2_dot`
  - `/double_inverted/torque_cmd.torque1`, `.torque2`

Foxglove can plot any numeric field directly by message path -- no extra
topics or bridging needed for these.

**Raw Messages panel** (optional)
- Useful for eyeballing `/…/state` and `/…/torque_cmd` values while
  debugging a controller before you trust the Plot panel.

## Saving your layout

Once you've arranged panels the way you like: **Layout -> Export layout**,
and drop the exported JSON into this `foxglove/` folder (e.g.
`foxglove/single_inverted_layout.json`) so you can re-import it in future
sessions via **Layout -> Import layout**.
