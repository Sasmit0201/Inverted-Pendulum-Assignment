"""PID/PD balance + energy-shaping swing-up controller. TODO (trainee).

See README.md Sec 4.2 for the full exercise spec:
  - Exercise A (interface): confirm nonzero torque moves the pendulum
    before writing any control law.
  - Exercise B (balance): PID/PD near upright. This is the easy case --
    don't overbuild it.
  - Exercise C (swing-up): energy shaping from the downward equilibrium,
    handing off to the balance law near upright. A large-gain PID that
    "happens" to swing up does NOT satisfy this exercise -- your writeup
    must show the energy argument.

Checkpoint 1: defend the swing-up energy argument from memory, then
retune live against a perturbed config/params.yaml you haven't seen.
"""
import rclpy
from rclpy.node import Node

from custom_msgs.msg import PendulumState, TorqueCmd


class PIDController(Node):
    def __init__(self):
        super().__init__('controller_pid')

        # TODO: declare/load kp, ki, kd, tau_max, upright_threshold_rad,
        # and swingup_gain from config/params.yaml (see the
        # controller_pid section of that file for the expected keys).

        self.state_sub = self.create_subscription(
            PendulumState, '/single_inverted/state', self.state_callback, 10)
        self.torque_pub = self.create_publisher(
            TorqueCmd, '/single_inverted/torque_cmd', 10)

    def state_callback(self, msg: PendulumState):
        """TODO (trainee).

        Mode switch (wrap-aware -- theta near +/-pi is the same physical
        point as theta near -pi/+pi):
            if |theta| < upright_threshold_rad -> balance law (PID/PD)
            else                                -> energy-shaping swing-up law

        Publish a TorqueCmd on /single_inverted/torque_cmd, respecting
        tau_max from config/params.yaml (the sim will also saturate, but
        your controller should not rely on that).
        """
        raise NotImplementedError(
            'controller_pid.py is a Week 1 trainee deliverable -- see README.md Sec 4.2')


def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
