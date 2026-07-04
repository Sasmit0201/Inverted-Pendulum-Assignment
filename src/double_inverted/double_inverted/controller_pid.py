"""Independent per-joint PID/PD balance + energy-shaping swing-up
controller for the double (2-link) pendulum. TODO (trainee).

See README.md Sec 5 for the full exercise spec. This mirrors
single_inverted/controller_pid.py's structure, decentralized across two
joints:

  - Exercise A (interface): confirm nonzero torque on each joint moves
    that joint before writing any control law.
  - Exercise B (balance): independent PID/PD per joint, IC near upright.
    Treat theta1 and theta2 as two separate single-pendulum-style error
    signals -- tau1 from a PID on theta1, tau2 from a PID on theta2. This
    deliberately ignores the cross-joint coupling in the real dynamics;
    that's expected and fine, it's the simple baseline before MPC handles
    the coupling properly.
  - Exercise C (swing-up): independent energy-shaping per joint, IC at
    the downward equilibrium. Apply the same per-joint energy-shaping
    idea you used in single_inverted/controller_pid.py, independently, to
    each joint. Perfect coordination between the two joints is NOT
    required or expected here -- this is intentionally an approximate
    baseline, not the optimal solution (MPC is where the coupled swing-up
    problem gets solved properly). You may switch modes per joint
    independently (theta1 and theta2 threshold-switch separately) or on
    a combined norm of both angles -- either is an acceptable design
    choice, just say which you picked and why in your writeup.
"""
import rclpy
from rclpy.node import Node

from custom_msgs.msg import PendulumState2, TorqueCmd2


class PIDController(Node):
    def __init__(self):
        super().__init__('controller_pid')

        # TODO: declare/load kp1, ki1, kd1, kp2, ki2, kd2 (or shared gains
        # if you choose that design), tau1_max, tau2_max,
        # upright_threshold_rad, and swingup_gain(s) from
        # config/params.yaml (controller_pid section).

        self.state_sub = self.create_subscription(
            PendulumState2, '/double_inverted/state', self.state_callback, 10)
        self.torque_pub = self.create_publisher(
            TorqueCmd2, '/double_inverted/torque_cmd', 10)

    def state_callback(self, msg: PendulumState2):
        """TODO (trainee).

        For each joint independently:
            if |angle| < upright_threshold_rad -> balance law (PID/PD)
            else                                 -> energy-shaping swing-up law

        Publish a TorqueCmd2 on /double_inverted/torque_cmd, respecting
        tau1_max/tau2_max from config/params.yaml.
        """
        raise NotImplementedError(
            'double_inverted/controller_pid.py is a trainee deliverable -- '
            'see README.md Sec 5')


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
