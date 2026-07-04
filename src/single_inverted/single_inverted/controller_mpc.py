"""CasADi multiple-shooting MPC controller. TODO (trainee). See README.md Sec 4.4.

Formulate on paper first: cost function, constraints, discretization
scheme. Do not touch CasADi until that's written down.

Required formulation:
  - state x = [theta, theta_dot], control u = tau
  - discretize the NONLINEAR EOM (reuse dynamics.theta_ddot) via RK4 at dt
  - multiple shooting: minimize
        sum_k (x_k - x_ref)^T Q (x_k - x_ref) + u_k^T R u_k + terminal cost
    subject to the shooting/dynamics constraints and |tau| <= tau_max
  - solve with casadi.Opti() + IPOPT. Do not hand-roll the NLP solver --
    the formulation is the exercise, not the solve.

Required in your writeup: measured NLP solve time, and a justification
for your chosen control-loop frequency given that solve time.
"""
import casadi as ca
import numpy as np
import rclpy
from rclpy.node import Node

from custom_msgs.msg import PendulumState, TorqueCmd

from single_inverted import dynamics  # noqa: F401  (reuse theta_ddot in your RK4 discretization)


class MPCController(Node):
    def __init__(self):
        super().__init__('controller_mpc')

        # TODO: load N (horizon_steps), dt, Q, R, tau_max from
        # config/params.yaml (controller_mpc section). Build the
        # casadi.Opti() problem ONCE here:
        #   - decision variables: X (n_x x N+1), U (n_u x N)
        #   - dynamics constraints via RK4 of the nonlinear EOM
        #     (multiple shooting)
        #   - cost: tracking + control effort + terminal cost
        #   - bounds: |tau| <= tau_max
        # Store the Opti object, solver options ('ipopt'), and a
        # warm-start cache for the previous solution.

        self.state_sub = self.create_subscription(
            PendulumState, '/single_inverted/state', self.state_callback, 10)
        self.torque_pub = self.create_publisher(
            TorqueCmd, '/single_inverted/torque_cmd', 10)

    def state_callback(self, msg: PendulumState):
        """TODO (trainee).

        Set x0 from msg (warm-start decision variables from the previous
        solution), solve the NLP, publish tau_0 (the first control in the
        solved sequence) as a TorqueCmd. Measure and log solve time --
        you need this number for your writeup.
        """
        raise NotImplementedError(
            'controller_mpc.py is a Week 3 trainee deliverable -- see README.md Sec 4.4')


def main(args=None):
    rclpy.init(args=args)
    node = MPCController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
