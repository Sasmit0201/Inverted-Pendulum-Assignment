"""CasADi multiple-shooting MPC controller, double pendulum. SCAFFOLDED.

The shooting skeleton (decision variables, RK4 dynamics constraints wired
against your derived EOM) is GIVEN below in `_build_opti` /
`_dynamics_ca`. Your TODO is:
  1. Port your double_inverted/dynamics.py::theta_ddot equations into
     CasADi symbols in `_dynamics_ca` (Opti() needs a differentiable
     symbolic expression, so you cannot call the numeric
     dynamics.theta_ddot directly here).
  2. The cost function and bounds (tau limits, optionally angle-rate
     limits) in `_build_opti`.
  3. `state_callback`: set x0, warm-start, solve, publish tau_0.

Same formulation exercise as single_inverted/controller_mpc.py, extended
to 4 states / 2 controls. See README.md Sec 5.2.
"""
import casadi as ca
import numpy as np
import rclpy
from rclpy.node import Node

from custom_msgs.msg import PendulumState2, TorqueCmd2

N_STATES = 4
N_CONTROLS = 2


class MPCController(Node):
    def __init__(self):
        super().__init__('controller_mpc')

        # TODO: load these from config/params.yaml (controller_mpc section)
        self.N = 20          # horizon steps
        self.dt = 0.02
        self.tau_max = np.array([5.0, 5.0])  # placeholder -- TODO tune/load

        self._build_opti()

        self.state_sub = self.create_subscription(
            PendulumState2, '/double_inverted/state', self.state_callback, 10)
        self.torque_pub = self.create_publisher(
            TorqueCmd2, '/double_inverted/torque_cmd', 10)

    def _dynamics_ca(self, x, u):
        """TODO (trainee): CasADi-symbolic version of your derived EOM.

        Port the same equations you wrote in
        double_inverted/dynamics.py::theta_ddot here using
        casadi.sin/cos/etc. instead of numpy, since this must be
        differentiable/symbolic to be used inside Opti(). Return the full
        4-state derivative [theta1_dot, theta1_ddot, theta2_dot, theta2_ddot]
        as a CasADi expression (x[1], your_theta1_ddot_expr, x[3],
        your_theta2_ddot_expr).
        """
        raise NotImplementedError(
            'controller_mpc.py::_dynamics_ca is a trainee deliverable -- port your '
            'double_inverted/dynamics.py EOM into CasADi symbols here')

    def _rk4_ca(self, x, u):
        """GIVEN: RK4 discretization, given _dynamics_ca is implemented."""
        k1 = self._dynamics_ca(x, u)
        k2 = self._dynamics_ca(x + self.dt / 2 * k1, u)
        k3 = self._dynamics_ca(x + self.dt / 2 * k2, u)
        k4 = self._dynamics_ca(x + self.dt * k3, u)
        return x + self.dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

    def _build_opti(self):
        """GIVEN: multiple-shooting skeleton (decision vars + dynamics
        constraints). Cost and bounds are your TODO -- fill in the marked
        section below, then set the solver.
        """
        self.opti = ca.Opti()
        self.X = self.opti.variable(N_STATES, self.N + 1)
        self.U = self.opti.variable(N_CONTROLS, self.N)
        self.x0_param = self.opti.parameter(N_STATES)

        self.opti.subject_to(self.X[:, 0] == self.x0_param)
        for k in range(self.N):
            x_next = self._rk4_ca(self.X[:, k], self.U[:, k])
            self.opti.subject_to(self.X[:, k + 1] == x_next)

        # --- TODO (trainee): cost function ---
        # Q = ca.diag([...])  # state weights, from config/params.yaml
        # R = ca.diag([...])  # control weights
        # Q_terminal = ca.diag([...])
        # cost = 0
        # for k in range(self.N):
        #     cost += ca.mtimes([self.X[:, k].T, Q, self.X[:, k]])
        #     cost += ca.mtimes([self.U[:, k].T, R, self.U[:, k]])
        # cost += ca.mtimes([self.X[:, self.N].T, Q_terminal, self.X[:, self.N]])
        # self.opti.minimize(cost)

        # --- TODO (trainee): bounds ---
        # for k in range(self.N):
        #     self.opti.subject_to(
        #         self.opti.bounded(-self.tau_max, self.U[:, k], self.tau_max))

        # --- TODO (trainee): solver ---
        # self.opti.solver('ipopt', {'print_time': False}, {'print_level': 0})

    def state_callback(self, msg: PendulumState2):
        """TODO (trainee).

        Set x0 from msg (warm-start decision variables from the previous
        solution), solve the NLP, publish [tau1_0, tau2_0] (the first
        control in the solved sequence) as a TorqueCmd2. Measure and log
        solve time -- you need this number for your writeup.
        """
        raise NotImplementedError(
            'controller_mpc.py::state_callback is a trainee deliverable -- '
            'see README.md Sec 5.2')


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
