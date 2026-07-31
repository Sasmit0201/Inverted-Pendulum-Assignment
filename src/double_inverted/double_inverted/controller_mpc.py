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
"""CasADi multiple-shooting MPC controller, double pendulum."""
import time
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

        self.declare_parameter('horizon_steps', 20)
        self.declare_parameter('dt', 0.02)
        self.declare_parameter('tau1_max', 5.0)
        self.declare_parameter('tau2_max', 5.0)
        self.declare_parameter('q_theta1', 10.0)
        self.declare_parameter('q_theta1_dot', 1.0)
        self.declare_parameter('q_theta2', 10.0)
        self.declare_parameter('q_theta2_dot', 1.0)
        self.declare_parameter('r_torque1', 1.0)
        self.declare_parameter('r_torque2', 1.0)

        self.N = self.get_parameter('horizon_steps').value
        self.dt = self.get_parameter('dt').value
        self.tau_max = np.array([
            self.get_parameter('tau1_max').value,
            self.get_parameter('tau2_max').value,
        ])

        self.last_X = None
        self.last_U = None
        self._build_opti()

        self.state_sub = self.create_subscription(
            PendulumState2, '/double_inverted/state', self.state_callback, 10)
        self.torque_pub = self.create_publisher(
            TorqueCmd2, '/double_inverted/torque_cmd', 10)

    def _dynamics_ca(self, x, u):
        th1, th1_dot, th2, th2_dot = x[0], x[1], x[2], x[3]
        tau1, tau2 = u[0], u[1]

        m1 = m2 = 1.0
        l1 = l2 = 0.5
        lc1 = lc2 = 0.25
        I1 = I2 = 0.25
        g = 9.81

        d11 = m1*lc1**2 + m2*(l1**2 + lc2**2 + 2*l1*lc2*ca.cos(th2)) + I1 + I2
        d12 = m2*(lc2**2 + l1*lc2*ca.cos(th2)) + I2
        d22 = m2*lc2**2 + I2

        c1 = -m2*l1*lc2*ca.sin(th2)*th2_dot*(2*th1_dot + th2_dot)
        c2 = m2*l1*lc2*ca.sin(th2)*th1_dot**2

        g1 = (m1*lc1 + m2*l1)*g*ca.sin(th1) + m2*lc2*g*ca.sin(th1+th2)
        g2 = m2*lc2*g*ca.sin(th1+th2)

        b1, b2 = tau1 - c1 - g1, tau2 - c2 - g2
        det = d11*d22 - d12*d12
        ddot1 = (d22*b1 - d12*b2) / det
        ddot2 = (-d12*b1 + d11*b2) / det

        return ca.vertcat(th1_dot, ddot1, th2_dot, ddot2)

    def _rk4_ca(self, x, u):
        k1 = self._dynamics_ca(x, u)
        k2 = self._dynamics_ca(x + self.dt/2*k1, u)
        k3 = self._dynamics_ca(x + self.dt/2*k2, u)
        k4 = self._dynamics_ca(x + self.dt*k3, u)
        return x + self.dt/6*(k1 + 2*k2 + 2*k3 + k4)

    def _build_opti(self):
        self.opti = ca.Opti()
        self.X = self.opti.variable(N_STATES, self.N + 1)
        self.U = self.opti.variable(N_CONTROLS, self.N)
        
        # We declare the initial state AND the tuning weights as CasADi parameters
        self.x0_param = self.opti.parameter(N_STATES)
        self.q_param = self.opti.parameter(N_STATES)
        self.r_param = self.opti.parameter(N_CONTROLS)

        self.opti.subject_to(self.X[:, 0] == self.x0_param)
        for k in range(self.N):
            self.opti.subject_to(self.X[:, k+1] == self._rk4_ca(self.X[:, k], self.U[:, k]))

        # Build the cost matrices using the CasADi parameters, NOT fixed numbers
        Q = ca.diag(self.q_param)
        R = ca.diag(self.r_param)
        Q_terminal = ca.diag(self.q_param * 500.0)

        cost = 0
        for k in range(self.N):
            cost += ca.mtimes([self.X[:, k].T, Q, self.X[:, k]])
            cost += ca.mtimes([self.U[:, k].T, R, self.U[:, k]])
        cost += ca.mtimes([self.X[:, self.N].T, Q_terminal, self.X[:, self.N]])
        self.opti.minimize(cost)

        for k in range(self.N):
            self.opti.subject_to(self.opti.bounded(-self.tau_max[0], self.U[0, k], self.tau_max[0]))
            self.opti.subject_to(self.opti.bounded(-self.tau_max[1], self.U[1, k], self.tau_max[1]))

        self.opti.solver('ipopt',
                          {"expand": True, "print_time": False},
                          {"max_iter": 50, "print_level": 0, "tol": 1e-4})

    def state_callback(self, msg: PendulumState2):
        start = time.time()
        
        # Inject the current physical state
        self.opti.set_value(self.x0_param,
                             [msg.theta1, msg.theta1_dot, msg.theta2, msg.theta2_dot])

        # Dynamically pull the latest values from rqt_reconfigure and inject them
        current_q = [self.get_parameter(p).value for p in 
                     ('q_theta1', 'q_theta1_dot', 'q_theta2', 'q_theta2_dot')]
        current_r = [self.get_parameter(p).value for p in ('r_torque1', 'r_torque2')]
        
        self.opti.set_value(self.q_param, current_q)
        self.opti.set_value(self.r_param, current_r)

        if self.last_X is not None:
            self.opti.set_initial(self.X, ca.horzcat(self.last_X[:, 1:], self.last_X[:, -1]))
            self.opti.set_initial(self.U, ca.horzcat(self.last_U[:, 1:], self.last_U[:, -1]))

        try:
            sol = self.opti.solve()
            self.last_X, self.last_U = sol.value(self.X), sol.value(self.U)
            u0 = sol.value(self.U[:, 0])
        except RuntimeError:
            self.get_logger().warn('IPOPT failed, reusing last plan')
            if self.last_U is not None:
                u0 = self.last_U[:, 1]
                self.last_X = self.opti.debug.value(self.X)
                self.last_U = self.opti.debug.value(self.U)
            else:
                u0 = [0.0, 0.0]

        self.get_logger().info(f'Solve time: {(time.time()-start)*1000:.2f} ms')

        cmd = TorqueCmd2()
        cmd.torque1 = float(u0[0])  # FIXED attribute name
        cmd.torque2 = float(u0[1])  # FIXED attribute name
        self.torque_pub.publish(cmd)


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