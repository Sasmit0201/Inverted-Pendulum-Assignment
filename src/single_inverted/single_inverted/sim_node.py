"""Single inverted pendulum simulator node. GIVEN -- do not modify.

Integrates the nonlinear EOM in dynamics.py with fixed-step RK4, publishes
state + joint_states, subscribes to torque commands, applies torque
saturation, and supports configurable initial conditions (upright,
near_upright, downward, random) -- see README.md Sec 4.1.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from custom_msgs.msg import PendulumState, TorqueCmd

from single_inverted import dynamics


class SimNode(Node):
    def __init__(self):
        super().__init__('sim_node')

        # --- physical / sim parameters (overridable via config/params.yaml) ---
        self.declare_parameter('m', 1.0)
        self.declare_parameter('l', 0.5)
        self.declare_parameter('I', 0.25)
        self.declare_parameter('b', 0.05)
        self.declare_parameter('g', 9.81)
        self.declare_parameter('tau_max', 5.0)
        self.declare_parameter('sim_rate_hz', 200.0)
        self.declare_parameter('ic_mode', 'downward')  # upright | near_upright | downward | random
        self.declare_parameter('theta0_upright', 0.0)
        self.declare_parameter('theta_dot0_upright', 0.0)
        self.declare_parameter('theta0_near_upright', 0.2)
        self.declare_parameter('theta_dot0_near_upright', 0.0)
        self.declare_parameter('theta0_downward', float(np.pi))
        self.declare_parameter('theta_dot0_downward', 0.0)
        self.declare_parameter('theta0_random_range', [-np.pi, np.pi])
        self.declare_parameter('theta_dot0_random_range', [-0.5, 0.5])

        self.params = {
            'm': self.get_parameter('m').value,
            'l': self.get_parameter('l').value,
            'I': self.get_parameter('I').value,
            'b': self.get_parameter('b').value,
            'g': self.get_parameter('g').value,
        }
        self.tau_max = float(self.get_parameter('tau_max').value)
        sim_rate_hz = float(self.get_parameter('sim_rate_hz').value)
        self.dt = 1.0 / sim_rate_hz

        self.theta, self.theta_dot = self._initial_condition()
        self.tau_cmd = 0.0

        self.state_pub = self.create_publisher(PendulumState, '/single_inverted/state', 10)
        self.joint_pub = self.create_publisher(JointState, '/single_inverted/joint_states', 10)
        self.torque_sub = self.create_subscription(
            TorqueCmd, '/single_inverted/torque_cmd', self._torque_cb, 10)

        self.timer = self.create_timer(self.dt, self._step)
        self.get_logger().info(
            f"sim_node started: ic_mode={self.get_parameter('ic_mode').value}, "
            f"theta0={self.theta:.3f}, dt={self.dt:.4f}s")

    def _initial_condition(self):
        mode = self.get_parameter('ic_mode').value
        if mode == 'upright':
            return (float(self.get_parameter('theta0_upright').value),
                     float(self.get_parameter('theta_dot0_upright').value))
        if mode == 'near_upright':
            return (float(self.get_parameter('theta0_near_upright').value),
                     float(self.get_parameter('theta_dot0_near_upright').value))
        if mode == 'random':
            th_lo, th_hi = self.get_parameter('theta0_random_range').value
            thd_lo, thd_hi = self.get_parameter('theta_dot0_random_range').value
            return (float(np.random.uniform(th_lo, th_hi)),
                     float(np.random.uniform(thd_lo, thd_hi)))
        # default: downward
        return (float(self.get_parameter('theta0_downward').value),
                 float(self.get_parameter('theta_dot0_downward').value))

    def _torque_cb(self, msg: TorqueCmd):
        self.tau_cmd = float(np.clip(msg.torque, -self.tau_max, self.tau_max))

    def _rk4_step(self, theta, theta_dot, tau):
        def deriv(th, thd):
            return thd, dynamics.theta_ddot(th, thd, tau, self.params)

        k1 = deriv(theta, theta_dot)
        k2 = deriv(theta + 0.5 * self.dt * k1[0], theta_dot + 0.5 * self.dt * k1[1])
        k3 = deriv(theta + 0.5 * self.dt * k2[0], theta_dot + 0.5 * self.dt * k2[1])
        k4 = deriv(theta + self.dt * k3[0], theta_dot + self.dt * k3[1])
        theta_next = theta + (self.dt / 6.0) * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
        theta_dot_next = theta_dot + (self.dt / 6.0) * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])
        return theta_next, theta_dot_next

    def _step(self):
        self.theta, self.theta_dot = self._rk4_step(self.theta, self.theta_dot, self.tau_cmd)

        now = self.get_clock().now().to_msg()

        state_msg = PendulumState()
        state_msg.header = Header(stamp=now, frame_id='pendulum')
        state_msg.theta = self.theta
        state_msg.theta_dot = self.theta_dot
        self.state_pub.publish(state_msg)

        joint_msg = JointState()
        joint_msg.header = Header(stamp=now, frame_id='pendulum')
        joint_msg.name = ['pivot_joint']
        joint_msg.position = [self.theta]
        joint_msg.velocity = [self.theta_dot]
        self.joint_pub.publish(joint_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
