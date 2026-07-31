"""Double inverted pendulum simulator node. SCAFFOLDED.

Message wiring (subscriptions, publishers, parameter declarations) is
GIVEN. The RK4 integration step -- including the call into
dynamics.theta_ddot -- is your TODO. This depends on
double_inverted/dynamics.py, which you implement by translating the
given equations in docs/eom_derivation.md (see README.md Sec 5.1). Use
single_inverted/sim_node.py (fully GIVEN) as a reference for the RK4
pattern -- the structure is identical, just with a 4-state vector
instead of 2.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from custom_msgs.msg import PendulumState2, TorqueCmd2

from double_inverted import dynamics


class SimNode(Node):
    def __init__(self):
        super().__init__('sim_node')

        # --- physical / sim parameters (overridable via config/params.yaml) ---
        self.declare_parameter('m1', 1.0)
        self.declare_parameter('l1', 0.5)
        self.declare_parameter('lc1', 0.25)
        self.declare_parameter('I1', 0.25)
        self.declare_parameter('b1', 0.05)
        self.declare_parameter('m2', 1.0)
        self.declare_parameter('l2', 0.5)
        self.declare_parameter('lc2', 0.25)
        self.declare_parameter('I2', 0.25)
        self.declare_parameter('b2', 0.05)
        self.declare_parameter('g', 9.81)
        self.declare_parameter('tau1_max', 5.0)
        self.declare_parameter('tau2_max', 5.0)
        self.declare_parameter('sim_rate_hz', 200.0)
        self.declare_parameter('ic_mode', 'downward')  # upright | downward | random
        self.declare_parameter('theta10_downward', float(np.pi))
        self.declare_parameter('theta20_downward', 0.0)

        self.params = {
            'm1': self.get_parameter('m1').value, 'l1': self.get_parameter('l1').value,
            'lc1': self.get_parameter('lc1').value,
            'I1': self.get_parameter('I1').value, 'b1': self.get_parameter('b1').value,
            'm2': self.get_parameter('m2').value, 'l2': self.get_parameter('l2').value,
            'lc2': self.get_parameter('lc2').value,
            'I2': self.get_parameter('I2').value, 'b2': self.get_parameter('b2').value,
            'g': self.get_parameter('g').value,
        }
        self.tau1_max = float(self.get_parameter('tau1_max').value)
        self.tau2_max = float(self.get_parameter('tau2_max').value)
        sim_rate_hz = float(self.get_parameter('sim_rate_hz').value)
        self.dt = 1.0 / sim_rate_hz

        # state = [theta1, theta1_dot, theta2, theta2_dot]
        self.state = self._initial_condition()
        self.tau_cmd = np.zeros(2)

        self.state_pub = self.create_publisher(PendulumState2, '/double_inverted/state', 10)
        self.joint_pub = self.create_publisher(JointState, '/double_inverted/joint_states', 10)
        self.torque_sub = self.create_subscription(
            TorqueCmd2, '/double_inverted/torque_cmd', self._torque_cb, 10)

        self.timer = self.create_timer(self.dt, self._step)
        self.get_logger().info(f'double_inverted sim_node started, dt={self.dt:.4f}s')

    def _initial_condition(self):
        mode = self.get_parameter('ic_mode').value
        if mode == 'upright':
            return np.array([0.1, 0.0, 0.0, 0.0])
        if mode == 'random':
            return np.array([
                np.random.uniform(-np.pi, np.pi), np.random.uniform(-0.5, 0.5),
                np.random.uniform(-np.pi, np.pi), np.random.uniform(-0.5, 0.5),
            ])
        return np.array([
            self.get_parameter('theta10_downward').value, 0.0,
            self.get_parameter('theta20_downward').value, 0.0,
        ])

    def _torque_cb(self, msg: TorqueCmd2):
        self.tau_cmd = np.array([
            float(np.clip(msg.torque1, -self.tau1_max, self.tau1_max)),
            float(np.clip(msg.torque2, -self.tau2_max, self.tau2_max)),
        ])

    def _rk4_step(self, state, tau):
        """RK4 integration of the 2-link EOM.
        
        Calls dynamics.theta_ddot to get accelerations, builds the 4-state 
        derivative, and integrates one dt step with RK4.
        """
        def f(s):
            # 1. Get accelerations: [theta1_ddot, theta2_ddot]
            accels = dynamics.theta_ddot(s, tau, self.params)
            
            # 2. Build 4-state derivative: [theta1_dot, theta1_ddot, theta2_dot, theta2_ddot]
            # s[1] is theta1_dot, s[3] is theta2_dot
            return np.array([s[1], accels[0], s[3], accels[1]])

        # 3. Standard RK4 Integration
        k1 = f(state)
        k2 = f(state + 0.5 * self.dt * k1)
        k3 = f(state + 0.5 * self.dt * k2)
        k4 = f(state + self.dt * k3)

        next_state = state + (self.dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        
        return next_state
    def _step(self):
        self.state = self._rk4_step(self.state, self.tau_cmd)

        now = self.get_clock().now().to_msg()

        state_msg = PendulumState2()
        state_msg.header = Header(stamp=now, frame_id='pendulum')
        state_msg.theta1 = float(self.state[0])
        state_msg.theta1_dot = float(self.state[1])
        state_msg.theta2 = float(self.state[2])
        state_msg.theta2_dot = float(self.state[3])
        self.state_pub.publish(state_msg)

        joint_msg = JointState()
        joint_msg.header = Header(stamp=now, frame_id='pendulum')
        joint_msg.name = ['joint1', 'joint2']
        joint_msg.position = [float(self.state[0]), float(self.state[2])]
        joint_msg.velocity = [float(self.state[1]), float(self.state[3])]
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
