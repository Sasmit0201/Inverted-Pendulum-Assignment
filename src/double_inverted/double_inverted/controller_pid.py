# """Per-joint PID/PD balance + energy-shaping swing-up, decentralized.

# Mode switch: each joint independently checks its own wrapped angle
# against upright_threshold_rad. Near upright -> PD balance. Otherwise ->
# energy-shaping swing-up.
# """
# import rclpy
# from rclpy.node import Node
# import numpy as np

# from custom_msgs.msg import PendulumState2, TorqueCmd2

# # Physical params (match sim_node.yaml)
# M1 = M2 = 1.0
# LC1 = LC2 = 0.25
# I1 = I2 = 0.25
# G = 9.81
# I_EFF1 = I1 + M1 * LC1**2
# I_EFF2 = I2 + M2 * LC2**2
# E_TOP1 = M1 * G * LC1
# E_TOP2 = M2 * G * LC2


# class PIDController(Node):
#     def __init__(self):
#         super().__init__('controller_pid')

#         self.declare_parameter('kp1', 15.0)
#         self.declare_parameter('kd1', 3.0)
#         self.declare_parameter('kp2', 10.0)
#         self.declare_parameter('kd2', 3.0)
#         self.declare_parameter('tau1_max', 5.0)
#         self.declare_parameter('tau2_max', 5.0)
#         self.declare_parameter('upright_threshold_rad', 0.26)
#         self.declare_parameter('swingup_gain1', 1.0)
#         self.declare_parameter('swingup_gain2', 1.0)

#         self.kp1 = self.get_parameter('kp1').value
#         self.kd1 = self.get_parameter('kd1').value
#         self.kp2 = self.get_parameter('kp2').value
#         self.kd2 = self.get_parameter('kd2').value
#         self.tau1_max = self.get_parameter('tau1_max').value
#         self.tau2_max = self.get_parameter('tau2_max').value
#         self.thresh = self.get_parameter('upright_threshold_rad').value
#         self.k1 = self.get_parameter('swingup_gain1').value
#         self.k2 = self.get_parameter('swingup_gain2').value

#         self.state_sub = self.create_subscription(
#             PendulumState2, '/double_inverted/state', self.state_callback, 10)
#         self.torque_pub = self.create_publisher(
#             TorqueCmd2, '/double_inverted/torque_cmd', 10)

#     def state_callback(self, msg: PendulumState2):
#         # Use raw sensor values directly to eliminate transformation bugs
#         theta1 = msg.theta1
#         theta2 = msg.theta2
#         theta1_dot = msg.theta1_dot
#         theta2_dot = msg.theta2_dot

#         # Debug print to prove the node is reading movement
#         print(f"T1: {theta1:.3f} | T1_dot: {theta1_dot:.3f}")

#         # Joint 1 Swing-up / Balance
#         if abs(theta1) < self.thresh:
#             tau1 = -self.kp1 * theta1 - self.kd1 * theta1_dot
#         else:
#             E1 = 0.5 * I_EFF1 * theta1_dot**2 + M1 * G * LC1 * np.cos(theta1)
#             tau1 = self.k1 * (E_TOP1 - E1) * theta1_dot
            
#             # Simple, un-nested dead-bottom kick
#             if abs(abs(theta1) - 3.14) < 0.2:
#                 tau1 = self.tau1_max  # Slam max torque if near bottom

#         # Joint 2 (Keep decentralized simple PD/Swing for now)
#         if abs(theta2) < self.thresh:
#             tau2 = -self.kp2 * theta2 - self.kd2 * theta2_dot
#         else:
#             E2 = 0.5 * I_EFF2 * theta2_dot**2 + M2 * G * LC2 * np.cos(theta2)
#             tau2 = self.k2 * (E_TOP2 - E2) * theta2_dot
#             if abs(abs(theta2) - 3.14) < 0.2:
#                 tau2 = self.tau2_max

#         tau1 = max(min(tau1, self.tau1_max), -self.tau1_max)
#         tau2 = max(min(tau2, self.tau2_max), -self.tau2_max)

#         cmd_msg = TorqueCmd2()
#         cmd_msg.torque1 = float(tau1)  
#         cmd_msg.torque2 = float(tau2)  
#         self.torque_pub.publish(cmd_msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = PIDController()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

"""Per-joint PID balance ONLY, decentralized.

Strictly applies proportional and derivative gains to regulate 
the pendulum near the upright equilibrium point. No swing-up logic.
"""
import rclpy
from rclpy.node import Node

from custom_msgs.msg import PendulumState2, TorqueCmd2


class PIDBalanceController(Node):
    def __init__(self):
        super().__init__('controller_pid_balance')

        # Declare parameters (Tuned for the balance phase)
        self.declare_parameter('kp1', 15.0)
        self.declare_parameter('kd1', 3.0)
        self.declare_parameter('kp2', 10.0)
        self.declare_parameter('kd2', 3.0)
        self.declare_parameter('tau1_max', 5.0)
        self.declare_parameter('tau2_max', 5.0)

        # Retrieve parameters
        self.kp1 = self.get_parameter('kp1').value
        self.kd1 = self.get_parameter('kd1').value
        self.kp2 = self.get_parameter('kp2').value
        self.kd2 = self.get_parameter('kd2').value
        self.tau1_max = self.get_parameter('tau1_max').value
        self.tau2_max = self.get_parameter('tau2_max').value

        # Set up ROS 2 communication
        self.state_sub = self.create_subscription(
            PendulumState2, '/double_inverted/state', self.state_callback, 10)
        self.torque_pub = self.create_publisher(
            TorqueCmd2, '/double_inverted/torque_cmd', 10)

    def state_callback(self, msg: PendulumState2):
        # 1. Read raw sensor states
        theta1 = msg.theta1
        theta2 = msg.theta2
        theta1_dot = msg.theta1_dot
        theta2_dot = msg.theta2_dot

        # Debug print to verify the node is running
        print(f"PID Active | T1: {theta1:.3f} | T2: {theta2:.3f}")

        # 2. Pure Decentralized PID Math
        # No energy shaping, just immediate reaction to positional error
        tau1 = -self.kp1 * theta1 - self.kd1 * theta1_dot
        tau2 = -self.kp2 * theta2 - self.kd2 * theta2_dot

        # 3. Saturation Protection (Do not exceed physical motor limits)
        tau1 = max(min(tau1, self.tau1_max), -self.tau1_max)
        tau2 = max(min(tau2, self.tau2_max), -self.tau2_max)

        # 4. Publish the final commands
        cmd_msg = TorqueCmd2()
        cmd_msg.torque1 = float(tau1)  
        cmd_msg.torque2 = float(tau2)  
        self.torque_pub.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PIDBalanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()