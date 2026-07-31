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
import math

import rclpy
from rclpy.node import Node

from custom_msgs.msg import PendulumState, TorqueCmd


class PIDController(Node):
    def __init__(self):
        super().__init__('controller_pid')

        # TODO: declare/load kp, ki, kd, tau_max, upright_threshold_rad,
        # and swingup_gain from config/params.yaml (see the
        # controller_pid section of that file for the expected keys).

        '''
        FOR EXERCISE A: WE DONT NEED TO MAKE CHANGES TO THIS FUNCTION
        We just need to ensure that the controller is publishing torque commands to the /single_inverted/torque_cmd topic. 
        Theres not the requirement to add parameters 
        '''

        '''
        Exercise B: We need to add the parameters for the PID controller as follows:
        '''

        # Declare the parameters that are in config/params.yaml
        self.declare_parameter('kp', 0.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('tau_max', 0.0)
        # (We will use these next ones in Exercise C, but let's declare them now)
        self.declare_parameter('upright_threshold_rad', 0.0)
        self.declare_parameter('swingup_gain', 0.0)

        # Setup the interface (same as Exercise A)
        self.state_sub = self.create_subscription(
            PendulumState, '/single_inverted/state', self.state_callback, 10)
        self.torque_pub = self.create_publisher(
            TorqueCmd, '/single_inverted/torque_cmd', 10)
        
        
        

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

        
        # Exercise A: We ignore the math for now and just push the pendulum constantly to prove that our code can talk to the simulation.
    

        # cmd = TorqueCmd()
        # cmd.torque = 10.0  # Apply 1.0 N*m of torque
        
        # self.torque_pub.publish(cmd)







        #Exercise B: We need to implement the PID controller here. The following is a simple implementation of a PD controller for balancing the pendulum.

        #  Get the current values from the parameter server
        # kp = self.get_parameter('kp').value
        # kd = self.get_parameter('kd').value
        # tau_max = self.get_parameter('tau_max').value

        # # Calculate the Balance Law: tau = -Kp*theta - Kd*theta_dot
        # # (Since upright is theta=0, the error is simply 0 - theta)
        # calculated_torque = -kp * msg.theta - kd * msg.theta_dot
        # # We are changing the target from 0.0 to 0.2 radians





        #Exercise C: We will implement the energy shaping swing-up law here. 

        import math

        # 1. Fetch your tuned parameters
        kp = self.get_parameter('kp').value
        kd = self.get_parameter('kd').value
        k_swing = self.get_parameter('swingup_gain').value
        threshold = self.get_parameter('upright_threshold_rad').value
        tau_max = self.get_parameter('tau_max').value

        # 2. Angle Normalization (Fixes Flaw #1)
        # Forces theta to always be between -pi and pi
        theta_norm = (msg.theta + math.pi) % (2 * math.pi) - math.pi

        # 3. The State Machine
        if abs(theta_norm) < threshold:
            # BALANCE MODE (Exercise B)
            calculated_torque = -kp * theta_norm - kd * msg.theta_dot
        else:
            # SWING-UP MODE (Exercise C)
            
            # Check if we are near the absolute bottom (pi or -pi)
            is_at_bottom = abs(abs(theta_norm) - math.pi) < 0.2
            
            # The Critical Patch: Only kick if it is stopped AND at the bottom
            if abs(msg.theta_dot) < 0.05 and is_at_bottom:
                calculated_torque = 2.0
            else:
                calculated_torque = k_swing * msg.theta_dot








        #  Critical Fix: Clamp the torque to prevent actuator saturation
        if calculated_torque > tau_max:
            calculated_torque = tau_max
        elif calculated_torque < -tau_max:
            calculated_torque = -tau_max

        #  Publish the command
        cmd = TorqueCmd()
        cmd.torque = float(calculated_torque)
        self.torque_pub.publish(cmd)

        # raise NotImplementedError(
        #     'controller_pid.py is a Week 1 trainee deliverable -- see README.md Sec 4.2')


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
