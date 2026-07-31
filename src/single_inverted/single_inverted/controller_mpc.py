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
import time
from rclpy.qos import qos_profile_sensor_data

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

        # 1. Declare exact names from params.yaml
        self.declare_parameter('horizon_steps', 20)
        self.declare_parameter('dt', 0.02)
        self.declare_parameter('tau_max', 5.0)
        
        self.declare_parameter('q_theta', 10.0)
        self.declare_parameter('q_theta_dot', 1.0)
        self.declare_parameter('r_torque', 1.0)
        self.declare_parameter('q_terminal_theta', 50.0)
        self.declare_parameter('q_terminal_theta_dot', 5.0)

        # 2. Extract the values
        self.horizon_steps = self.get_parameter('horizon_steps').value
        self.step_size = self.get_parameter('dt').value
        self.tau_max = self.get_parameter('tau_max').value
        
        # 3. Assemble the matrices manually
        q_1 = self.get_parameter('q_theta').value
        q_2 = self.get_parameter('q_theta_dot').value
        r_1 = self.get_parameter('r_torque').value
        p_1 = self.get_parameter('q_terminal_theta').value
        p_2 = self.get_parameter('q_terminal_theta_dot').value

        self.penalty_Q = np.diag([q_1, q_2])
        self.penalty_R = np.diag([r_1])
        self.penalty_P = np.diag([p_1, p_2])


        self.opti = ca.Opti()

        # Decision variables: X (states), U (controls)
        self.predicted_states = self.opti.variable(2, self.horizon_steps + 1)
        self.predicted_torques = self.opti.variable(1, self.horizon_steps)

        # Parameters (Placeholders for real-time data)
        self.current_state_sensor = self.opti.parameter(2, 1)
        self.target_state = self.opti.parameter(2, 1)
        self.opti.set_value(self.target_state, [0.0, 0.0]) # Target: Upright & Still

        total_penalty_score = 0

        # Anchor constraint: Imaginary timeline starts at current physical reality
        self.opti.subject_to(self.predicted_states[:, 0] == self.current_state_sensor)

        # Build the chain
        for k in range(self.horizon_steps):
            state_k = self.predicted_states[:, k]
            torque_k = self.predicted_torques[:, k]
            
            # Tracking + Control Effort Cost
            error_k = state_k - self.target_state
            state_penalty = ca.mtimes([error_k.T, self.penalty_Q, error_k])
            motor_penalty = ca.mtimes([torque_k.T, self.penalty_R, torque_k])
            total_penalty_score += (state_penalty + motor_penalty)
            
            # Dynamics constraints via RK4
            k1 = self.calculate_physics_rates(state_k, torque_k)
            k2 = self.calculate_physics_rates(state_k + (self.step_size / 2.0) * k1, torque_k)
            k3 = self.calculate_physics_rates(state_k + (self.step_size / 2.0) * k2, torque_k)
            k4 = self.calculate_physics_rates(state_k + self.step_size * k3, torque_k)
            
            where_i_should_be_next = state_k + (self.step_size / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
            
            self.opti.subject_to(self.predicted_states[:, k+1] == where_i_should_be_next)
            
            # Bounds: |tau| <= tau_max
            self.opti.subject_to(self.opti.bounded(-self.tau_max, torque_k, self.tau_max))

        # Terminal Cost
        final_state = self.predicted_states[:, self.horizon_steps]
        final_error = final_state - self.target_state
        total_penalty_score += ca.mtimes([final_error.T, self.penalty_P, final_error])

        self.opti.minimize(total_penalty_score)
        self.opti.solver("ipopt", {"expand": True}, {"max_iter": 100, "print_level": 0, "tol": 1e-4})

        # Warm-start cache
        self.memory_last_states = None
        self.memory_last_torques = None

        self.state_sub = self.create_subscription(
            PendulumState, '/single_inverted/state', self.state_callback, qos_profile_sensor_data)
        self.torque_pub = self.create_publisher(
            TorqueCmd, '/single_inverted/torque_cmd', 10)


    def calculate_physics_rates(self, state, torque):
        angle = state[0]
        velocity = state[1]
        motor_push = torque[0]
        
        # This MUST perfectly match the sim_node parameters in params.yaml
        pendulum_params = {
            'm': 1.0,
            'l': 0.5,    # Corrected length
            'I': 0.25,   # Corrected inertia
            'b': 0.05,   # Corrected damping
            'g': 9.81
        }
        
        acceleration = dynamics.theta_ddot(angle, velocity, motor_push, pendulum_params)
        return ca.vertcat(velocity, acceleration)

    def state_callback(self, msg: PendulumState):
        raw_angle = msg.theta
        
        # CRITICAL FIX: Wrap the angle to strictly stay within [-pi, pi]
        # This ensures the top is ALWAYS exactly 0.0, no matter how many times it spins.
        angle = (raw_angle + np.pi) % (2 * np.pi) - np.pi
        velocity = msg.theta_dot
        
        real_world_state = np.array([angle, velocity])
        self.opti.set_value(self.current_state_sensor, real_world_state)
    
        # ==========================================
        # THE SHADOW MPC (Continuously Solving)
        # ==========================================
        if self.memory_last_states is not None and self.memory_last_torques is not None:
            last_x = np.atleast_2d(self.memory_last_states)
            if last_x.shape[0] != 2: last_x = last_x.T
            
            last_u = np.atleast_2d(self.memory_last_torques)
            if last_u.shape[0] != 1: last_u = last_u.T

            guessed_states = np.hstack((last_x[:, 1:], last_x[:, -1:]))
            guessed_torques = np.hstack((last_u[:, 1:], last_u[:, -1:]))

            self.opti.set_initial(self.predicted_states, guessed_states)
            self.opti.set_initial(self.predicted_torques, guessed_torques)
        else:
            for k in range(self.horizon_steps + 1):
                self.opti.set_initial(self.predicted_states[:, k], real_world_state)
            self.opti.set_initial(self.predicted_torques, np.zeros((1, self.horizon_steps)))

        try:
            solution = self.opti.solve()
            
            _states = np.atleast_2d(solution.value(self.predicted_states))
            self.memory_last_states = _states if _states.shape[0] == 2 else _states.T
            
            _torques = np.atleast_2d(solution.value(self.predicted_torques))
            self.memory_last_torques = _torques if _torques.shape[0] == 1 else _torques.T
            
            # Extract the optimal torque, but do NOT publish it yet
            tau_mpc = self.memory_last_torques[0, 0]

        except Exception as e:
            tau_mpc = 0.0
            self.memory_last_states = None 
            self.memory_last_torques = None

        # ==========================================
        # HYBRID ROUTER (Control Authority Switch)
        # ==========================================
        catch_zone = 0.5  # Widen the net to give the MPC more braking runway

        if abs(angle) < catch_zone:
            # Phase 2: MPC takes authority
            self.get_logger().info(f"CATCH ZONE: Stabilizing at theta={angle:.2f}")
            tau_final = tau_mpc
        else:
            # Phase 1: Bang-Bang Heuristic Pump
            pump_gain = 0.7
            if velocity > 0:
                tau_final = pump_gain
            else:
                tau_final = -pump_gain
                
            # Hard limit against the motor's physical constraints
            tau_final = max(min(tau_final, self.tau_max), -self.tau_max)

        # Publish the final routed command
        cmd_msg = TorqueCmd()
        cmd_msg.torque = float(tau_final)
        self.torque_pub.publish(cmd_msg)


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