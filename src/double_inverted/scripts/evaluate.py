#!/usr/bin/env python3
"""Objective grader for the double inverted pendulum. GIVEN -- full
implementation, do not modify.

Subscribes to /double_inverted/state and /double_inverted/torque_cmd, runs
for a fixed duration, and checks settling time, overshoot, steady-state
error, and torque-saturation violations (both joints) against
config/eval_thresholds.yaml. See README.md Sec 8.

Usage (with a sim + controller already running via the launch file):
    ros2 run double_inverted evaluate.py --exercise mpc_balance --duration 15
"""
import argparse
import sys

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from custom_msgs.msg import PendulumState2, TorqueCmd2


def wrap_to_pi(theta):
    return (theta + np.pi) % (2 * np.pi) - np.pi


class Evaluator(Node):
    def __init__(self, duration):
        super().__init__('evaluate')
        self.duration = duration

        self.t0 = None
        self.samples = []  # (t, theta1, theta1_dot, theta2, theta2_dot)
        self.torques = []  # (t, tau1, tau2)

        self.create_subscription(PendulumState2, '/double_inverted/state', self._state_cb, 50)
        self.create_subscription(TorqueCmd2, '/double_inverted/torque_cmd', self._torque_cb, 50)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _state_cb(self, msg: PendulumState2):
        if self.t0 is None:
            self.t0 = self._now()
        t = self._now() - self.t0
        self.samples.append((t, msg.theta1, msg.theta1_dot, msg.theta2, msg.theta2_dot))

    def _torque_cb(self, msg: TorqueCmd2):
        if self.t0 is None:
            return
        t = self._now() - self.t0
        self.torques.append((t, msg.torque1, msg.torque2))

    def done(self):
        return self.t0 is not None and (self._now() - self.t0) >= self.duration


def compute_metrics(samples, torques, tau1_max, tau2_max, band):
    times = np.array([s[0] for s in samples])
    theta1 = np.array([wrap_to_pi(s[1]) for s in samples])
    theta2 = np.array([wrap_to_pi(s[3]) for s in samples])
    err = np.maximum(np.abs(theta1), np.abs(theta2))

    in_band = err <= band

    settle_idx = None
    for i in range(len(in_band)):
        if in_band[i:].all():
            settle_idx = i
            break

    settling_time = float(times[settle_idx]) if settle_idx is not None else float('inf')
    max_overshoot = float(np.max(err)) if len(err) else float('inf')
    steady_state_error = (
        float(np.max(err[settle_idx:])) if settle_idx is not None else float('inf')
    )

    saturation_violations = sum(
        1 for _, tau1, tau2 in torques
        if abs(tau1) > tau1_max + 1e-9 or abs(tau2) > tau2_max + 1e-9)

    return {
        'settling_time_s': settling_time,
        'max_overshoot_rad': max_overshoot,
        'steady_state_error_rad': steady_state_error,
        'saturation_violations': saturation_violations,
    }


def grade(metrics, thresholds):
    return {
        'settling_time_s': metrics['settling_time_s'] <= thresholds['settling_time_s'],
        'max_overshoot_rad': metrics['max_overshoot_rad'] <= thresholds['max_overshoot_rad'],
        'steady_state_error_rad': (
            metrics['steady_state_error_rad'] <= thresholds['steady_state_error_rad']),
        'saturation_violations': (
            metrics['saturation_violations'] <= thresholds['max_saturation_violations']),
    }


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--exercise', required=True, choices=['balance', 'swingup', 'mpc_balance'])
    parser.add_argument('--duration', type=float, default=15.0)
    parser.add_argument('--tau1-max', type=float, default=5.0)
    parser.add_argument('--tau2-max', type=float, default=5.0)
    parser.add_argument('--thresholds-file', default=None,
                         help='defaults to <pkg share>/config/eval_thresholds.yaml')
    parsed = parser.parse_args(args=args)

    thresholds_file = parsed.thresholds_file
    if thresholds_file is None:
        thresholds_file = (
            get_package_share_directory('double_inverted') + '/config/eval_thresholds.yaml')
    with open(thresholds_file) as f:
        all_thresholds = yaml.safe_load(f)
    thresholds = all_thresholds[parsed.exercise]

    rclpy.init()
    node = Evaluator(parsed.duration)

    print(f"[evaluate] running exercise='{parsed.exercise}' for {parsed.duration}s ...")
    while rclpy.ok() and not node.done():
        rclpy.spin_once(node, timeout_sec=0.1)

    metrics = compute_metrics(node.samples, node.torques, parsed.tau1_max, parsed.tau2_max,
                               thresholds['settling_band_rad'])
    checks = grade(metrics, thresholds)
    passed = all(checks.values())

    print('\n=== evaluate.py results ===')
    for k, v in metrics.items():
        print(f'  {k}: {v}')
    print('--- pass/fail ---')
    for k, v in checks.items():
        print(f"  {k}: {'PASS' if v else 'FAIL'}")
    print(f"\nOVERALL: {'PASS' if passed else 'FAIL'}")

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if passed else 1)


if __name__ == '__main__':
    main()
