#!/usr/bin/env python3
"""Objective grader for the single inverted pendulum. GIVEN -- full
implementation, do not modify.

Subscribes to /single_inverted/state and /single_inverted/torque_cmd, runs
for a fixed duration, and checks settling time, overshoot, steady-state
error, and torque-saturation violations against config/eval_thresholds.yaml.
Swing-up exercises additionally check time-to-catch (first entry into the
near-upright band). See README.md Sec 8.

Usage (with a sim + controller already running via the launch file):
    ros2 run single_inverted evaluate.py --exercise swingup --duration 15
"""
import argparse
import sys

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from custom_msgs.msg import PendulumState, TorqueCmd


def wrap_to_pi(theta):
    return (theta + np.pi) % (2 * np.pi) - np.pi


class Evaluator(Node):
    def __init__(self, duration):
        super().__init__('evaluate')
        self.duration = duration

        self.t0 = None
        self.samples = []  # (t, theta, theta_dot)
        self.torques = []  # (t, tau)

        self.create_subscription(PendulumState, '/single_inverted/state', self._state_cb, 50)
        self.create_subscription(TorqueCmd, '/single_inverted/torque_cmd', self._torque_cb, 50)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _state_cb(self, msg: PendulumState):
        if self.t0 is None:
            self.t0 = self._now()
        t = self._now() - self.t0
        self.samples.append((t, msg.theta, msg.theta_dot))

    def _torque_cb(self, msg: TorqueCmd):
        if self.t0 is None:
            return
        t = self._now() - self.t0
        self.torques.append((t, msg.torque))

    def done(self):
        return self.t0 is not None and (self._now() - self.t0) >= self.duration


def compute_metrics(samples, torques, tau_max, target_theta, band, time_to_catch_required):
    times = np.array([s[0] for s in samples])
    thetas = np.array([wrap_to_pi(s[1] - target_theta) for s in samples])

    in_band = np.abs(thetas) <= band

    settle_idx = None
    for i in range(len(in_band)):
        if in_band[i:].all():
            settle_idx = i
            break

    settling_time = float(times[settle_idx]) if settle_idx is not None else float('inf')
    time_to_catch = (
        float(times[int(np.argmax(in_band))]) if time_to_catch_required and in_band.any()
        else (float('inf') if time_to_catch_required else None)
    )

    max_overshoot = float(np.max(np.abs(thetas))) if len(thetas) else float('inf')
    steady_state_error = (
        float(np.max(np.abs(thetas[settle_idx:]))) if settle_idx is not None else float('inf')
    )

    saturation_violations = sum(1 for _, tau in torques if abs(tau) > tau_max + 1e-9)

    return {
        'settling_time_s': settling_time,
        'time_to_catch_s': time_to_catch,
        'max_overshoot_rad': max_overshoot,
        'steady_state_error_rad': steady_state_error,
        'saturation_violations': saturation_violations,
    }


def grade(metrics, thresholds):
    checks = {
        'settling_time_s': metrics['settling_time_s'] <= thresholds['settling_time_s'],
        'max_overshoot_rad': metrics['max_overshoot_rad'] <= thresholds['max_overshoot_rad'],
        'steady_state_error_rad': (
            metrics['steady_state_error_rad'] <= thresholds['steady_state_error_rad']),
        'saturation_violations': (
            metrics['saturation_violations'] <= thresholds['max_saturation_violations']),
    }
    if 'time_to_catch_s' in thresholds and metrics.get('time_to_catch_s') is not None:
        checks['time_to_catch_s'] = metrics['time_to_catch_s'] <= thresholds['time_to_catch_s']
    return checks


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--exercise', required=True,
                         choices=['balance', 'swingup', 'mpc_balance'])
    parser.add_argument('--duration', type=float, default=15.0)
    parser.add_argument('--target-theta', type=float, default=0.0)
    parser.add_argument('--tau-max', type=float, default=5.0)
    parser.add_argument('--thresholds-file', default=None,
                         help='defaults to <pkg share>/config/eval_thresholds.yaml')
    parsed = parser.parse_args(args=args)

    thresholds_file = parsed.thresholds_file
    if thresholds_file is None:
        thresholds_file = (
            get_package_share_directory('single_inverted') + '/config/eval_thresholds.yaml')
    with open(thresholds_file) as f:
        all_thresholds = yaml.safe_load(f)
    thresholds = all_thresholds[parsed.exercise]

    rclpy.init()
    node = Evaluator(parsed.duration)

    print(f"[evaluate] running exercise='{parsed.exercise}' for {parsed.duration}s ...")
    while rclpy.ok() and not node.done():
        rclpy.spin_once(node, timeout_sec=0.1)

    time_to_catch_required = 'time_to_catch_s' in thresholds
    metrics = compute_metrics(node.samples, node.torques, parsed.tau_max,
                               parsed.target_theta, thresholds['settling_band_rad'],
                               time_to_catch_required)
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
