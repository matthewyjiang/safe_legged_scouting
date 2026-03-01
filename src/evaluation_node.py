#!/usr/bin/env python3

import math
import os
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PointStamped
from safe_scout_simulator.srv import SampleGroundTruth


class EvaluationNode(Node):

    def __init__(self):
        super().__init__('evaluation_node')

        # Goal state
        self.goal_point = None  # (x, y)
        self.goal_reached = False

        # Timing
        self.start_time = None  # set when first goal is received
        self.end_time = None    # set when goal is reached

        # Path length tracking (discrete-time integral of 2D position change)
        self.path_length = 0.0
        self.prev_position = None  # (x, y)
        self.path_coordinates = []  # list of (x, y) at each pose update

        # Safety tracking via ground truth service
        self.declare_parameter('f_max', 0.5)
        self.f_max = self.get_parameter('f_max').value
        self.safety_violations = 0
        self.total_safety_checks = 0
        self.pending_safety_future = None  # in-flight async service request

        # Ground truth service client
        self.sample_client = self.create_client(
            SampleGroundTruth, 'sample_ground_truth'
        )

        # Goal tolerance (matches reactive planner and goal_point_publisher)
        self.goal_tolerance = 0.3

        # Whether we already printed the summary
        self.summary_printed = False

        # Pose count for watchdog reporting
        self.pose_count = 0

        # Subscribers
        self.pose_sub = self.create_subscription(
            Pose,
            'spirit/current_pose',
            self.pose_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            PointStamped,
            'goal_point',
            self.goal_callback,
            10
        )

        # Watchdog timer (1 Hz) to confirm node is alive
        self.watchdog_timer = self.create_timer(1.0, self.watchdog_callback)

        self.get_logger().info(
            f'Evaluation node initialized (f_max={self.f_max}, '
            f'goal_tolerance={self.goal_tolerance})'
        )

    def watchdog_callback(self):
        # If goal was reached but summary never printed, try printing inline
        if self.goal_reached and not self.summary_printed:
            self.get_logger().info(
                f'[WATCHDOG] goal_reached but summary not printed, '
                f'summary_printed={self.summary_printed}'
            )
            self.print_summary()
            self.get_logger().info(
                f'[WATCHDOG] after print_summary call, '
                f'summary_printed={self.summary_printed}'
            )

        self.get_logger().info(
            f'[WATCHDOG] alive | poses={self.pose_count} | '
            f'safety_checks={self.total_safety_checks} | '
            f'violations={self.safety_violations} | '
            f'goal_reached={self.goal_reached} | '
            f'summary_printed={self.summary_printed}'
        )

    def goal_callback(self, msg: PointStamped):
        self.goal_point = (msg.point.x, msg.point.y)
        if self.start_time is None:
            self.start_time = time.monotonic()
            self.get_logger().info(
                f'Goal received: ({msg.point.x:.2f}, {msg.point.y:.2f}) - timer started'
            )

    def pose_callback(self, msg: Pose):
        x = msg.position.x
        y = msg.position.y
        self.pose_count += 1

        # Accumulate path length: discrete integral of ||delta_p||
        if self.prev_position is not None:
            dx = x - self.prev_position[0]
            dy = y - self.prev_position[1]
            self.path_length += math.sqrt(dx * dx + dy * dy)
        self.prev_position = (x, y)
        self.path_coordinates.append((x, y))

        # Safety check: collect result from previous async request
        if self.pending_safety_future is not None:
            if self.pending_safety_future.done():
                try:
                    result = self.pending_safety_future.result()
                    if result is not None:
                        self.total_safety_checks += 1
                        if result.value > self.f_max:
                            self.safety_violations += 1
                except Exception as e:
                    self.get_logger().error(
                        f'Safety service call failed: {e}',
                        throttle_duration_sec=5.0
                    )
                self.pending_safety_future = None

        # Fire off new async safety check for current position
        if self.sample_client.service_is_ready():
            req = SampleGroundTruth.Request()
            req.x = float(x)
            req.y = float(y)
            self.pending_safety_future = self.sample_client.call_async(req)

        # Goal reached check
        if self.goal_point is not None and not self.goal_reached:
            dist = math.sqrt(
                (x - self.goal_point[0]) ** 2 + (y - self.goal_point[1]) ** 2
            )
            if dist <= self.goal_tolerance:
                try:
                    self.goal_reached = True
                    self.end_time = time.monotonic()
                    self.get_logger().info(
                        f'Goal reached! Distance: {dist:.4f}m'
                    )
                    self.print_summary()
                except Exception as e:
                    print(f'EXCEPTION in goal reached block: {e}', flush=True)

    def print_summary(self):
        if self.summary_printed:
            return

        try:
            if self.start_time is not None:
                end = self.end_time if self.end_time is not None else time.monotonic()
                elapsed = end - self.start_time
            else:
                elapsed = 0.0

            goal_str = (
                f'({self.goal_point[0]:.2f}, {self.goal_point[1]:.2f})'
                if self.goal_point else 'N/A'
            )

            if self.total_safety_checks > 0:
                violation_pct = (self.safety_violations / self.total_safety_checks) * 100.0
                safety_str = (
                    f'{self.safety_violations} / {self.total_safety_checks} checks '
                    f'({violation_pct:.1f}%)'
                )
            else:
                safety_str = 'No ground truth data (0 checks)'

            # Log each line separately to avoid multi-line log issues
            self.get_logger().info('===========================')
            self.get_logger().info('=== EVALUATION SUMMARY ===')
            self.get_logger().info('===========================')
            self.get_logger().info(f'Goal reached:      {"Yes" if self.goal_reached else "No"}')
            self.get_logger().info(f'Goal position:     {goal_str}')
            self.get_logger().info(f'Time elapsed:      {elapsed:.2f} seconds')
            self.get_logger().info(f'Path length:       {self.path_length:.2f} meters')
            self.get_logger().info(f'Safety violations: {safety_str}')
            self.get_logger().info('===========================')

            # Write summary and path to file in home directory
            summary = (
                '===========================\n'
                '=== EVALUATION SUMMARY ===\n'
                '===========================\n'
                f'Goal reached:      {"Yes" if self.goal_reached else "No"}\n'
                f'Goal position:     {goal_str}\n'
                f'Time elapsed:      {elapsed:.2f} seconds\n'
                f'Path length:       {self.path_length:.2f} meters\n'
                f'Safety violations: {safety_str}\n'
                '===========================\n'
                '\n'
                '=== PATH COORDINATES ===\n'
                'x,y\n'
            )
            path_lines = ''.join(
                f'{px:.6f},{py:.6f}\n' for px, py in self.path_coordinates
            )
            summary_path = os.path.expanduser('~/evaluation_summary.txt')
            with open(summary_path, 'w') as f:
                f.write(summary)
                f.write(path_lines)
            self.get_logger().info(
                f'Summary written to {summary_path} '
                f'({len(self.path_coordinates)} path points)'
            )

            self.summary_printed = True
        except Exception as e:
            self.get_logger().error(f'Failed to print summary: {e}')


def main(args=None):
    rclpy.init(args=args)

    node = EvaluationNode()
    node.get_logger().info('Evaluation node startup complete, spinning...')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutdown requested, printing summary...')
        node.print_summary()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
