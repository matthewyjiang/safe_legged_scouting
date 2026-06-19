#!/usr/bin/env python3

import math
import os
import random

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from trusses_custom_interfaces.msg import SpatialMeasurement


class TurtlesimSpatialPublisher(Node):

    def __init__(self):
        super().__init__('spirit_spatial_publisher')

        # Load terrain data
        self.load_terrain_data()

        # Publisher for spatial measurements
        self.spatial_pub = self.create_publisher(
            SpatialMeasurement,
            'spirit/spatial_measurements',
            10
        )

        # Subscriber to spirit pose
        self.pose_sub = self.create_subscription(
            Pose,
            'spirit/current_pose',
            self.pose_callback,
            10
        )

        # Store current pose
        self.current_pose = None
        self.initial_points_published = False

        # Timer for publishing spatial measurements
        self.timer = self.create_timer(1, self.publish_spatial_measurement)

        self.get_logger().info('Spirit Spatial Publisher initialized')

    def load_terrain_data(self):
        """Load terrain data from CSV file."""
        try:
            # Use ament_index to find package share directory
            from ament_index_python.packages import get_package_share_directory
            package_share_dir = get_package_share_directory(
                'safe_bayesian_optimization')
            terrain_file = os.path.join(
                package_share_dir, 'data', 'terrain.csv')

            # Load terrain data using numpy
            self.terrain_data = np.loadtxt(terrain_file, delimiter=',', skiprows=1)

            # Report original terrain statistics
            orig_min = np.min(self.terrain_data)
            orig_max = np.max(self.terrain_data)
            orig_avg = np.mean(self.terrain_data)
            self.get_logger().info(
                f'Loaded terrain data with shape: {self.terrain_data.shape}')
            self.get_logger().info(
                f'Original terrain stats - Min: {orig_min:.3f}, '
                f'Max: {orig_max:.3f}, Average: {orig_avg:.3f}')

            # Terrain data shape and scaling factor
            self.terrain_rows, self.terrain_cols = self.terrain_data.shape
            self.scale_factor = 10

            # Terrain coordinate bounds (centered around 0,0)
            self.terrain_x_min, self.terrain_x_max = - \
                float(self.terrain_cols)/2, float(self.terrain_cols)/2
            self.terrain_y_min, self.terrain_y_max = - \
                float(self.terrain_rows)/2, float(self.terrain_rows)/2

        except Exception as e:
            self.get_logger().error(f'Failed to load terrain data: {e}')
            self.terrain_data = None

    def get_terrain_value(self, x, y):
        """Get terrain value at given x, y coordinates with scaling."""
        if self.terrain_data is None:
            return 100.0  # Default value

        # Scale pose coordinates up by scale_factor to map to terrain coordinates
        terrain_x = x * self.scale_factor
        terrain_y = y * self.scale_factor

        # Convert terrain coordinates to array indices
        col = int((terrain_x - self.terrain_x_min) / (self.terrain_x_max -
                  self.terrain_x_min) * (self.terrain_cols - 1))
        row = int((terrain_y - self.terrain_y_min) / (self.terrain_y_max -
                  self.terrain_y_min) * (self.terrain_rows - 1))

        # Clamp to valid indices
        col = max(0, min(col, self.terrain_cols - 1))
        row = max(0, min(row, self.terrain_rows - 1))

        # Scale terrain values to 0-10 range
        terrain_value = float(self.terrain_data[row, col])
        # Normalize to 0-1 range first, then scale to 0-10
        min_val = np.min(self.terrain_data)
        max_val = np.max(self.terrain_data)
        normalized = (terrain_value - min_val) / (max_val - min_val)
        return normalized * 10.0

    def pose_callback(self, msg):
        """Store the current spirit pose."""
        self.current_pose = msg

        # Publish initial random points around robot on first pose received
        if not self.initial_points_published:
            self.publish_initial_random_points()
            self.initial_points_published = True

    def publish_initial_random_points(self):
        """Publish random spatial measurements around the robot's initial position."""
        if self.current_pose is None:
            return

        num_points = 30  # Number of random points to publish
        max_distance = 1  # Maximum distance from robot

        for i in range(num_points):
            # Generate random point in vicinity of robot
            angle = random.uniform(0, 2 * math.pi)
            distance = random.uniform(0.0, max_distance)

            # Calculate random point coordinates
            x = self.current_pose.position.x + distance * math.cos(angle)
            y = self.current_pose.position.y + distance * math.sin(angle)

            # Keep points within reasonable bounds for pose coordinates
            min_x = self.terrain_x_min / self.scale_factor + 0.1
            max_x = self.terrain_x_max / self.scale_factor - 0.1
            min_y = self.terrain_y_min / self.scale_factor + 0.1
            max_y = self.terrain_y_max / self.scale_factor - 0.1
            x = max(min_x, min(max_x, x))
            y = max(min_y, min(max_y, y))

            # Create and publish spatial measurement
            spatial_msg = SpatialMeasurement()
            spatial_msg.position.x = x
            spatial_msg.position.y = y
            spatial_msg.position.z = 0.0
            spatial_msg.value = self.get_terrain_value(x, y)
            spatial_msg.unit = 'm'
            spatial_msg.source_name = 'spirit_init'
            spatial_msg.uncertainty = 0.0  # Lower uncertainty for initial points
            spatial_msg.time = self.get_clock().now().to_msg()

            self.spatial_pub.publish(spatial_msg)

        self.get_logger().info(
            f'Published {num_points} initial random points around robot')

    def publish_spatial_measurement(self):
        """Publish spatial measurement based on turtle position."""
        if self.current_pose is None:
            return

        # Create spatial measurement message
        spatial_msg = SpatialMeasurement()

        # Set position from spirit pose
        spatial_msg.position.x = self.current_pose.position.x
        spatial_msg.position.y = self.current_pose.position.y
        spatial_msg.position.z = self.current_pose.position.z

        # Get terrain value from loaded data
        spatial_msg.value = self.get_terrain_value(
            self.current_pose.position.x, self.current_pose.position.y)

        spatial_msg.unit = 'm'
        spatial_msg.source_name = 'spirit'
        # Add base uncertainty (spirit pose doesn't have velocity info in geometry_msgs/Pose)
        base_uncertainty = 0.0
        spatial_msg.uncertainty = base_uncertainty

        # Set timestamp
        spatial_msg.time = self.get_clock().now().to_msg()

        # Publish the measurement
        self.spatial_pub.publish(spatial_msg)

        self.get_logger().info(
            'Published spatial measurement: '
            f'pos=({self.current_pose.position.x:.2f}, '
            f'{self.current_pose.position.y:.2f}), '
            f'value={spatial_msg.value:.2f}, '
            f'uncertainty={spatial_msg.uncertainty:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = TurtlesimSpatialPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
