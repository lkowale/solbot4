#!/usr/bin/env python3
"""
Origin Publisher for Mapviz
Publishes /local_xy_origin topic for swri_transform_util/Mapviz compatibility.

The origin should be at the map frame origin (where base_link starts),
NOT at the GPS antenna position. Uses Gazebo world spherical_coordinates
as the origin to align GPS with TF.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped


class OriginPublisher(Node):
    def __init__(self):
        super().__init__('origin_publisher')

        # Declare parameters for origin coordinates
        # Default values match Gazebo world spherical_coordinates (where robot spawns)
        self.declare_parameter('origin_latitude', 53.5204991)
        self.declare_parameter('origin_longitude', 17.8258532)
        self.declare_parameter('origin_altitude', 100.0)

        self.origin_lat = self.get_parameter('origin_latitude').value
        self.origin_lon = self.get_parameter('origin_longitude').value
        self.origin_alt = self.get_parameter('origin_altitude').value

        # QoS profile for local_xy_origin - TRANSIENT_LOCAL so late subscribers get the message
        origin_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publish local_xy_origin for swri_transform_util/Mapviz
        self.origin_pub = self.create_publisher(
            PoseStamped,
            '/local_xy_origin',
            origin_qos
        )

        # Timer to publish origin at 1 Hz
        self.timer = self.create_timer(1.0, self.publish_origin)

        self.get_logger().info(
            f'Origin Publisher started with origin: '
            f'lat={self.origin_lat:.6f}, lon={self.origin_lon:.6f}, alt={self.origin_alt:.2f}'
        )

    def publish_origin(self):
        # Publish origin as PoseStamped
        origin_msg = PoseStamped()
        origin_msg.header.stamp = self.get_clock().now().to_msg()
        origin_msg.header.frame_id = 'map'

        # Position is the lat/lon encoded - swri_transform_util will decode it
        # Store lat/lon in position for swri compatibility
        origin_msg.pose.position.x = self.origin_lon
        origin_msg.pose.position.y = self.origin_lat
        origin_msg.pose.position.z = self.origin_alt

        # Identity quaternion
        origin_msg.pose.orientation.w = 1.0
        origin_msg.pose.orientation.x = 0.0
        origin_msg.pose.orientation.y = 0.0
        origin_msg.pose.orientation.z = 0.0

        self.origin_pub.publish(origin_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OriginPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
