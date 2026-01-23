#!/usr/bin/env python3
"""
Origin Publisher for Mapviz
Publishes /local_xy_origin topic based on robot_localization's datum
This bridges robot_localization to swri_transform_util for Mapviz compatibility
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped


class OriginPublisher(Node):
    def __init__(self):
        super().__init__('origin_publisher')

        self.origin_set = False
        self.origin_lat = None
        self.origin_lon = None
        self.origin_alt = None

        # Subscribe to GPS fix to auto-set origin from first message
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10
        )

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

        self.get_logger().info('Origin Publisher started - waiting for GPS fix')

    def gps_callback(self, msg):
        if not self.origin_set and msg.status.status >= 0:
            # Set origin from first valid GPS fix
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.origin_alt = msg.altitude
            self.origin_set = True
            self.get_logger().info(
                f'Origin set from GPS: lat={self.origin_lat:.6f}, '
                f'lon={self.origin_lon:.6f}, alt={self.origin_alt:.2f}'
            )

    def publish_origin(self):
        if self.origin_set:
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
