#!/usr/bin/env python3
"""
IMU Visualizer for Mapviz
Subscribes to IMU data and publishes it as Odometry for visualization in Mapviz
Shows robot orientation as arrows on the map
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf2_ros import TransformException


class IMUVisualizer(Node):
    def __init__(self):
        super().__init__('imu_visualizer')

        # Parameters
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('output_topic', '/imu/odometry')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('publish_rate', 10.0)

        imu_topic = self.get_parameter('imu_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        publish_rate = self.get_parameter('publish_rate').value

        # TF2 buffer and listener for frame transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribe to IMU
        self.imu_sub = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            10
        )

        # Publisher for odometry (Mapviz can visualize this)
        self.odom_pub = self.create_publisher(
            Odometry,
            output_topic,
            10
        )

        # Store latest IMU data
        self.latest_imu = None

        # Timer to publish at fixed rate
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_visualization)

        self.get_logger().info(f'IMU Visualizer started')
        self.get_logger().info(f'Subscribing to: {imu_topic}')
        self.get_logger().info(f'Publishing to: {output_topic}')
        self.get_logger().info(f'Target frame: {self.target_frame}')

    def imu_callback(self, msg):
        """Store latest IMU message"""
        self.latest_imu = msg

    def publish_visualization(self):
        """Publish IMU data as Odometry for visualization"""
        if self.latest_imu is None:
            return

        try:
            # Get transform from IMU frame to target frame (map)
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.latest_imu.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            # Create Odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = self.target_frame
            odom_msg.child_frame_id = self.latest_imu.header.frame_id

            # Set position from transform
            odom_msg.pose.pose.position.x = transform.transform.translation.x
            odom_msg.pose.pose.position.y = transform.transform.translation.y
            odom_msg.pose.pose.position.z = transform.transform.translation.z

            # Set orientation from IMU
            odom_msg.pose.pose.orientation = self.latest_imu.orientation

            # Set orientation covariance from IMU
            odom_msg.pose.covariance[21] = self.latest_imu.orientation_covariance[0]
            odom_msg.pose.covariance[22] = self.latest_imu.orientation_covariance[1]
            odom_msg.pose.covariance[23] = self.latest_imu.orientation_covariance[2]
            odom_msg.pose.covariance[27] = self.latest_imu.orientation_covariance[3]
            odom_msg.pose.covariance[28] = self.latest_imu.orientation_covariance[4]
            odom_msg.pose.covariance[29] = self.latest_imu.orientation_covariance[5]
            odom_msg.pose.covariance[33] = self.latest_imu.orientation_covariance[6]
            odom_msg.pose.covariance[34] = self.latest_imu.orientation_covariance[7]
            odom_msg.pose.covariance[35] = self.latest_imu.orientation_covariance[8]

            # Set angular velocity from IMU
            odom_msg.twist.twist.angular = self.latest_imu.angular_velocity

            # Publish
            self.odom_pub.publish(odom_msg)

        except TransformException as e:
            # If transform fails, publish in IMU's own frame
            odom_msg = Odometry()
            odom_msg.header = self.latest_imu.header
            odom_msg.child_frame_id = self.latest_imu.header.frame_id

            # Position at origin (no transform available)
            odom_msg.pose.pose.position.x = 0.0
            odom_msg.pose.pose.position.y = 0.0
            odom_msg.pose.pose.position.z = 0.0

            # Orientation from IMU
            odom_msg.pose.pose.orientation = self.latest_imu.orientation

            # Angular velocity from IMU
            odom_msg.twist.twist.angular = self.latest_imu.angular_velocity

            # Publish
            self.odom_pub.publish(odom_msg)

            if not hasattr(self, '_transform_warned'):
                self.get_logger().warn(
                    f'No transform from {self.latest_imu.header.frame_id} to {self.target_frame}. '
                    f'Publishing in IMU frame. Error: {str(e)}'
                )
                self._transform_warned = True


def main(args=None):
    rclpy.init(args=args)
    node = IMUVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
