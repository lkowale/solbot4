#!/usr/bin/env python3

import rclpy
from rclpy.time import Time

from time import sleep
import sys
import threading
import numpy as np
import os
import yaml
import paho.mqtt.client as mqtt
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.qos import qos_profile_services_default
from rclpy import qos
from rclpy.qos import QoSProfile
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
#from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from std_msgs.msg import Int32, Float32, String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import (Quaternion, Vector3)
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply
import json
from json.decoder import JSONDecodeError

class RelayRos2Mqtt(Node):
    def __init__(self):
        super().__init__('imu_bridge')

        self.sleep_rate = 0.025
        self.rate = 10
        self.r = self.create_rate(self.rate)

        # Configuration file path - try package share directory first
        try:
            from ament_index_python.packages import get_package_share_directory
            package_dir = get_package_share_directory('solbot_control')
        except Exception:
            package_dir = os.path.dirname(os.path.dirname(__file__))
        self.config_file = os.path.join(package_dir, 'config', 'imu_bridge.yaml')

        # Load yaw offset from config file
        self.yaw_offset = 0.0
        self.load_config()

        # Subscribe to yaw offset updates
        self.yaw_offset_subscriber = self.create_subscription(
            Float32,
            'imu_yaw_offset',
            self.yaw_offset_callback,
            10
        )

        self.broker_address= self.declare_parameter("~broker_ip_address", 't630').value
        # self.mqttclient = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, "sol")
        self.mqttclient = mqtt.Client(client_id="imu_bridge", userdata=None, protocol=mqtt.MQTTv5)
        self.mqttclient.username_pw_set(username="mark", password="pass")
        self.mqttclient.connect(self.broker_address)
        self.mqttclient.subscribe("imu")
        self.mqttclient.on_message = self.topic_get

        self.get_logger().info(f'imu_bridge started with yaw_offset: {self.yaw_offset} rad ({np.degrees(self.yaw_offset):.2f} deg)')

        self.imu_publisher = self.create_publisher(Imu, 'imu', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Start MQTT loop in a separate thread to allow ROS2 to spin
        self.mqttclient.loop_start()

    def __del__(self):
        """Cleanup MQTT connection when node is destroyed"""
        try:
            self.mqttclient.loop_stop()
            self.mqttclient.disconnect()
        except:
            pass

    def topic_get(self, client, userdata, msg):
        topic = msg.topic
        #self.get_logger().info(f'got message on topic {topic}')

        if topic == "imu":
            message = msg.payload.decode("utf-8")
            msg_in=json.loads(message)
            self.publish_imu_message(msg_in)
            #self.publish_tf(msg_in)

    def load_config(self):
        """Load configuration from YAML file"""
        try:
            if os.path.exists(self.config_file):
                with open(self.config_file, 'r') as f:
                    config = yaml.safe_load(f)
                    self.yaw_offset = config.get('yaw_offset', 0.0)
                    self.get_logger().info(f'Loaded yaw_offset from config: {self.yaw_offset} rad ({np.degrees(self.yaw_offset):.2f} deg)')
            else:
                self.get_logger().warn(f'Config file not found: {self.config_file}. Using default yaw_offset: 0.0')
                self.save_config()  # Create default config file
        except Exception as e:
            self.get_logger().error(f'Error loading config: {e}')
            self.yaw_offset = 0.0

    def save_config(self):
        """Save current configuration to YAML file"""
        try:
            config = {'yaw_offset': float(self.yaw_offset)}
            os.makedirs(os.path.dirname(self.config_file), exist_ok=True)
            with open(self.config_file, 'w') as f:
                yaml.dump(config, f, default_flow_style=False)
            self.get_logger().info(f'Saved yaw_offset to config: {self.yaw_offset} rad ({np.degrees(self.yaw_offset):.2f} deg)')
        except Exception as e:
            self.get_logger().error(f'Error saving config: {e}')

    def yaw_offset_callback(self, msg):
        """Callback for yaw offset updates via ROS2 topic"""
        self.yaw_offset = msg.data
        self.get_logger().info(f'Updated yaw_offset: {self.yaw_offset} rad ({np.degrees(self.yaw_offset):.2f} deg)')
        self.save_config()

    def apply_yaw_offset(self, quat_in):
        """Apply yaw offset to quaternion

        Args:
            quat_in: Input quaternion [x, y, z, w]

        Returns:
            Modified quaternion with yaw offset applied [x, y, z, w]
        """
        # Convert quaternion to euler angles
        euler = euler_from_quaternion(quat_in)
        roll, pitch, yaw = euler

        # Apply yaw offset
        yaw_adjusted = yaw + self.yaw_offset

        # Convert back to quaternion
        quat_out = quaternion_from_euler(roll, pitch, yaw_adjusted)

        return quat_out

    def publish_tf(self, msg_in):

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'imu_link'
        # t.child_frame_id = 'imu_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # Get original quaternion and apply yaw offset
        quat_original = [float(i) for i in msg_in["orientation"]]
        quat_adjusted = self.apply_yaw_offset(quat_original)

        quat = Quaternion()
        quat.x, quat.y, quat.z, quat.w = quat_adjusted
        t.transform.rotation = quat
        self.tf_broadcaster.sendTransform(t)     

    def publish_imu_message(self, msg_in):

        #try:
        imu_msg = Imu()
        # data = self.sensor.getMotion6()
        # yaw, pitch, roll, x_accel, y_accel, z_accel = rvc.heading

        gyro = Vector3()
        gyro.x, gyro.y, gyro.z = [float(i) for i in msg_in["angular_velocity"]]

        imu_msg.angular_velocity = gyro
        imu_msg.angular_velocity_covariance[0] = 0.00001
        imu_msg.angular_velocity_covariance[4] = 0.00001
        imu_msg.angular_velocity_covariance[8] = 0.00001

        accel = Vector3()
        accel.x, accel.y, accel.z = [float(i) for i in msg_in["linear_acceleration"]]
        imu_msg.linear_acceleration = accel
        imu_msg.linear_acceleration_covariance[0] = 0.00001
        imu_msg.linear_acceleration_covariance[4] = 0.00001
        imu_msg.linear_acceleration_covariance[8] = 0.00001

        # Get original quaternion and apply yaw offset
        quat_original = [float(i) for i in msg_in["orientation"]]
        quat_adjusted = self.apply_yaw_offset(quat_original)

        quat = Quaternion()
        quat.x, quat.y, quat.z, quat.w = quat_adjusted
        #print(f'The quaternion representation is x: {quat}.')
        imu_msg.orientation = quat

        imu_msg.orientation_covariance[0] = 0.00001
        imu_msg.orientation_covariance[4] = 0.00001
        imu_msg.orientation_covariance[8] = 0.00001

        # add header
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        #self.get_logger().info('Publishing imu message')
        #self.pub_imu_raw.publish(imu_raw_msg)
        self.imu_publisher.publish(imu_msg)

def main(args=None):
    

    rclpy.init(args=args)
    try:
        relay_ros2_mqtt = RelayRos2Mqtt()
        rclpy.spin(relay_ros2_mqtt)
    except rclpy.exceptions.ROSInterruptException:
        pass

    relay_ros2_mqtt.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()