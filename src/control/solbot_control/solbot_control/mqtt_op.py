#!/usr/bin/env python3

import rclpy
from rclpy.time import Time
from rclpy.timer import Timer

import time
import paho.mqtt.client as mqtt
from rclpy.node import Node
import uuid
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
#from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from std_msgs.msg import Int32, Float32, String, Bool
from sensor_msgs.msg import Imu
from geometry_msgs.msg import (Quaternion, Vector3)
from tf2_ros import TransformBroadcaster
import json
from json.decoder import JSONDecodeError
from opennav_coverage_msgs.msg import JobPaused

class MQTT_op(Node):
    def __init__(self):
        super().__init__('mqtt_op_bridge')
        # cmd_pub_timer_period = 0.5  # seconds
        # self.cmd_pub_timer = Timer(self.cmd_pub_timer_callback, callback_group='',clock='', timer_period_ns=cmd_pub_timer_period*1000000000)
        # self.cmd_pub_timer = self.create_timer(timer_period, self.cmd_pub_timer_callback)       

        self.rate = 10
        self.r = self.create_rate(self.rate)
        # self.mqttclient = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, "mqtt_op_bridge")
        # Generate unique client ID to prevent conflicts
        unique_id = str(uuid.uuid4())[:8]
        client_id = f"mqtt_op_bridge_{unique_id}"
        self.mqttclient = mqtt.Client(client_id=client_id, userdata=None, protocol=mqtt.MQTTv5)
        self.get_logger().info(f"MQTT Client ID: {client_id}")
        self.mqttclient.on_connect = self.on_connect
        self.mqttclient.on_disconnect = self.on_disconnect
        self.mqttclient.tls_set(tls_version=mqtt.ssl.PROTOCOL_TLS)
        self.mqttclient.username_pw_set(username="aargideon", password="para!234")
        self.mqttclient.connect("ae4cb1b10ad84e53af8887dd32476b04.s2.eu.hivemq.cloud",8883)                  
        self.mqttclient.subscribe("outer/#")        
        self.mqttclient.on_message = self.topic_get  

        self.get_logger().info('mqtt_op_bridge started...')
        self.steer = 0.0
        self.speed = 0.0
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)           
        self.lift_pos_publisher = self.create_publisher(Int32, 'lift/set_position', 10) 
        self.lift_cmd_publisher = self.create_publisher(String, 'lift/cmd', 10)           
        self.job_paused_publisher = self.create_publisher(JobPaused, 'job_paused', 10)    
        self.planter_cmd_publisher = self.create_publisher(String, 'planter/cmd', 10)     
        self.planter_dd_publisher = self.create_publisher(Int32, 'planter/set_dd', 10)
        self.planter_aospr_publisher = self.create_publisher(Int32, 'planter/set_aospr', 10)
        # self.tf_broadcaster = TransformBroadcaster(self)

        self.mqttclient.loop_start()

    def topic_get(self, client, userdata, msg):
        topic = msg.topic     

        if topic == "outer/steer":
            message = msg.payload.decode("utf-8")
            msg_in=json.loads(message)
            self.steer =  float(msg_in["data"])
            self.publish_cmd_vel_msg()

        if topic == "outer/speed":
            message = msg.payload.decode("utf-8")
            msg_in=json.loads(message)
            self.speed =  float(msg_in["data"])
            self.publish_cmd_vel_msg()

        if topic == "outer/lift_pos":
            message = msg.payload.decode("utf-8")
            msg_in=json.loads(message)
            lift_pos =  int(msg_in["data"])
            out_msg = Int32()
            out_msg.data = lift_pos
            self.lift_pos_publisher.publish(out_msg)      

        if topic == "outer/lift_cmd":
            message = msg.payload.decode("utf-8")
            msg_in=json.loads(message)
            lift_cmd =  msg_in["data"]
            out_msg = String()
            out_msg.data = lift_cmd
            self.lift_cmd_publisher.publish(out_msg)    

        if topic == "outer/job_pause":
            message = msg.payload.decode("utf-8")
            msg_in=json.loads(message)
            pause_cmd =  int(msg_in["data"])
            out_msg = JobPaused()   
            out_msg.data = bool(pause_cmd)
            out_msg.reason = "User triggered"
            self.job_paused_publisher.publish(out_msg)  

        if topic == "outer/planter_cmd":
            message = msg.payload.decode("utf-8")
            msg_in=json.loads(message)
            planter_cmd =  msg_in["data"]
            out_msg = String()
            out_msg.data = planter_cmd
            self.planter_cmd_publisher.publish(out_msg)    

        if topic == "outer/planter_aospr":
            message = msg.payload.decode("utf-8")
            msg_in=json.loads(message)
            aospr =  int(msg_in["data"])
            out_msg = Int32()
            out_msg.data = aospr
            self.planter_aospr_publisher.publish(out_msg)

        if topic == "outer/planter_dd":
            message = msg.payload.decode("utf-8")
            msg_in=json.loads(message)
            dd =  int(msg_in["data"])
            out_msg = Int32()
            out_msg.data = dd
            self.planter_dd_publisher.publish(out_msg)
            
    def publish_cmd_vel_msg(self):
        cmd_msg = Twist()
        cmd_msg.linear.x = self.speed
        cmd_msg.angular.z = self.steer
            
        #self.get_logger().info('Publishing imu message')
        #self.pub_imu_raw.publish(imu_raw_msg)
        self.cmd_vel_publisher.publish(cmd_msg)

    def on_connect(self, client, userdata, flags, rc, properties=None):
        # print("CONNACK received with code %s." % rc)
        if rc == 0:
            self.get_logger().info("Successfully connected to HiveMQ cloud broker", once=True)
        else:
            self.get_logger().error(f"Connection to hivemq failed with code: {rc}")

    def on_disconnect(self, client, userdata, flags, reason_code, properties=None):
        FIRST_RECONNECT_DELAY = 1
        RECONNECT_RATE = 2
        MAX_RECONNECT_COUNT = 12
        MAX_RECONNECT_DELAY = 60
        # self.get_logger().info(f'My log message {num}', once=True)
        self.get_logger().info(f"Disconnected with result code: {reason_code}")
        # logging.info("Disconnected with result code: %s", rc)
        logging = self.get_logger()
        reconnect_count, reconnect_delay = 0, FIRST_RECONNECT_DELAY
        while reconnect_count < MAX_RECONNECT_COUNT:
            self.get_logger().info(f"Reconnecting in {reconnect_delay} seconds...")
            time.sleep(reconnect_delay)

            try:
                client.reconnect()
                logging.info("Reconnected successfully!")
                return
            except Exception as err:
                logging.error(f"{err} Reconnect failed. Retrying...")

            reconnect_delay *= RECONNECT_RATE
            reconnect_delay = min(reconnect_delay, MAX_RECONNECT_DELAY)
            reconnect_count += 1
        logging.info(f"Reconnect failed after {reconnect_count} attempts. Exiting...")

    def destroy_node(self):
        """Cleanup when node is destroyed"""
        self.mqttclient.loop_stop()
        self.mqttclient.disconnect()
        super().destroy_node()


def main(args=None):
    

    rclpy.init(args=args)
    try:
        relay_ros2_mqtt = MQTT_op()
        rclpy.spin(relay_ros2_mqtt)
    except rclpy.exceptions.ROSInterruptException:
        pass

    relay_ros2_mqtt.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
