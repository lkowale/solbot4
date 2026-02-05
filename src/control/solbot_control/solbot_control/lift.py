#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import yaml
import os
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import paho.mqtt.client as mqtt
import time
from ament_index_python.packages import get_package_share_directory

class LiftControl(Node):
# ros2 topic pub -r 1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
    def __init__(self):
        super().__init__('lift_control')
        try:
            pkg_dir = get_package_share_directory('solbot_control')
            config_path = os.path.join(pkg_dir, 'config', 'config.yaml')
        except Exception:
            config_path = os.path.join(os.path.dirname(__file__), 'config.yaml')
        with open(config_path, 'r') as file:
            self.yaml_params = yaml.safe_load(file)
        # print(self.yaml_params)
        # self.yaml_params['steer']['max_left']
        self.set_pos_sub = self.create_subscription(Int32,'lift/set_position',self.listener_callback,10)
        self.lift_cmd_sub = self.create_subscription(String,'lift/cmd',self.lift_cmd_clb,10)
        self.broker_address= self.declare_parameter("broker_ip_address", 't630').value
        self.mqttclient = mqtt.Client(client_id="lift_outer_bridge", userdata=None, protocol=mqtt.MQTTv5)
        self.mqttclient.username_pw_set(username="mark", password="pass")
        self.mqttclient.on_disconnect = self.on_disconnect
        self.mqttclient.connect(self.broker_address, keepalive=0)


    def on_disconnect(self, client, userdata, flags, reason_code, properties):
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

    def on_connect(client, userdata, flags, reason_code, properties):
        if reason_code.is_failure:
            print(f"Failed to connect: {reason_code}. loop_forever() will retry connection")
        else:
            # we should always subscribe from on_connect callback to be sure
            # our subscribed is persisted across reconnections.
            client.subscribe("$SYS/#")

    def listener_callback(self, msg):
        lift_posiiton = msg.data
        msg = "{\"data\": " + str(lift_posiiton) + "}"
        self.mqttclient.publish('lift/set_position', msg)
        # self.get_logger().info('I heard: "%f"' % msg.angular.z)

    def lift_cmd_clb(self, msg):
        command = msg.data
        match command:
            case 'lift_up':
                lift_posiiton = self.yaml_params['lift']['up_position']
                msg = "{\"data\": " + str(lift_posiiton) + "}"
                self.mqttclient.publish('lift/set_position', msg)
            case 'lift_down':
                lift_posiiton = self.yaml_params['lift']['down_position']
                msg = "{\"data\": " + str(lift_posiiton) + "}"
                self.mqttclient.publish('lift/set_position', msg) 

def main(args=None):
    rclpy.init(args=args)

    lift_control = LiftControl()

    rclpy.spin(lift_control)

    lift_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()