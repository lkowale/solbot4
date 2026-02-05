#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import yaml
import os
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import paho.mqtt.client as mqtt
import math
import time
from ament_index_python.packages import get_package_share_directory

class DriveControl(Node):
# ros2 topic pub -r 1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
    def __init__(self):
        super().__init__('drive_control')
        # Try package share directory first, then source directory
        try:
            pkg_dir = get_package_share_directory('solbot_control')
            config_path = os.path.join(pkg_dir, 'config', 'config.yaml')
        except Exception:
            config_path = os.path.join(os.path.dirname(__file__), 'config.yaml')
        with open(config_path, 'r') as file:
            self.yaml_params = yaml.safe_load(file)
        # print(self.yaml_params)
        # self.yaml_params['steer']['max_left']
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)

        self.broker_address= self.declare_parameter("broker_ip_address", 't630').value
        self.mqttclient = mqtt.Client(client_id="drive_outer_bridge", userdata=None, protocol=mqtt.MQTTv5)
        self.mqttclient.username_pw_set(username="mark", password="pass")
        self.mqttclient.on_disconnect = self.on_disconnect
        self.get_logger().info(f"Connecting to : {self.broker_address}")
        self.mqttclient.connect(self.broker_address, keepalive=0)     
        # self.mqttclient.subscribe("imu")        
        # self.mqttclient.on_message = self.topic_get        


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


    def listener_callback(self, msg):
        drive_cmd = msg.linear.x
        twist_cmd = msg.angular.z    
        # self.get_logger().info(f'Got msg on cmd_vel topic x:{drive_cmd} z:{twist_cmd}')
        wheels_W = self.yaml_params['wheels']['width']
        wheels_L = self.yaml_params['wheels']['length']
        wheel_d = self.yaml_params['wheels']['diameter']
        wheel_gear_ratio = self.yaml_params['wheels']['gear_ratio']
        gen_speed = drive_cmd
        #calcualte wheel rpm speed knowing its diameter and the desired speed
        # gen_speed = gen_speed / (wheel_d * math.pi) / wheel_gear_ratio 
        rps = gen_speed / (wheel_d * math.pi) #in m/s
        rpm_w = rps *60
        rpm_m = rpm_w / wheel_gear_ratio
        gen_speed = rpm_m
        abs_turn_angle_rd = abs(twist_cmd)
        F_outer_speed, F_inner_speed, R_outer_speed, R_inner_speed = 0,0,0,0
        FL_speed,FR_speed,RL_speed,RR_speed = 0, 0, 0, 0

        if(abs_turn_angle_rd>0.02):
            #use differential
            X = wheels_L/math.tan(abs_turn_angle_rd)
            R_inner_speed = gen_speed
            R_outer_speed = gen_speed * ( (X+ wheels_W)/X ) 
            F_inner_speed = gen_speed * ( math.sqrt((X*X)+(wheels_L*wheels_L))/X)
            F_outer_speed = gen_speed * ( math.sqrt( ((X+wheels_W)*(X+wheels_W))+(wheels_L*wheels_L) ) /X)
            
            if(twist_cmd>0):
              #turn left
              FL_speed = F_inner_speed
              RL_speed = R_inner_speed
              FR_speed = F_outer_speed
              RR_speed = R_outer_speed

            else:
              #turn right
              FL_speed = F_outer_speed
              RL_speed = R_outer_speed
              FR_speed = F_inner_speed
              RR_speed = R_inner_speed

        else:
           #same speed for every wheel
              FL_speed = gen_speed
              RL_speed = gen_speed
              FR_speed = gen_speed
              RR_speed = gen_speed
        
        self.send_wheel_speed("FL",FL_speed)
        self.send_wheel_speed("RL",RL_speed)
        self.send_wheel_speed("FR",FR_speed)
        self.send_wheel_speed("RR",RR_speed)
    
    def send_wheel_speed(self, wheel_name,speed):
        topic = wheel_name + "/set_speed"
        msg = "{\"data\": " + str(speed) + "}"
        self.mqttclient.publish(topic, msg)

def main(args=None):
    rclpy.init(args=args)

    drive_control = DriveControl()

    rclpy.spin(drive_control)

    drive_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()