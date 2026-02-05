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
from nav_msgs.msg import Odometry  # Add this import
from opennav_coverage_msgs.srv import RotatePlanter
from ament_index_python.packages import get_package_share_directory

class PlanterControl(Node):
# ros2 topic pub -r 1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
    def __init__(self):
        super().__init__('planter_control')
        try:
            pkg_dir = get_package_share_directory('solbot_control')
            config_path = os.path.join(pkg_dir, 'config', 'config.yaml')
        except Exception:
            config_path = os.path.join(os.path.dirname(__file__), 'config.yaml')
        with open(config_path, 'r') as file:
            self.yaml_params = yaml.safe_load(file)
        
        self.broker_address= self.declare_parameter("broker_ip_address", 't630').value
        self.mqttclient = mqtt.Client(client_id="planter_control", userdata=None, protocol=mqtt.MQTTv5)
        self.mqttclient.username_pw_set(username="mark", password="pass")
        self.mqttclient.on_disconnect = self.on_disconnect
        self.mqttclient.connect(self.broker_address, keepalive=0)

        # Add odometry subscriber and robot_speed attribute
        #in m/s?
        self.robot_speed = 0.0
        self.odom_sub = self.create_subscription(Odometry, 'odometry/global', self.odom_callback,10)
        self.planter_cmd_sub = self.create_subscription(String,'planter/cmd',self.planter_cmd_clb,10)
        self.planter_set_aospr_sub = self.create_subscription(Int32,'planter/set_aospr',self.planter_set_aospr_clb,10)
        self.planter_set_dd_sub = self.create_subscription(Int32,'planter/set_dd',self.planter_set_dd_clb,10)

        self.rotate_cilent = self.create_client(RotatePlanter, 'rotate_planter')
        self.is_planter_active = False
        # seeds per meter
        self.desired_density = self.yaml_params['planter']['desired_density']
        # amount of seeds per rotation
        self.aospr = self.yaml_params['planter']['aospr']

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

    def planter_cmd_clb(self, msg):
        command = msg.data
        # self.get_logger().info(f"got message {command} on planter/cmd topic")
        match command:
            case 'start':
                self.is_planter_active = True
            case 'stop':
                self.is_planter_active = False
                planter_rpm = 0.0
                msg = "{\"data\": " + str(planter_rpm) + "}"
                self.mqttclient.publish('PLANTER/set_speed', msg)
            case 'rotate_once':
                self.rotate_once()

    def planter_set_aospr_clb(self, msg):
        self.aospr = int(msg.data)
        self.get_logger().info(f"Set planter AOSPR to {self.aospr}")

    def planter_set_dd_clb(self, msg):
        self.desired_density = int(msg.data)
        self.get_logger().info(f"Set planter desired density to {self.desired_density}")

    # Add odometry callback
    def odom_callback(self, msg):
        self.robot_speed = msg.twist.twist.linear.x
        if(self.is_planter_active):
            if(self.robot_speed > 0.1):
            # if(self.is_planter_active):
                # Calculate the speed for the planter based on the desired density and robot speed
                s = self.robot_speed
                aos = s * self.desired_density
                rps = aos / self.aospr
                planter_rpm = rps * 60
                # planter_rpm = 100/6 * self.desired_density * self.robot_speed / self.aospr
            else:
                planter_rpm = 0.0
            msg = "{\"data\": " + str(planter_rpm) + "}"
            self.mqttclient.publish('PLANTER/set_speed', msg)
        
    def rotate_once(self):
        if not self.rotate_cilent.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service not available, waiting...')
            return
        request = RotatePlanter.Request()
        request.servo_name = 'PLANTER'
        request.revolutions = 1
        future = self.rotate_cilent.call_async(request)
        # rclpy.spin_until_future_complete(self, future)
        # response = future.result()

def main(args=None):
    rclpy.init(args=args)

    planter_control = PlanterControl()

    rclpy.spin(planter_control)

    planter_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()