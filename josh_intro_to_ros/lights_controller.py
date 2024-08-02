#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ManualControl,OverrideRCIn
from std_msgs.msg import Bool

class LightsController(Node):

    def __init__(self):
        super().__init__("lights_controller")

        self.lights_subscriber = self.create_subscription(
            Bool,
            "bluerov2/lights_control",
            self.lightsCallback,
            10

        )

        self.command_pub = self.create_publisher(
            OverrideRCIn,
            "bluerov2/override_rc",
            10
        )

        self.get_logger().info("STARTING")


    def lightsCallback(self,msg):
        # self.get_logger().info("SDGFSDGSDGS")
        # self.get_logger().info(msg)
        if (msg.data):
            self.turn_lights_on(99)
        else:
            self.turn_lights_on(0)

    def turn_lights_on(self,level):

        self.get_logger().info(f"Turning lights on to level {level}")
        commands = OverrideRCIn()
        commands.channels = [OverrideRCIn.CHAN_NOCHANGE] * 10
        commands.channels[8] = 1000 + level * 10
        commands.channels[9] = 1000 + level * 10

        self.command_pub.publish(commands)


def main(args = None):
    rclpy.init(args=args)
    node = LightsController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()