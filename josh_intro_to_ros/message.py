#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ManualControl,Altitude
from std_msgs.msg import Int16, Float64

class Message(Node):

    x = None
    y = None
    z = None
    r = None
    manualControl = None

    def __init__(self):
        super().__init__("Message")

        self.x_sub = self.create_subscription(
            Float64,
            "bluerov2/x",
            self.xCallback,
            10
        )

        self.y_sub = self.create_subscription(
            Float64,
            "bluerov2/y",
            self.yCallback,
            10
        )

        self.z_sub = self.create_subscription(
            Float64,
            "bluerov2/z",
            self.zCallback,
            10
        )

        self.r_sub = self.create_subscription(
            Float64,
            "bluerov2/r",
            self.rCallback,
            10
        )

        self.manual_control_sub = self.create_subscription(
            ManualControl,
            "bluerov2/manual_control",
            self.manualControlCallback,
            10
        )

        self.manual_control_pub = self.create_publisher(
            ManualControl,
            "bluerov2/manual_control",
            10
        )

        self.timer = self.create_timer(0.5, self.timerCallback)

    def xCallback(self,msg):
        self.x = msg.data

    def yCallback(self,msg):
        self.y = msg.data

    def zCallback(self,msg):
        self.z = msg.data

    def rCallback(self,msg):
        self.r = msg.data

    def manualControlCallback(self,msg):
        self.manualControl = msg

    def timerCallback(self):
        if (self.manualControl is None):
            self.manualControl = ManualControl()
        if (self.x != 0.0 and self.x is not None):
            self.manualControl.x = float(self.x)
        if (self.y != 0.0 and self.y is not None):
            self.manualControl.y = float(self.y)
        if (self.z != 0.0 and self.z is not None):
            self.manualControl.z = float(self.z)
        if (self.r != 0.0 and self.r is not None):
            self.manualControl.r = float(self.r)
        self.get_logger().info(f"PUBLISHING: {self.manualControl.x}, {self.manualControl.y}, {self.manualControl.z}, {self.manualControl.r}")
        self.manual_control_pub.publish(self.manualControl)

def main(args = None):
    rclpy.init(args = args)
    node = Message()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=="__main__":
    main()