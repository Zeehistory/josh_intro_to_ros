#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ManualControl,Altitude
from std_msgs.msg import Int16

class Controller(Node):

    tagHeading = None
    laneHeading = None
    heading = None

    def __init__(self):
        super().__init__("controller")

        self.tag_subscriber = self.create_subscription(
            Int16,
            "bluerov2/tag_desired_heading",
            self.tagCallback,
            10
        )

        self.lane_subscriber = self.create_subscription(
            Int16,
            "bluerov2/lane_desired_heading",
            self.laneCallback,
            10
        )

        self.heading_publisher = self.create_publisher(
            Int16,
            "bluerov2/desired_heading",
            10
        )

        self.heading_subscriber = self.create_subscription(
            Int16,
            "bluerov2/heading",
            self.headingCallback,
            10
        )

        self.forward_publisher = self.create_publisher(
            ManualControl,
            "bluerov2/manual_control",
            10
        )

        self.depth_publisher = self.create_publisher(
            Altitude,
            "bluerov2/desired_depth",
            10
        )

        self.timer = self.create_timer(
            0.5,self.timerCallback
        )

    def tagCallback(self,msg):
        self.tagHeading = msg

    def laneCallback(self,msg):
        self.laneHeading = msg

    def headingCallback(self,msg):
        self.heading = msg
    
    def timerCallback(self):
        self.depth_publisher.publish(Altitude(local = 0.5))
        if (self.tagHeading is not None):
            self.heading_publisher.publish(self.tagHeading)
            self.forward_publisher.publish(ManualControl(x=100))
            self.get_logger().info(f"PUBLISHING TAG SUBSCRIBER: {self.tagHeading.data}")
            return
        if (self.laneHeading is not None):
            self.heading_publisher.publish(self.laneHeading)
            self.forward_publisher.publish(ManualControl(x=100))
            self.get_logger().info(f"PUBLISHING LANE SUBSCRIBER: {self.laneHeading.data}")
        
        else:
            if (self.heading is not None):
                self.heading_publisher.publish(Int16(data = self.heading.data + 90))

def main(args = None):
    rclpy.init(args = args)
    node = Controller()
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
            