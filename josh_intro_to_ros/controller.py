#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import Altitude 
from std_msgs.msg import Int16

class Controller(Node):

    tagHeading = None
    laneHeading = None

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

        self.heading_publisher = self.create_subscription(
            Int16,
            "bluerov2/desired_heading",
            10
        )

        self.timer = self.create_timer(
            0.5,self.timerCallback
        )

    def tagCallback(self,msg):
        self.tagHeading = msg

    def laneCallback(self,msg):
        self.laneHeading = msg
    
    def timerCallback(self):
        if (self.tagHeading is not None):
            self.heading_publisher.publish(self.tagHeading)
            self.get_logger().info(f"PUBLISHING TAG SUBSCRIBER: {self.tagHeading.data}")
            return
        if (self.laneHeading is not None):
            self.heading_publisher.publish(self.laneHeading)
            self.get_logger().info(f"PUBLISHING LANE SUBSCRIBER: {self.laneHeading.data}")