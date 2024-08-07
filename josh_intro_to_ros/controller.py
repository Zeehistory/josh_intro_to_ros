#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ManualControl,Altitude
from std_msgs.msg import Int16, Float64

class Controller(Node):

    tagHeading = None
    laneHeading = None
    heading = None
    orig_heading = None
    step = 0
    called = False

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
            Float64,
            "bluerov2/x",
            10
        )

        self.rotate_publisher = self.create_publisher(
            Float64,
            "bluerov2/r",
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
        if (not self.called):
            self.orig_heading = msg.data
            self.called = True
    
    def timerCallback(self):
        self.depth_publisher.publish(Altitude(local = 0.4))

        if (self.tagHeading is not None and self.tagHeading.data != 0.0):
            self.heading_publisher.publish(self.tagHeading)
            self.forward_publisher.publish(Float64(data=100.0))
            self.get_logger().info(f"PUBLISHING TAG SUBSCRIBER: {self.tagHeading.data}")
            return
        if (self.laneHeading is not None and self.laneHeading.data != 0.0):
            self.heading_publisher.publish(self.laneHeading)
            self.forward_publisher.publish(Float64(data=100.0))
            self.get_logger().info(f"PUBLISHING LANE SUBSCRIBER: {self.laneHeading.data}")
        else:
            if (self.heading is not None):
                self.get_logger().info(f"LOST")
                #self.heading_publisher.publish(Int16(data = self.heading.data))
                #if (int(self.step/20) % 2 == 0):
                self.get_logger().info(f"ORIGINAL HEADING: {self.orig_heading}")
                if ((self.step-30)%30 > 25):
                    if (int(self.step/30)%2 == 0):
                        self.heading_publisher.publish(Int16(data = self.orig_heading + 180))
                    else:
                        self.heading_publisher.publish(Int16(data = self.orig_heading))
                    self.forward_publisher.publish(Float64(data = 0.1))
                else:
                    self.forward_publisher.publish(Float64(data = 25.0))
                    self.rotate_publisher.publish(Float64(data = 0.1))

                self.step += 0.5
                #     if (int(self.step/10) % 2 == 0):
                #         self.forward_publisher.publish(Float64(data = 25.0))
                #         self.rotate_publisher.publish(Float64(data = 0.1))
                #     else:
                #         self.forward_publisher.publish(Float64(data = 0.1))
                #         if (int(self.step/5) % 2 == 0):
                #             self.heading_publisher.publish(Int16(data = self.heading.data + 90))
                #         else:
                #             self.heading_publisher.publish(Int16(data = self.orig_heading))
                # else:
                #     if (int(self.step/10) % 2 == 0):
                #         self.forward_publisher.publish(Float64(data = -25.0))
                #         self.rotate_publisher.publish(Float64(data = 0.1))
                #     else:
                #         self.forward_publisher.publish(Float64(data = 0.1))
                #         if (int(self.step/5) % 2 == 0):
                #             self.heading_publisher.publish(Int16(data = self.heading.data - 90))
                #         else:
                #             self.heading_publisher.publish(Int16(data = self.orig_heading-180))


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
            