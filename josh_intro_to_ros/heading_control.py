#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ManualControl
from std_msgs.msg import Int16, Float64
from sensor_msgs.msg import Imu
import numpy as np
import math

class HeadingControl(Node):

    t1 = 0
    t2 = 9
    error_accumulator = 0
    previous_error = 0
    max_integral = 1.0
    max_throttle = 100.0
    desired_heading = None
    measured_imu = None
    measured_heading = None

    def __init__(self):
        super().__init__("headingControl")
        """
        Initializes the HeadingControl node, creating subscriptions and a publisher.
        Subscriptions:
            - "bluerov2/heading": Current heading (Int16)
            - "bluerov2/imu": Angular velocity from IMU (Imu)
            - "bluerov2/desired_heading": Target heading (Int16)
        Publisher:
            - "bluerov2/r": Control signal for heading adjustment (Float64)
        """

        self.measured_heading_sub = self.create_subscription(
            Int16,
            "bluerov2/heading",
            self.measuredHeadingCallback,
            10
        )

        self.measured_imu_sub = self.create_subscription(
            Imu,
            "bluerov2/imu",
            self.measuredImuCallback, 
            10
        )

        self.desired_heading_sub = self.create_subscription(
            Int16,
            "bluerov2/desired_heading",
            self.desiredHeadingCallback,
            10
        )

        self.publisher = self.create_publisher(
            Float64,
            "bluerov2/r",
            10
        )

    def measuredHeadingCallback(self, msg):
        """
        Callback function for the "bluerov2/heading" topic.
        Updates the current measured heading from the incoming Int16 message.

        Args:
            msg (Int16): Message containing the current heading in degrees.
        """
        self.measured_heading = msg.data

    def measuredImuCallback(self, msg):
        """
        Callback function for the "bluerov2/imu" topic.
        Retrieves angular velocity from the IMU message and updates timestamps for calculations.

        Args:
            msg (Imu): IMU message containing angular velocity data.
        """
        self.measured_imu = msg.angular_velocity.z
        if (self.t1 == 0):
            self.t1 = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.t2 = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        else:
            self.t1 = self.t2
            self.t2 = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.power_calculations()

    def desiredHeadingCallback(self, msg):
        """
        Callback function for the "bluerov2/desired_heading" topic.
        Updates the target desired heading from the incoming Int16 message.

        Args:
            msg (Int16): Message containing the desired heading in degrees.
        """
        self.desired_heading = msg.data
        self.get_logger().info(f"\nDesired Heading: {msg.data}")

    def power_calculations(self):
        """
        Computes the control signal for heading adjustment based on PID calculations.
        
        Proportional-Derivative Control:
            - Proportional term: Adjusts output based on the current error.
            - Derivative term: Predicts future trends using angular velocity.
        
        Publishes the computed control signal to the "bluerov2/r" topic.
        """
        Kp = 1.0
        Kd = 0.5
        msg = Float64()
        if self.desired_heading is None or self.measured_heading is None:
            return
        self.error = (self.desired_heading - self.measured_heading)
        if self.error > 180:
            self.error -= 360
        self.error = self.error * Kp * 100 / 180
        self.proportional = Kp * self.error
        self.derivative = self.measured_imu * 180 / math.pi * Kd
        msg.data = float((self.proportional + self.derivative))
        msg.data = min(max(msg.data, -self.max_throttle), self.max_throttle)
        self.publisher.publish(msg)

# Main
def main(args=None):
    """
    Main function to initialize and spin the HeadingControl node.
    Handles graceful shutdown on KeyboardInterrupt.
    """
    rclpy.init(args=args)
    node = HeadingControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
