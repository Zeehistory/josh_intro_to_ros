#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import Altitude, ManualControl
from std_msgs.msg import Int16, Float64

class depthControl(Node):
    '''
    Attributes:
        error_accumulator (float): Cumulative error for the integral term.
        previous_error (float): Error from the previous control cycle for the derivative term.
        t1 (float): Timestamp of the previous control cycle.
        t2 (float): Timestamp of the current control cycle.
        max_integral (float): Maximum value for the integral term to prevent windup.
        max_throttle (float): Maximum value for the output throttle signal.
        desired_depth (Altitude): Desired depth target received from the subscription.
        measured_depth (Altitude): Current depth measurement received from the subscription.
    '''
    error_accumulator = 0
    previous_error = 0
    t1 = 0
    t2 = 0
    max_integral = 1.0
    max_throttle = 100.0
    desired_depth = None
    measured_depth = None
    
    def __init__(self):
        """
        Initializes the DepthControl node, setting up the required subscriptions and publisher.

        Subscriptions:
            - "bluerov2/depth": Current depth measurements (Altitude message).
            - "bluerov2/desired_depth": Target depth (Altitude message).

        Publisher:
            - "bluerov2/z": PID control signal for adjusting depth (Float64 message).
        """
        super().__init__("depthControl")

        self.measuredDepth = self.create_subscription(
            Altitude,
            "bluerov2/depth",
            self.measuredDepthCallback,
            10
        )
        self.get_logger().info("Starting measured depth subscription.")

        self.desiredDepth = self.create_subscription(
            Altitude,
            "bluerov2/desired_depth",
            self.desiredDepthCallback,
            10
        )
        self.get_logger().info("Starting desired depth subscription.")

        self.publisher = self.create_publisher(
            Float64,
            "bluerov2/z",
            10
        )
        self.get_logger().info("Starting publisher.")

    def measuredDepthCallback(self, msg):
        """
        Callback function for the "bluerov2/depth" topic.
        Updates the current measured depth and timestamps for the control loop.

        Args:
            msg (Altitude): Message containing the current depth measurement.
        """
        self.measured_depth = msg
        if self.t1 == 0:
            self.t1 = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.t2 = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        else:
            self.t1 = self.t2
            self.t2 = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.depth_control()

    def desiredDepthCallback(self, msg):
        """
        Callback function for the "bluerov2/desired_depth" topic.
        Updates the target depth for the control loop.

        Args:
            msg (Altitude): Message containing the desired depth.
        """
        self.desired_depth = msg

    def depth_control(self):
        """
        Implements the PID control algorithm to calculate the depth adjustment signal.
        Combines proportional, integral, and derivative terms based on the current and desired depth.

        The output signal is limited to `max_throttle` to prevent excessive control commands.

        Control Parameters:
            - Proportional gain (Kp): -150.0
            - Integral gain (Ki): -20.0
            - Derivative gain (Kd): -50.0

        Steps:
            1. Calculate the error between desired and measured depth.
            2. Compute the proportional, integral, and derivative terms.
            3. Combine these terms to create the control signal.
            4. Clamp the control signal to the range [-max_throttle, max_throttle].
            5. Publish the control signal to the "bluerov2/z" topic.
        """
        msg = Float64()
        msg.data = 0.0

        if self.measured_depth is None or self.desired_depth is None:
            return

        measured_position = self.measured_depth.local
        self.desired_position = self.desired_depth.local

        if self.desired_position is None:
            return

        dt = self.t2 - self.t1

        # PID coefficients
        Kp = -150.0
        Ki = -20.0
        Kd = -50.0

        # Proportional term
        error = self.desired_position - measured_position
        self.proportional = Kp * error

        # Integral term
        self.error_accumulator += error * dt
        self.integral = min(max(Ki * self.error_accumulator, -self.max_integral), self.max_integral)

        # Derivative term
        self.derivative = 0 if dt == 0 else Kd * (error - self.previous_error) / dt
        self.previous_error = error

        # Control signal
        msg.data = float((self.proportional + self.integral + self.derivative) * 1)
        msg.data = min(max(msg.data, -self.max_throttle), self.max_throttle)

        self.publisher.publish(msg)
        
def main(args = None):
    rclpy.init(args = args)
    node = depthControl()
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
