#!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from mavros_msgs.msg import ManualControl
# from std_msgs.msg import Int16
# from sensor_msgs.msg import Imu

# class HeadingControl(Node):
#     def __init__(self):
#         super().__init__("headingControl")
        
#         # Constants
#         self.Kp = 1.5
#         self.Ki = 0.01
#         self.Kd = 0.1
#         self.max_integral = 10.0
#         self.max_throttle = 100.0

#         # PID variables
#         self.error_accumulator = 0
#         self.previous_error = 0
#         self.desired_heading = None
#         self.measured_heading = None
#         self.t1 = 0
#         self.t2 = 0

#         # Subscriptions
#         self.measured_heading_sub = self.create_subscription(
#             Int16,
#             "bluerov2/heading",
#             self.measuredHeadingCallback,
#             10
#         )

#         self.measured_imu_sub = self.create_subscription(
#             Imu,
#             "bluerov2/imu",
#             self.measuredImuCallback, 
#             10
#         )

#         self.desired_heading_sub = self.create_subscription(
#             Int16,
#             "bluerov2/desired_heading",
#             self.desiredHeadingCallback,
#             10
#         )

#         # Publisher
#         self.publisher = self.create_publisher(
#             ManualControl,
#             "bluerov2/manual_control",
#             10
#         )

#     def measuredHeadingCallback(self, msg):
#         self.measured_heading = msg.data
#         self.get_logger().info(f"\nMeasured Heading: {self.measured_heading}")

#     def measuredImuCallback(self, msg):
#         self.measured_imu = msg.angular_velocity.z
#         if self.t1 == 0:
#             self.t1 = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
#             self.t2 = self.t1
#         else:
#             self.t1 = self.t2
#             self.t2 = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
#         self.power_calculations()

#     def desiredHeadingCallback(self, msg):
#         self.desired_heading = msg.data
#         self.get_logger().info(f"\nDesired Heading: {msg.data}")

#     def power_calculations(self):
#         if self.desired_heading is None or self.measured_heading is None:
#             return

#         # PID calculations
#         error = self.desired_heading - self.measured_heading

#         # Normalize error to [-180, 180] range
#         if error > 180:
#             error -= 360
#         elif error < -180:
#             error += 360

#         dt = self.t2 - self.t1
#         if dt <= 0:
#             return

#         self.error_accumulator += error * dt
#         self.error_accumulator = max(min(self.error_accumulator, self.max_integral), -self.max_integral)

#         derivative = (error - self.previous_error) / dt
#         self.previous_error = error

#         control_signal = self.Kp * error + self.Ki * self.error_accumulator + self.Kd * derivative

#         # Constrain control signal
#         control_signal = max(min(control_signal, self.max_throttle), -self.max_throttle)

#         # Create and publish ManualControl message
#         msg = ManualControl()
#         msg.x = 0.0  # No forward/backward movement
#         msg.y = 0.0  # No lateral movement
#         msg.z = 0.0  # No vertical movement
#         msg.r = float(control_signal)

#         self.publisher.publish(msg)
#         self.get_logger().info(f"Control Signal: {control_signal}")

#         # Check if the error is within the 1% tolerance
#         error_percentage = abs(error / self.desired_heading) * 100 if self.desired_heading != 0 else 0
#         if error_percentage < 1.0:
#             self.get_logger().info("Heading error within 1% tolerance")

# def main(args=None):
#     rclpy.init(args=args)
#     node = HeadingControl()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == "__main__":
#     main()


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ManualControl
from std_msgs.msg import Int16
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
        '''
        creating current heading and desired heading subscriptions
        creating a power putput publisher
        '''

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
            ManualControl,
            "bluerov2/manual_control",
            10
        )

    def measuredHeadingCallback(self,msg):
        self.measured_heading = msg.data
        # if (self.t1 == 0):
        #     self.t1 = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        #     self.t2 = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        # else:
        #     self.t1 = self.t2
        #     self.t2 = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        
        # self.power_calculations()
        self.get_logger().info(f"\nMeasured Heading: {self.measured_heading}")

    def measuredImuCallback(self,msg):
        """
        Retrieving angular velocity from IMU sensors with a timer 
        """
        self.measured_imu = msg.angular_velocity.z
        if (self.t1 == 0):
            self.t1 = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            self.t2 = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        else:
            self.t1 = self.t2
            self.t2 = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        
        self.power_calculations()
        # self.get_logger().info(f"\nMeasured Angular Velocity: {self.measured_imu}")

    def desiredHeadingCallback(self,msg):
        self.desired_heading = msg.data
        self.get_logger().info(f"\nDesired Heading: {msg.data}")

    def power_calculations(self):
        '''
        Calculating the 
        power output based off of: 
    
        rac{360}{1+e^{rac{-\pi x}{180}}}-180
        '''
        # constants
        Kp = 1.0
        #Ki = 0.0
        Kd = 0.5
        msg = ManualControl()
        if (self.desired_heading == None): return
        if (self.measured_heading == None): return
        self.measured_heading = self.measured_heading
        self.desired_heading = self.desired_heading
        self.error = (self.desired_heading - self.measured_heading)
        if (self.error > 180):
            self.error = self.error-360
        self.error = self.error * Kp * 100/180
        # self.error = 360 / (1 + math.pow(2.7182,(self.error * 3.1415 * -0.01))) - 180

        # proportional control
        self.get_logger().info(f"\nError: {self.error}")
        self.proportional = Kp * self.error
        self.get_logger().info(f"\nProportional: {self.proportional}")


        self.derivative = self.measured_imu * 180 / 3.1415 * Kd
        self.get_logger().info(f"\nDerivative: {self.derivative}")
        # add all & publish 
        # msg.r = float((self.proportional + self.integral + self.derivative) * -1)
        msg.r = float((self.proportional + self.derivative) * 1)
        msg.r = min(max(msg.r, -self.max_throttle), self.max_throttle)
        self.get_logger().info(f"\nPID: {msg.r}")
        self.publisher.publish(msg)

#Main
def main(args = None):
    rclpy.init(args = args)
    node = HeadingControl()
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