#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import FluidPressure
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy


class bluerov2_sensors(Node):
    def __init__(self):
        '''
        initializes node
        '''
        super().__init__("bluerov2_sensors")
        qosProfile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.battery = self.create_subscription(
            BatteryState,
            "/mavros/battery",
            self.battery_callback,
            qosProfile
        )
        self.battery
        self.imu = self.create_subscription(
            Imu,
            "/mavros/imu/data",
            self.imu_callback,
            qosProfile
        )
        self.imu
        self.static_pressure = self.create_subscription(
            FluidPressure,
            "/mavros/imu/static_pressure",
            self.static_pressure_callback,
            qosProfile
        )
        self.static_pressure
        self.diff_pressure = self.create_subscription(
            FluidPressure,
            "/mavros/imu/diff_pressure",
            self.diff_pressure_callback,
            qosProfile
        )
        self.diff_pressure
        self.get_logger().info("starting subscriber node")


        self.battery_param = BatteryState()
        self.imu_param = Imu()
        self.static_pressure_param = FluidPressure()
        self.diff_pressure_param = FluidPressure()

        self.timer = self.create_timer(5, self.timer_callback)

    def battery_callback(self, msg):
        '''
        battery callback, logs and sets attribtue
        
        msg is the message
        '''
        self.battery_param = msg
        self.get_logger().info(f"Battery\n\tvoltage: {msg.voltage}\tcurrent: {msg.current}\n")

    def imu_callback(self,msg):
        '''
        imu callback, logs and sets imu attribute
        
        msg is the message
        '''
        self.imu_param = msg
        self.get_logger().info(f"IMU\n\tAcceleration: {msg.linear_acceleration}\n")

    def static_pressure_callback(self,msg):
        '''
        static pressure callback, logs and sets static pressure
        
        msg is the message
        '''
        self.static_pressure_param = msg
        self.get_logger().info(f"Static Pressure\n\tStatic Pressure: {msg.fluid_pressure}\n")
        self.get_logger().info(f"Depth: {self.depth_calculation(msg.fluid_pressure)}\n")
    
    def depth_calculation(self, pressure):
        return (pressure-101325)/9.81/1000

    def diff_pressure_callback(self,msg):
        '''
        diff pressure callback, logs and sets diff pressure

        msg is the message
        '''
        self.diff_pressure_param = msg
        self.get_logger().info(f"Diff Pressure\n\tDiff Pressure: {msg.fluid_pressure}\n")

    def timer_callback(self):
        '''
        imter callback, checks if voltage is too low
        '''
        voltage = self.battery_param.voltage
        if voltage < 12.0:
            self.get_logger().info("WARNING: Voltage below safe level")
        

def main(args = None):
    rclpy.init(args=args)
    node = bluerov2_sensors()
    
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