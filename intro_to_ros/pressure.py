#!/usr/bin/env python3

import rclpy
import time
import math
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
u


'''Node to Find the Pressure'''
"""
Pressure (pascals) ---> depth (meters)

depth = (recorded pressure - at)ospheric pressure/
(water density * gravity)
"""

class DepthCalculate(Node): 

    def __init__(self):
        super().__init__("Depth")
        self.subscriber = self.create_subscription(
            FluidPressure,
            "bluerov2/pressure",
            self.depth_calculate,
        )
        self.get_logger().info("Starting Subscriber")

    def depth_calculate(self, msg):
        recorded_pressure = msg.fluid_pressure
        
        atmospheric_pressure = 101325
        water_density = 1000
        g = 9.81
        depth = (recorded_pressure - atmospheric_pressure)/(water_density * g)
        self.get_logger().info(f"Pressure: {recorded_pressure}\nCalculated Depth: {depth}")
               

def main(args = None):
    rclpy.init(args = args)
    node = DepthCalculate()
    try:
        rclpy.spin()
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destrooy_node()
        if rclpy.ok():
            rclpy.shutdown()