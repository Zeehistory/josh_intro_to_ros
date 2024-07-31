#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from mavros_msgs.msg import Altitude 

'''
Notes: 
- fwd slash indicates fixed name
'''

'''Node to Find the Pressure'''

class DepthCalculate(Node): 

    def __init__(self):
        """
        subscribing to pressure
       
        publishing depth (altitude)
        """
        super().__init__("Depth")
        self.subscriber = self.create_subscription(
            FluidPressure,
            "bluerov2/pressure",
            self.depth_calculate,
            10
        )
        self.publisher = self.create_publisher(
            Altitude,
            "bluerov2/depth",
            10
        )
        self.get_logger().info("Starting Subscriber")


    def depth_calculate(self, msg):
        """
        Converting pressure to depth

        depth = (recorded pressure - atmospheric pressure)/(water density * gravity)

        recorded_pressure, atmospheric_pressure = pascals
        water_density = m^3
        gravity = m/s^2
        """
        recorded_pressure = msg.fluid_pressure
        atmospheric_pressure = 101325
        water_density = 1000
        g = 9.81
        
        depth = (recorded_pressure - atmospheric_pressure)/(water_density * g)
        # self.get_logger().info(f"\nPressure: {recorded_pressure}\nCalculated Depth: {depth}")
        msg = Altitude()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.local = depth
        self.publisher.publish(msg)

def main(args = None):
    rclpy.init(args = args)
    node = DepthCalculate()
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
            