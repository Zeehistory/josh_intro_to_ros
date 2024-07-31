#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from mavros_msgs.srv import CommandBool
from std_srvs.srv import SetBool
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

class Arm(Node):
    '''
    Node to arm and disarm ROV'''
    def __init__(self):
        '''
        Initializes node, cals inherited Node class
        Creates client, then waits for service request
        '''
        super().__init__("ARM")
        self.cli = self.create_client(SetBool,'bluerov2/arming')
        while not self.cli.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('service not available, waiting again...')
        # self.req = SetBool.Request()

    def arm_request(self, value):
        '''
        Arming Request, assigns value to request
        asynchronously calls
        '''
        self.future = self.cli.call_async(SetBool.Request(data=value))
        return self.future

    def destroy_node(self):
        disarm_future = self.arm_request(False)
        rclpy.spin_until_future_complete(self, disarm_future)
        self.get_logger().info("Disarmed before shutting down the node")
        super().destroy_node()


def main(args = None):
    rclpy.init(args=args)
    node = Arm()
    arm(node,True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

def arm(node,value):
    '''
    refactor of code, takes in value and requests service
    
    value: boolean in true or false
    '''
    future = node.arm_request(value)
    rclpy.spin_until_future_complete(node, future)
    response = future.result()
    if (value): 
        node.get_logger().info("Robot has been ARMED")
    if (not value): 
        node.get_logger().info("Robot has been DISARMED")

if __name__=="__main__":
    main()