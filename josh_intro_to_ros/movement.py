#!/usr/bin/env python3

import rclpy
import time
import math
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

class Movement(Node):
    _move_forward = False
    _move_back = False
    _move_left = False
    _move_right = False
    _move_up = False
    _move_down = False
    _turn_left = False
    _turn_right = False
    _roll_left = False
    _roll_right = False
    _light_on = False
    _light_off = False
    _step_counter = 0
    def __init__(self):
        super().__init__("movement")
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.SYSTEM_DEFAULT,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.command_pub = self.create_publisher(
            OverrideRCIn,
            "bluerov2/override_rc",
            10
        )
        self.get_logger().info("starting publish")
        # self.publisher_timer = self.create_timer(
        #         1.0, self.roll(2000)
        # )
        
        self.get_logger().info("SUBSCRIBERS: " + str(self.command_pub.get_subscription_count()))
        self.loop = self.create_timer(0.1, self.movement)

    def _set_neutral_all_channels(self):
        neutral = OverrideRCIn()
        neutral.channels = [1500] * 18
        self.command_pub.publish(neutral)

    # def _loop(self):
    #     print (self._step_counter)
    #     if self._step_counter > 60:
    #         self.destroy_node()
    #         return
    #     self._dance_moves()

    #     # See https://www.ardusub.com/developers/rc-input-and-output.html#rc-input
    #     commands = OverrideRCIn()
    #     commands.channels = [OverrideRCIn.CHAN_NOCHANGE] * 8
    #     if self._move_forward:
    #         commands.channels[4] = 1700
    #     elif self._move_back:
    #         commands.channels[4] = 1300
    #     if self._move_left:
    #         commands.channels[5] = 1300
    #     elif self._move_right:
    #         commands.channels[5] = 1700
    #     if self._move_up:
    #         commands.channels[2] = 1700
    #     elif self._move_down:
    #         commands.channels[2] = 1300
    #     if self._turn_left:
    #         commands.channels[3] = 1300
    #     elif self._turn_right:
    #         commands.channels[3] = 1700

    #     self.command_pub.publish(commands)

    """"movement functions
    
    m_right  D   ONE
    m_left       DONE
    forward.     DONE
    back.        DONE
    up.          DONE
    down.        DONE
    t_right.     DONE
    t_left.      DONE
    r_right.     DONE
    r_left.      DONE
    light_on     DONE
    light_off.   DONE
    """""
    def all_false(self):
        self._move_forward = False
        self._move_back = False
        self._move_left = False
        self._move_right = False
        self._move_up = False
        self._move_down = False
        self._turn_left = False
        self._turn_right = False
        self._roll_left = False
        self._roll_right = False
        self._light_on = False
        self._light_off = False

    def m_right(self):
        self.all_false()
        self._move_right = True
                
    def m_left(self):
        self.all_false()
        self._move_left = True

    def forward(self):
        self.all_false()
        self._move_forward = True
        
    def backward(self):
        self.all_false()
        self._move_back = True

    def up(self):
        self.all_false()
        self._move_up = True

    def down(self):
        self.all_false()
        self._move_down = True
        
    def t_right(self):
        self.all_false()
        self._turn_right = True

    def t_left(self):
        self.all_false()
        self._turn_left = True

    def r_right(self):
        self.all_false()
        self._roll_right = True

    def r_left(self):
        self.all_false()
        self._roll_left = True

    def light_on(self):
        self.all_false()
        self._light_on = True
        
    def light_off(self):
        self.all_false()
        self._light_off = True

    def _dance_moves(self):
        self._step_counter += 0.1
        print (self._step_counter)
        if self._step_counter < 9.3 and math.floor(self._step_counter)%2 == 0: # intro
            self.m_left()

        elif self._step_counter < 9.3 and math.floor(self._step_counter)%2 == 1: # intro
            self.m_right()

        elif self._step_counter < 9.9: #hot
            print("hot")
            self._move_right = False
            self._move_left = False
            self._turn_left = True

        elif self._step_counter < 10.5: # cross
            print("hot")
            self._turn_left = False
            self._turn_right = True

        elif self._step_counter < 11.9: # buns
            print("hot")
            self._turn_right = False
            self._turn_left = True

        elif self._step_counter < 12.5: # hot
            self._turn_left = False
            self._turn_right = True

        elif self._step_counter < 13.1: # cross
            self._turn_right = False
            self._turn_left = True

        elif self._step_counter < 14.4: # buns
            self._turn_left = False
            self._turn_right = True

        elif self._step_counter < 15.6: # one a penny
            self._turn_right = False
            self._move_down = True

        elif self._step_counter < 16.8: # two a penny
            self._move_down = False
            self._move_up = True
            
        elif self._step_counter < 17.4: # hot
            self._move_up = False
            self._turn_left = True
            
        elif self._step_counter < 18.0: # cross
            self._turn_left = False
            self._turn_right = True

        elif self._step_counter < 19.7: # buns
             self._turn_left = False
             self._turn_right = True

        elif self._step_counter < 21.8: # if no daughters
            self._roll_left = True
            self._turn_right = False
            
        elif self._step_counter < 23.0: # give to sons
             self._roll_left = False
             self._roll_right = True
            
        elif self._step_counter < 25: # barrel (move down part)
            self._roll_right = False
            self._move_down = True
            
        elif self._step_counter < 30.3: # barrel (spin)
            self._move_down = False
            self._roll_right = True
            
        elif self._step_counter < 32: # one penny
            self._move_down = True
            self._roll_right = False

        elif self._step_counter < 32.6: # two penny
            self._move_up = True
            self._move_down = False
            
        elif self._step_counter < 33.2: # hot
            self._move_up = False
            self._turn_right = True
            
        elif self._step_counter < 33.8: # cross
            self._turn_right = False
            self._turn_left = True

        elif self._step_counter < 36: # barrel (down)
            self._turn_left = False
            self._move_down = True
            
        elif self._step_counter < 45: # barrel (spin)
            self._move_down = False
            self._roll_right = True
            
        elif self._step_counter < 48: # barrel ( down)
            self._roll_right = False
            self._move_down = True

        elif self._step_counter < 55.5: # barrel ( spin)
            self._move_down = False
            self._roll_right = True
            
        elif self._step_counter < 56.9: # sons
            self._roll_right = False
            self._roll_left = True

        elif self._step_counter < 58.1: # one penny
            self._roll_left = False
            self._move_down = True
            
        elif self._step_counter < 59.3: # two penny
            self._move_down = False
            self._move_up = True
            
        elif self._step_counter < 59.9: # hot
            self._move_up = False
            self._turn_right = True

        elif self._step_counter < 60.5: # cross
            self._turn_right = False
            self._turn_left = True
        
        else: # buns
            self._turn_right = True
            self._turn_left = False

    def movement(self):
        if self._step_counter > 62:
            self.destroy_node()
            return
        self._dance_moves()

        commands = OverrideRCIn()
        commands.channels = [OverrideRCIn.CHAN_NOCHANGE] * 18
        if self._move_forward:
            commands.channels[4] = 1600
        elif self._move_back:
            commands.channels[4] = 1400
        if self._move_left:
            commands.channels[5] = 1400
        elif self._move_right:
            commands.channels[5] = 1600
        if self._move_up:
            commands.channels[2] = 1600
        elif self._move_down:
            commands.channels[2] = 1400
        if self._turn_left:
            commands.channels[3] = 1400
        elif self._turn_right:
            commands.channels[3] = 1600
        if self._roll_left:
            commands.channels[1] = 1900
        elif self._roll_right:
            commands.channels[1] = 1900
        if self._light_on:
            commands.channels[8] = 1900
            commands.channels[9] = 1900
        elif self._light_off: 
            commands.channels[8] = 1100
            commands.channels[9] = 1100
        self.command_pub.publish(commands)     
                               
    # 

    
    def power_on(self, sec):
        msg = OverrideRCIn()
        msg.channels = [65535] * 18
        msg.channels[8] = True
        msg.channels[9] = True
        self.override_rc_callback(msg)
        time.sleep(sec)

    def power_off(self, sec):
        msg = OverrideRCIn()
        msg.channels = [65535] * 18
        msg.channels[8] = False
        msg.channels[9] = False
        self.override_rc_callback(msg)    

def main(args=None):
    rclpy.init(args=args)
    node = Movement()
    node._light_on = True
    node.movement()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        
        node.destroy_node()
        rclpy.shutdown()