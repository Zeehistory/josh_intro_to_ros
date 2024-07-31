#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
# from geometry_msgs.msg import Vector3
# from sensor_msgs.msg import Imu
# from sensor_msgs.msg import BatteryState
# from sensor_msgs.msg import FluidPressure
from geometry_msgs.msg import Pose2D
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

class PhysicsSim(Node):
    def __init__(self):
        '''
        initializes node
        '''
        super().__init__("physicsSim")
        qosProfile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.subscription = self.create_subscription(
            Pose2D,
            "/physics/pose2d",
            self.pose2d_callback,
            qosProfile
        )
        self.get_logger().info("starting subscriber node")

    def pose2d_callback(self,msg):
        pass

    def calculate_auv2_acceleration(self, T, alpha, theta, mass = 100):
        '''
        calculates the acceleration of the AUV in the 2D plane

        T: an np.ndarray of the magnitudes of the forces applied by the thrusters in Newtons
        alpha: the angle of the thrusters in radians
        theta: the angle of the AUV in radians
        mass: the mass of the AUV in kilograms. The default value is 100 kg
        '''
        # a1 = calculate_auv_acceleration(T[0],alpha,mass)
        # a2 = calculate_auv_acceleration(T[1],alpha,mass)
        # a3 = calculate_auv_acceleration(T[2],alpha,mass)
        # a4 = calculate_auv_acceleration(T[3],alpha,mass)
        # return a1 + a2 - a3 - a4
        ax = T[0]*np.cos(alpha) + T[1]*np.cos(alpha) - T[2]*np.cos(alpha) - T[3]*np.cos(alpha)
        ay = T[0]*np.sin(alpha) - T[1]*np.sin(alpha) + T[2]*np.sin(alpha) - T[3]*np.sin(alpha)
        acceleration = np.array(ax,ay)/mass
        rotation_matrix = np.ndarray([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
        acceleration = np.dot(rotation_matrix,acceleration)
        return acceleration

    def calculate_auv2_angular_acceleration(self, T, alpha, L, l, inertia = 100):
        '''
        calculates the angular acceleration of the AUV

        T: an np.ndarray of the magnitudes of the forces applied by the thrusters in Newtons
        alpha: the angle of the thrusters in radians
        L: the distance from the center of mass of the AUV to the thrusters in meters
        l: the distance from the center of mass of the AUV to the thrusters in meters
        inertia: the moment of inertia of the AUV in kg * m^3. The default value is 100 kg * m^3
        '''
        distance = np.sqrt(L**2, l**2)
        angular_acceleration = T[0]-T[1]+T[2]-T[3]
        angular_acceleration = angular_acceleration * distance * np.sin(alpha)/inertia

        return angular_acceleration

    def simulate_auv2_motion(self, T, alpha, L, l, mass = 100, inertia = 100, dt = 0.1, t_final = 10, x0 = 0, y0 = 0, theta0 = 0):
        '''
        simulates the motion of the AUV in the 2D plane

        T: an np.ndarray of the magnitudes of the forces applied by the thrusters in Newtons
        alpha: the angle of the thrusters in radians
        L: the distance from the center of mass of the AUV to the thrusters in meters
        l: the distance from the center of mass of the AUV to the thrusters in meters
        mass: the mass of the AUV in kilograms. The default value is 100 kg
        inertia: the moment of inertia of the AUV in kg * m^2. The default value is 100 kg * m^2
        dt: the time step of the simulation in seconds. The default value is 0.1 s
        t_final: the final time of the simulation in seconds. The default value is 10 s
        x0: the initial x-position of the AUV in meters. The default value is 0 m
        y0: the initial y-position of the AUV in meters. The default value is 0 m
        theta0: the initial angle of the AUV in radians. The default value is 0 rad
        '''
        # alpha = math.pi/180 * alpha
        # t = np.arange(0,t_final+dt,dt)
        # moment_arm = np.sqrt(l**2+L**2)
        # net_torque = moment_arm*np.sin(alpha)*(T[0]+T[2]-T[1]-T[3])
        # theta = t**2 + 1/2 * net_torque/inertia + theta0
        # print (theta)

        # horizontal = np.sin(math.pi/2-alpha+theta)*(T[0] + T[1] - T[2] - T[3])/mass
        # vertical = np.cos(math.pi/2-alpha+theta)*(T[0] + T[1] - T[2] - T[3])/mass
        # a = np.sqrt(horizontal**2 + vertical**2)

        # hv = (np.array([sum(horizontal[:1])]) * dt for i in range(len(horizontal)))
        # vv = (np.array([sum(vertical[:1])]) * dt for i in range(len(vertical)))
        # v = np.sqrt(hv**2 + vv**2)

        # x = x0 + (np.array([sum(hv[:1])]) * dt for i in range(len(hv)))
        # y = y0 + (np.array([sum(vv[:1])]) * dt for i in range(len(vv)))

        # alpha_radians = np.radians(alpha)
        theta_degrees = theta0
        intervals = t_final/dt + 1
        t = np.arange(0,t_final+dt,dt)

        thruster_distance = np.sqrt(l**2 + L**2)
        x = np.zeros(intervals)
        y = np.zeros(intervals)
        theta = np.zeros(intervals)
        v = np.zeros(intervals)
        omega = np.zeros(intervals)
        a = np.zeros(intervals)
        x[0], y[0], theta[0], v[0], omega[0], a[0] = x0,y0,theta0,0,0,0
        for i in range(1,intervals+1):
            a[i] = a[i-1] + self.calculate_auv2_acceleration(T,alpha,theta_degrees,mass)
            v[i] = v[i-1] + a[i]*dt
            x[i] = x[i-1] + v[i][0]*dt
            y[i] = y[i-1] + v[i][1]*dt

            angular_acceleration = self.calculate_auv2_angular_acceleration(T,alpha,L,l,inertia)
            omega[i] = omega[i-1] + angular_acceleration*dt
            theta[i] = theta[i-1] + omega[i]*dt

        return t,x,y,theta,v,omega,a