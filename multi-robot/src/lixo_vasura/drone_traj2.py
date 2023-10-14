#!/usr/bin/env python

#####################
###### Import
#####################

import rospy
from geometry_msgs.msg import Twist
from hector_uav_msgs.srv import EnableMotors
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
import numpy as np
import math
from matplotlib import pyplot as plt
import get_drone_position
from threading import Event 
import sys
from env import *
import time
from copy import deepcopy

#####################
###### Classes
#####################

class drone_follow:

    #####################
    ###### Initialize
    #####################

    def __init__(self, drone_N, xTrue, xGoal, drone_pos, drone_gps):
        self.orientation = [0,0,0]
        self.xyz = [0,0,0]
        self.gps = [0,0,0]
        self.gps_0 = drone_gps
        self.drone_pos = drone_pos
        self.Drone = drone_N
        self.xTrue = xTrue  
        self.xGoal = xGoal
        rospy.Subscriber('/drone{}/fix'.format(self.Drone), NavSatFix, self.callback_gps_init)
        
        for (Xt, Xg) in zip(self.xTrue, self.xGoal):
            # Simulate Vehicle motion
            self.move(Xt,Xg)
            #self.orientation_control()

    #####################
    ## Get Orientation
    #####################

    def callback_orientation(self, data): 
        q = data.orientation
        # Roll (x-axis rotation)
        self.orientation[0] = math.atan2(2*q.w*q.x + 2*q.y*q.z, 1 - 2*q.x*q.x + 2*q.y*q.y)
        # Pitch (y-axis rotation)
        self.orientation[1] = math.asin(2*q.w*q.y - 2*q.z*q.x)
        # Yaw (z-axis rotation)
        self.orientation[2] = math.atan2(2*q.z*q.w + 2*q.y*q.x, 1 - 2*q.z*q.z + 2*q.y*q.y)

    #####################
    ##### Get gps init
    #####################

    def callback_gps_init(self, data): 
        self.gps[0] = data.longitude
        self.gps[1] = data.latitude
        self.gps[2] = data.altitude

    #####################
    ##### Get gps
    #####################
    def conversion_measure(self, lat1, lon1, lat2, lon2):  
        R = 6378.137 # Radius of earth in KM
        dLat = (lat2 * math.pi / 180) - (lat1 * math.pi / 180)
        dLon = (lon2 * math.pi/ 180) - (lon1 * math.pi/ 180)
        a = math.sin(dLat/2) * math.sin(dLat/2) + math.cos(lat1 * math.pi/ 180) * math.cos(lat2 * math.pi/ 180) * math.sin(dLon/2) * math.sin(dLon/2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        d = R * c
        return d * 1000 # meters


    def callback_gps(self, data): 
       
        deltax = self.conversion_measure(self.gps_0[0], data.longitude, data.latitude, data.longitude)
        deltay = self.conversion_measure(data.latitude, self.gps_0[1], data.latitude, data.longitude)
        self.xyz[0] = deltax
        self.xyz[1] = deltay
        self.xyz[2] = (data.altitude  - self.gps_0[2])





    #####################
    ## Update orientation
    #####################
    
    def orientation_control(self):
        assert int(self.Drone) in {1, 2, 3, 4, 5, 6, 7, 8, 9, 10}
        # check if motors are on
        if self.motor_on():
            velocity_publisher = rospy.Publisher('drone{}/cmd_vel'.format(self.Drone), Twist, queue_size=1, latch=True)
            rospy.Subscriber('/drone{}/raw_imu'.format(self.Drone), Imu, self.callback_orientation)
            vel_msg = Twist()

            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0.2

            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0

            # Setting the current time for distance calculus
            t0 = rospy.Time.now().to_sec()
            t2 = t0
            try:
                # Loop to move the turtle in an specified distance
                while (np.abs(self.orientation[2]) > 0.1) and not rospy.is_shutdown():
                    # Takes actual time to velocity calculus
                    t1 = rospy.Time.now().to_sec()
                    # Publish the velocity
                    if (t1-t2) > 0.01:
                        rospy.Subscriber('/drone{}/raw_imu'.format(self.Drone), Imu, self.callback_orientation)
                        vel_msg.angular.z = -self.orientation[2]*2
                        if vel_msg.angular.z > 3:
                            vel_msg.angular.z = 3
                        elif vel_msg.angular.z < -3:
                            vel_msg.angular.z = -3
                        velocity_publisher.publish(vel_msg)
                        t2 = t1
                        # print("Vel_ang_z",vel_msg.angular.z)
                        # print("Orientation (rad): ", self.orientation[2])

                # After the loop, stops the robot
                vel_msg.linear.x = 0.0
                vel_msg.linear.y = 0.0
                vel_msg.linear.z = 0.0

                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = 0
                
                # Force the robot to stop
                velocity_publisher.publish(vel_msg)

            except KeyboardInterrupt:
                print('Interrupted')
                vel_msg.linear.x = 0.0
                vel_msg.linear.y = 0.0
                vel_msg.linear.z = 0.0

                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = 0
                # Force the robot to stop
                velocity_publisher.publish(vel_msg)
                sys.exit(0)
    
    #####################
    ##### Uptade speed
    #####################

    def DroneToPoseControl(self,xTrue,xGoal):
        
        # parameters
        K_rho = 0.8 
        K_sigma = 4


        ## Calcul des Vitesses de translation et rotation
        # Variables auxiliaires
        rho_x = xGoal[0]-xTrue[0]
        rho_y = xGoal[1]-xTrue[1]
        rho_z = xGoal[2]-xTrue[2]


        ## Commande Loin
        v_x = K_rho*rho_x
        v_y = K_rho*rho_y
        v_z = K_rho*rho_z

        # # Variables auxiliaires
        # phi_x = 0 - self.orientation[0] 
        # theta_y = 0 - self.orientation[1] 
        # psi_z = 0 - self.orientation[2] 

        # ## Commande Loin
        # w_x = K_sigma*phi_x
        # w_y = K_sigma*theta_y
        # w_z = K_sigma*psi_z
        
        # omega = K_alpha*alpha
        #omega = 0



        ## Commande
        u = np.array([v_x, v_y, v_z])#, omega]
        idx = np.argwhere(u > 6)
        u[idx] = 6

        # omega = np.array([w_x, w_y, w_z])
        return u

    #####################
    ##### Uptade position
    #####################

    def SimulateDrone(self, xTrue,u,dt):
        x = xTrue[0] + u[0]*dt
        y = xTrue[1] + u[1]*dt
        z = xTrue[2] + u[2]*dt

        print('xTrue_new', [x,y,z])
        rospy.Subscriber('/drone{}/fix'.format(self.Drone), NavSatFix, self.callback_gps)
        print("xyz ", self.xyz)
        print("GPS_0",self.gps_0)

        return np.array([x,y,z])

    #####################
    ##### Angle wrap
    #####################

    def angle_wrap(self, a):
        """
        Keep angle between -pi and pi
        """
        return np.fmod(a + np.pi, 2*np.pi ) - np.pi

    #####################
    ##### Move drone
    #####################

    def move(self,xTrue,xGoal):
        # Starts a new node
        assert int(self.Drone) in {1, 2, 3, 4, 5, 6, 7, 8, 9, 10}
        rospy.init_node('move_drone{}'.format(self.Drone))
        # check if motors are on
        if self.motor_on():
            # u=self.DroneToPoseControl(xTrue,xGoal)
            velocity_publisher = rospy.Publisher('drone{}/cmd_vel'.format(self.Drone), Twist, queue_size=1, latch=True)

            vel_msg = Twist()

            # Setting the current time for distance calculus
            t0 = rospy.Time.now().to_sec()
            # current_distance = 0
            crit = np.max(np.absolute(xTrue-xGoal))
            try:
                # Loop to move the turtle in an specified distance
                while (crit>0.06) and not rospy.is_shutdown(): 
                    rospy.Subscriber('/drone{}/fix'.format(self.Drone), NavSatFix, self.callback_gps)
                    #print("Entrou")
                    t1 = rospy.Time.now().to_sec()
                    u =self.DroneToPoseControl(xTrue,xGoal)
                    if t1-t0 > 0.01:
                        dt = t1-t0
                        
                        # Compute Control
                        # print("Dt: ", dt)
                        # print("xTrue: ",xTrue)
                        # print("Criteria: ",crit)
                        u=self.DroneToPoseControl(xTrue,xGoal)
                        
                        # Calculates distancePoseStamped~
                        #if dt < 0.2:
                        if dt > 1:
                            xTrue = self.SimulateDrone(xTrue,u,0.01)
                            t0 = t1
                        else:
                            xTrue = self.SimulateDrone(xTrue,u,dt)
                            t0 = t1
                    
                    # rospy.init_node('move_drone{}'.format(self.Drone))
                    
                    # velocity_publisher = rospy.Publisher('drone{}/cmd_vel'.format(self.Drone), Twist, queue_size=1, latch=True)
                    # vel_msg = Twist()

                    vel_msg.linear.x = u[0]
                    vel_msg.linear.y = u[1]
                    vel_msg.linear.z = u[2]

                    vel_msg.angular.x = 0
                    vel_msg.angular.y = 0
                    vel_msg.angular.z = 0
                    #print(vel_msg)    
                    velocity_publisher.publish(vel_msg)
                    
                    crit = np.max(np.absolute(xTrue-xGoal))

                # After the loop, stops the robot
                vel_msg.linear.x = 0.0
                vel_msg.linear.y = 0.0
                vel_msg.linear.z = 0.0

                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = 0
                
                # Force the robot to stop
                velocity_publisher.publish(vel_msg)
            except KeyboardInterrupt:
                print('Interrupted')
                vel_msg.linear.x = 0.0
                vel_msg.linear.y = 0.0
                vel_msg.linear.z = 0.0

                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = 0
                # Force the robot to stop
                velocity_publisher.publish(vel_msg)
                sys.exit(0)

    #####################
    ### Turn on motor
    #####################

    # Rosservice function to turn on the drone motors
    def motor_on(self):
        rospy.wait_for_service('drone{}/enable_motors'.format(self.Drone))
        try:
            motor_on = rospy.ServiceProxy('drone{}/enable_motors'.format(self.Drone), EnableMotors, True)
            turn_on = motor_on(True)
            return turn_on
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

#####################
### Main
#####################

if __name__ == '__main__':
    drone_N = int(sys.argv[1])
    xTrue = drones_traj_true[int(sys.argv[1])-1]
    xGoal = drones_traj_goal[int(sys.argv[1])-1]
    drone_Position = drones_pos[int(sys.argv[1])-1]
    drone_GPS = drones_gps[int(sys.argv[1])-1]
    follow = drone_follow(drone_N, xTrue, xGoal, drone_Position, drone_GPS)

    print("Terminou")
