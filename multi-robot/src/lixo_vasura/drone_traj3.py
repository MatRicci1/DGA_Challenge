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
from env2 import *
import time
from copy import deepcopy
from geometry_msgs.msg import PoseStamped
from dronetraj import *

#####################
###### Classes
#####################

class drone_follow:

    #####################
    ###### Initialize
    #####################

    def __init__(self, drone_N, xTrue, xGoal, drone_pos, drone_rel_pos_0):
        self.orientation = [0,0,0]
        self.real_pos = [0,0,0]
        self.drone_nav_pos = [0,0,0]
        self.drone_rel_pos = drone_rel_pos_0
        self.drone_pos = drone_pos
        #self.gps = [0,0,0]
        #self.gps_0 = drone_gps
        self.drone_rel_pos_0 = drone_rel_pos_0
        self.Drone = drone_N
        self.xTrue = xTrue  
        self.xGoal = xGoal
        rospy.Subscriber('/drone{}/sensor_pose'.format(self.Drone), PoseStamped, self.callback_sensor_pose)
        for Xg in self.xGoal:
            # Simulate Vehicle motion
            self.move(self.drone_rel_pos,Xg)
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

    def callback_sensor_pose(self, data):
        # x-axis
        self.drone_nav_pos[0] = data.pose.position.x
        # y-axis
        self.drone_nav_pos[1] = data.pose.position.y
        # z-axis
        self.drone_nav_pos[2] = data.pose.position.z

        self.real_pos = self.drone_nav_pos + self.drone_pos
        # Update rel_pos
        self.drone_rel_pos = self.drone_nav_pos + self.drone_rel_pos_0
        #print("\n sensor pose: ", self.sensor_pose)

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
                        rospy.Subscriber('drone{}/nav/sensor_pose'.format(self.Drone), PoseStamped, self.callback_sensor_pose)
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

        ## Commande
        u = np.array([v_x, v_y, v_z])#, omega]
        idx = np.argwhere(u > 6)
        u[idx] = 6

        return u

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

    def move(self,xTrue ,xGoal):
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
                    rospy.Subscriber('/drone{}/nav/sensor_pose'.format(self.Drone), PoseStamped, self.callback_sensor_pose)
                    #rospy.Subscriber('/drone{}/fix'.format(self.Drone), NavSatFix, self.callback_gps)
                    #print("Entrou")
                    xTrue = self.drone_rel_pos
                    t1 = rospy.Time.now().to_sec()
                    u = self.DroneToPoseControl(xTrue,xGoal)
                    if t1-t0 > 0.01:
                        dt = t1-t0
                        
                        # Compute Control
                        # print("Dt: ", dt)
                        # print("xTrue: ",xTrue)
                        # print("Criteria: ",crit)
                        u=self.DroneToPoseControl(xTrue,xGoal)
                    xTrue = self.drone_rel_pos
                    # rospy.init_node('move_drone{}'.format(self.Drone))
                    
                    # velocity_publisher = rospy.Publisher('drone{}/cmd_vel'.format(self.Drone), Twist, queue_size=1, latch=True)
                    # vel_msg = Twist()

                    print("xTrue: ",xTrue)
                    print("xGoal: ",xGoal)

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
    drone_rel_Position = drones_pos_rel[int(sys.argv[1])-1]
    drone_Position = drones_pos[int(sys.argv[1])-1]
    #drone_GPS = drones_gps[int(sys.argv[1])-1]
    follow = drone_follow(drone_N, xTrue, xGoal, drone_Position, drone_rel_Position)

    print("Terminou")
