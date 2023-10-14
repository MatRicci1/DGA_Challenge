#! /usr/bin/env python

#################################################################################################################
# Node that increases the accuracy of people localization
# Moves in four directions over a person
# Publishes a topic with the accurate position of each people
#################################################################################################################

import rospy
import time
import random
from hector_uav_msgs.srv import EnableMotors
from geometry_msgs.msg import PoseStamped
from std_msgs.msg    import Float64MultiArray
from std_msgs.msg    import Float64
import actionlib
import hector_uav_msgs.msg
import numpy as np  
from Env_strategy import *
import sys
import rosnode

class Correct_position:

    def callback_seen(self, data):
        self.seen = data.data
    
    def callback_people(self, data):
        self.Goals = np.array(data.data)
    
    def callback_flag(self, data):
        self.flag = data.data

    def __init__(self, droneN, pos):
        try:
            self.seen = []
            self.droneN = droneN 
            self.droneN_main = droneN - 5  
            self.Goals = []
            self.flag = 0.0
            self.height = 9
            rospy.init_node('drone_explorer{}'.format(self.droneN_main))

            ### Verify if strategy drone has fished its path or if it has been killed 
            while (self.flag != 1.0):
                rospy.Subscriber('drone{}/final'.format(self.droneN_main), Float64, self.callback_flag)
                node_list = rosnode.get_node_names()
                if '/drone{}/controller_spawner'.format(self.droneN_main) not in node_list:
                    break
            
            ### Drone rises to avoid obstacles 
            result = self.hector_pose_client(self.droneN,[pos[0],pos[1],self.height])
            rospy.loginfo("Client navigated")

            ### Read people seen by strategy drone         
            rospy.Subscriber('drone{}/people_detected_true'.format(self.droneN_main), Float64MultiArray, self.callback_people)
            rospy.sleep(0.2)

            final = rospy.Publisher('drone{}/people_detected'.format(self.droneN_main), Float64MultiArray, queue_size=1, latch=True)
            ### Precise position of the people
            if self.Goals.any():
                n = len(self.Goals)/4
                for person in range(n):
                    idx = person*4
                    xGoal = np.array([self.Goals[idx+1],self.Goals[idx+2], self.height])
                    result = self.hector_pose_client(self.droneN,xGoal)
                    rospy.loginfo("Client navigated")

                    rospy.sleep(0.2)
                    rospy.Subscriber('drone{}/people_seen'.format(self.droneN), Float64MultiArray, self.callback_seen)
                    rospy.sleep(0.2)
                    
                    n_xGoal = xGoal
                    step = 2

                    ### Search for person in the area
                    while(self.Goals[idx] not in self.seen):
                        n_xGoal = xGoal + np.array([step,0,0])
                        self.search(n_xGoal)
                        if self.Goals[idx] in self.seen:
                            break
                        n_xGoal = xGoal + np.array([0,step,0])
                        self.search(n_xGoal)
                        if self.Goals[idx] in self.seen:
                            break
                        n_xGoal = xGoal + np.array([-step,0,0])
                        self.search(n_xGoal)
                        if self.Goals[idx] in self.seen:
                            break
                        n_xGoal = xGoal + np.array([0,-step,0])
                        self.search(n_xGoal)
                        if self.Goals[idx] in self.seen:
                            break

                        step = step + 1  
                    xGoal = n_xGoal

                    ### Define more accurate position
                    right, left, up, down = 0.0, 0.0, 0.0, 0.0
                    left = self.get_border('x_axis', -1, self.droneN, xGoal, self.Goals[idx])
                    #print("Left side = ", left)
                    right = self.get_border('x_axis', 1, self.droneN, xGoal, self.Goals[idx])
                    #print("Right side = ",right)
                    up = self.get_border('y_axis', 1, self.droneN, xGoal, self.Goals[idx])
                    #print("Up side = ", up)
                    down = self.get_border('y_axis', -1, self.droneN, xGoal, self.Goals[idx])
                    #print("Down side = ", down)

                    ### Apply correction
                    y_correction = (down+up)/2
                    x_correction = (right+left)/2
                    total_correction = x_correction +  y_correction
                    
                    xGoal = xGoal + total_correction
                    print("self ",self.Goals)
                    print("xGoal", xGoal)
                    self.Goals[idx+1] = xGoal[0] 
                    self.Goals[idx+2] = xGoal[1]
                    self.Goals[idx+3] = xGoal[2]
                
            ### Plublishes new precise position
            True_Goals = Float64MultiArray()
            rospy.sleep(0.2)
            print("Goals", self.Goals)
            True_Goals.data = self.Goals
            final.publish(True_Goals)            

        except rospy.ROSInterruptException:
            print("Ended")

    ### Move drone by a step in search mode
    def search(self, n_xGoal):
        result = self.hector_pose_client(self.droneN, n_xGoal)
        rospy.loginfo("Client navigated")
        rospy.Subscriber('drone{}/people_seen'.format(self.droneN), Float64MultiArray, self.callback_seen)
        rospy.sleep(0.2)

    ### Move drone to desired position
    def hector_pose_client(self, droneN, xGoal):
        ### First enable motor service
        rospy.wait_for_service("/drone{}/enable_motors".format(droneN))
        enabler1 = rospy.ServiceProxy("/drone{}/enable_motors".format(droneN), EnableMotors)
        resp1 = enabler1(True)
        rospy.loginfo("Creating Action Client.")
        client = actionlib.SimpleActionClient('/drone{}/action/pose'.format(droneN), hector_uav_msgs.msg.PoseAction)
        rospy.loginfo("Client created.".format(droneN))
        client.wait_for_server()

        ### Create a random goal
        g = hector_uav_msgs.msg.PoseGoal()
        g.target_pose.header.frame_id = 'drone{}/world'.format(droneN)
        g.target_pose.pose.position.x = xGoal[0]
        g.target_pose.pose.position.y = xGoal[1]
        g.target_pose.pose.position.z = xGoal[2]

        rospy.loginfo("Sending goal")
        client.send_goal(g)
        client.wait_for_result()
        return(client.get_result())
    
    def get_border(self, direction, increment, droneN, xGoal, name):
        ### Defines the direction that will be incremented
        if direction == 'x_axis':
            step = np.array([increment,0,0])
        elif direction == 'y_axis':
            step = np.array([0,increment,0])
        
        ### Vector to be added to xGoal
        iteration = np.array([0,0,0])

        ### Loop for adding the steps i
        while (name in self.seen):
            print("Seen, ",self.seen)
            print("Name, ",name)
            rospy.sleep(0.1)
            rospy.Subscriber('drone{}/people_seen'.format(self.droneN), Float64MultiArray, self.callback_seen)
            iteration += step
            result = self.hector_pose_client(droneN,xGoal + iteration)
            #print("xGoal test",xGoal + iteration)
            rospy.loginfo("Client navigated")

        result = self.hector_pose_client(droneN,xGoal)
        rospy.loginfo("Client navigated")
        
        dist = iteration    
        return dist

if __name__ == "__main__":
    droneN = int(sys.argv[1])
    pos = drones_pos[int(sys.argv[1])-1]
    Correction = Correct_position(droneN, pos)
    
