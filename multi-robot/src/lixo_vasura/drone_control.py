#! /usr/bin/env python

import rospy
import time
import random
from hector_uav_msgs.srv import EnableMotors
from geometry_msgs.msg import PoseStamped
from std_msgs.msg    import Float64
import actionlib
import hector_uav_msgs.msg  
from Env_strategy import *
import sys

###############################################################################################################################################################
# Drone Control
# Takes the main strategy of the environment and executes the cotnrol
# Returns the first estimate position of the persons found
###############################################################################################################################################################

def hector_pose_client(droneN, xGoal):
    # First enable motor service
    rospy.wait_for_service("/drone{}/enable_motors".format(droneN))
    enabler1 = rospy.ServiceProxy("/drone{}/enable_motors".format(droneN), EnableMotors)
    resp1 = enabler1(True)

    rospy.loginfo("Creating Action Client.")
    client = actionlib.SimpleActionClient('/drone{}/action/pose'.format(droneN), hector_uav_msgs.msg.PoseAction)
    rospy.loginfo("Client created.".format(droneN))

    client.wait_for_server()

    # Create a random goal
    g = hector_uav_msgs.msg.PoseGoal()
    g.target_pose.header.frame_id = 'drone{}/world'.format(droneN)
    g.target_pose.pose.position.x = xGoal[0]
    g.target_pose.pose.position.y = xGoal[1]
    g.target_pose.pose.position.z = xGoal[2]

    rospy.loginfo("Sending goal")
    client.send_goal(g)
    client.wait_for_result()
    return(client.get_result())

if __name__ == "__main__":
    try:
        droneN = int(sys.argv[1])
        #drone_Position = drones_pos[int(sys.argv[1])-1]
        xGoals = drones_traj_goal[int(sys.argv[1])-1]
        print(xGoals)
        #drone_GPS = drones_gps[int(sys.argv[1])-1]
        rospy.init_node('drone_explorer{}'.format(droneN))
        final = rospy.Publisher('drone{}/final'.format(droneN), Float64, queue_size=1, latch=True)
        rospy.sleep(0.1)
        final.publish(0.0)
        for xGoal in xGoals:
            #print(xGoal)
            result = hector_pose_client(droneN,xGoal)
            #print(result)
            rospy.loginfo("Client navigated")
        
        print("terminou")
        final.publish(1.0)

    except rospy.ROSInterruptException:
        print("bla")
        #print("program interrupted before completion", file=  sys.stderr)