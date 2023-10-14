#! /usr/bin/env python
import rospy
import time
import random
from hector_uav_msgs.srv import EnableMotors
from geometry_msgs.msg import PoseStamped
import actionlib
import hector_uav_msgs.msg
from std_msgs.msg    import Float64
from Env_strategy import *
import sys

#################################################################################################################
# Node to test drone position control
# Moves the drone to a specified position
#################################################################################################################

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
        #droneN = int(sys.argv[1])
        droneN = 1
        #drone_Position = drones_   pos[int(sys.argv[1])-1]
        #xGoals = [[-32, -32, 10],[0, -13, 10]]
        #xGoals = [[-32, -32, 10],[-32, -32, 0]]
        #xGoals = [[-32, -32, 0]]
        xGoals = [[-32, -32, 10]]
        #drone_GPS = drones_gps[int(sys.argv[1])-1]
        rospy.init_node('drone_explorer{}'.format(droneN))
        for xGoal in xGoals:
            result = hector_pose_client(droneN,xGoal)
            rospy.loginfo("Client navigated")
        
        
    except rospy.ROSInterruptException:
        print("Interrupeted")
       #print("program interrupted before completion", file=  sys.stderr)