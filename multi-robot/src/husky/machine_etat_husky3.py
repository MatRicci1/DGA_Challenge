#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from std_msgs.msg    import Float64
from sensor_msgs.msg import Imu
from std_msgs.msg    import Float64MultiArray

class husky_search:

    def __init__(self):
        # Starts a new node
        rospy.init_node('machine_etat_husky1')
	#----------------------------
        self.state = "State_1"
        self.map_lenght  = 60.0
        self.person = []
        self.person2 = []
	self.goalX = 0.0
	self.goalY = 0.0
	self.count_people = 0
	self.count_people2 = 0
        self.machine()
	

    def machine(self):

	    client = actionlib.SimpleActionClient('husky3/move_base',MoveBaseAction)
	    rospy.loginfo("Waiting for client server")
	    client.wait_for_server()

	        # Advancing to a closer point to take advantage of the time used by the drones to 
	    try:
		while 1:
		    if self.state == "State_1":
			
			#Read the list of the drones with the position of the persons detected
			#if new objective in list
			#get position in :		
    
		        rospy.Subscriber('/drone1/people_detected_true', Float64MultiArray, self.callback_person)
                        rospy.Subscriber('/drone2/people_detected_true', Float64MultiArray, self.callback_person2)

			if len(self.person) != 4*self.count_people:
			   self.count_people+=1
			   
		           self.goalX = self.person[-3]
		           self.goalY = self.person[-2]
			   self.state = "State_2"

			if len(self.person2) != 4*self.count_people2:
			   self.count_people2+=1
			   
		           self.goalX = self.person2[-3]
		           self.goalY = self.person2[-2]
			   self.state = "State_2"
		# Change state to go to the postition of the person in the list
		    elif self.state == "State_2":

			goal = MoveBaseGoal()
	    		goal.target_pose.header.frame_id = "husky3_tf/map"
	    		goal.target_pose.header.stamp = rospy.Time.now()
	    		goal.target_pose.pose.position.x = self.goalX
	    		goal.target_pose.pose.position.y = self.goalY
	    		goal.target_pose.pose.orientation.w = 1.0

		        client.send_goal(goal)
		        wait = client.wait_for_result()
		        if not wait:
			   rospy.logerr("Action server not available!")
			   rospy.signal_shutdown("Action server not available!")
		        else:
			   rospy.loginfo("Goal execution done!")
			   self.state = "State_1"

	      
	    except KeyboardInterrupt:
		print('Interrupted')
		# Force the robot to stop
		print("exit")
		sys.exit(0)
    
    def callback_person(self, data):
        self.person = data.data

    def callback_person2(self, data):
        self.person2 = data.data

        
if __name__ == '__main__':
    # Testing our function
    follower = husky_search()






