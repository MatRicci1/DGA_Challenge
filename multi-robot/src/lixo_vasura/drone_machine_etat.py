#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from hector_uav_msgs.srv import EnableMotors
from std_msgs.msg    import Float64
from sensor_msgs.msg import Imu

####################################################################################################################################
# Finite-State Machine of the drone
# Defines the states of the drone and its actions
####################################################################################################################################

class drone_follow:

    def __init__(self, drone_N):
        # Starts a new node
        assert int(drone_N) in {1, 2, 3, 4, 5, 6, 7, 8, 9, 10}
        rospy.init_node('machine_etat{}'.format(drone_N))
        self.tracked = False
        self.ang = 0
        self.b = 0
        self.State = "State_1"
        self.distance_z = 10
        self.distance_x = 20
        self.person_flag = 0
        self.machine()

    def machine(self):
        # check if motors are on
        if self.motor_on():
            velocity_publisher = rospy.Publisher('drone{}/cmd_vel'.format(drone_N), Twist, queue_size=1, latch=True)

            vel_msg = Twist()

            # Setting the current time for distance calculus
            try:
                while 1:
                    if self.State == "State_1":
                        rospy.Subscriber('Window', Float64, self.callback)
                        
                        vel_msg.linear.x = 0
                        vel_msg.linear.y = 0
                        vel_msg.linear.z = 0.05

                        vel_msg.angular.x = 0
                        vel_msg.angular.y = 0
                        vel_msg.angular.z = 0
                        velocity_publisher.publish(vel_msg)
                        if self.ang != 0:
                            self.State = "State_2" 

                    elif self.State == "State_2":
                        print(self.ang)
                        # Loop to move the turtle in an specified distance
                        if (3 < abs(self.ang)) and not rospy.is_shutdown():
                            rospy.Subscriber('Window', Float64, self.callback)
                            rospy.Subscriber('/drone{}/raw_imu'.format(drone_N), Imu, self.callback2)
                            vel_msg.linear.x = 0
                            vel_msg.linear.y = 0
                            vel_msg.linear.z = 0

                            vel_msg.angular.x = 0
                            vel_msg.angular.y = 0
                            #vel_msg.angular.z = -(self.ang/abs(self.ang))*2
                            vel_msg.angular.z = -self.ang*0.15
                            #print(vel_msg)
                            print(vel_msg.angular.z)
                            # Publish the velocity
                            velocity_publisher.publish(vel_msg)
                        else:
                            vel_msg.angular.z = -2*vel_msg.angular.z
                            velocity_publisher.publish(vel_msg)
                            self.State = "State_3"

                    elif self.State == "State_3":
                        print(self.State)
                        # Loop to move the turtle in an specified distance
                        current_distance = 0
                        vel_msg.linear.z = 7
                        vel_msg.angular.z =  0
                        t0 = rospy.Time.now().to_sec()
                        print(vel_msg)
                        print(vel_msg.angular.z)
                        while (current_distance < self.distance_z) and not rospy.is_shutdown():
                            # Publish the velocity
                            velocity_publisher.publish(vel_msg)
                            # Takes actual time to velocity calculus
                            t1 = rospy.Time.now().to_sec()
                            # Calculates distancePoseStamped
                            current_distance =  vel_msg.linear.z * (t1 - t0)
                            #print(current_distance)
                            #print(self.distance_z)
                        # After the loop, stops the robot
                        vel_msg.linear.x = 0.0
                        vel_msg.linear.y = 0.0
                        vel_msg.linear.z = 0.05
                        # Force the robot to stop
                        velocity_publisher.publish(vel_msg)
                        self.State = "State_4"

                    elif self.State == "State_4":
                        print(self.State)
                        # Loop to move the turtle in an specified distance
                        current_distance = 0
                        vel_msg.linear.z = 0.0
                        vel_msg.linear.x = 7
                        t0 = rospy.Time.now().to_sec()
                        while not self.person_flag and not rospy.is_shutdown():
                            print(self.person_flag)
                            rospy.Subscriber('Person_flag', Float64, self.callback_person)
                            # Publish the velocity
                            velocity_publisher.publish(vel_msg)                      
                        # After the loop, stops the robot
                        vel_msg.linear.x = -1.5*vel_msg.linear.x
                        vel_msg.linear.y = 0.0
                        vel_msg.linear.z = 0.0
                        # Force the robot to stop
                        velocity_publisher.publish(vel_msg)
                        self.State = "State_5"

                    elif self.State == "State_5":
                        print(self.State)
                        vel_msg.linear.x = 0.0
                        vel_msg.linear.y = 0.0
                        vel_msg.linear.z = 0.05
                        # Force the robot to stop
                        velocity_publisher.publish(vel_msg)
                        print("Acabou")
            except KeyboardInterrupt:
                print('Interrupted')
                vel_msg.linear.x = 0.0
                vel_msg.linear.y = 0.0
                vel_msg.linear.z = 0.0
                # Force the robot to stop
                velocity_publisher.publish(vel_msg)
                print("exit")
                sys.exit(0)

    def callback(self, data):
        self.ang = data.data
    
    def callback_person(self, data):
        self.person_flag = data.data

    def callback2(self, data):
        self.b = data
        print(data)
        
    # Rosservice function to turn on the drone motors
    def motor_on(self):
        rospy.wait_for_service('drone{}/enable_motors'.format(drone_N))
        try:
            print("aqui")
            motor_on = rospy.ServiceProxy('drone{}/enable_motors'.format(drone_N), EnableMotors, True)
            turn_on = motor_on(True)
            return turn_on
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)


if __name__ == '__main__':
    # Receiveing the user's input
    print("Let's move your drone")
    drone_N = input("Select a drone to move. (Options 1, 2, 3, 4, 5, 6, 7, 8, 9, 10): ")
    # Testing our function
    follower = drone_follow(drone_N)






