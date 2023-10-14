#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from hector_uav_msgs.srv import EnableMotors

def move(drone_N):
    # Starts a new node
    assert int(drone_N) in {1, 2, 3, 4, 5, 6, 7, 8, 9, 10}

    rospy.init_node('move_drone{}'.format(drone_N))
    # check if motors are on
    if motor_on():

        velocity_publisher = rospy.Publisher('drone{}/cmd_vel'.format(drone_N), Twist, queue_size=1, latch=True)

        vel_msg = Twist()

        # Receiveing the user's input
        speed_x = input("Input your linear velocity in x: ")
        speed_y = input("Input your linear velocity in y: ")
        speed_z = input("Input your vertical_velocity: ")
        distance = input("Type your distance: ")

        vel_msg.linear.x = speed_x
        vel_msg.linear.y = speed_y
        vel_msg.linear.z = speed_z

        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
	while not rospy.is_shutdown():

		# Setting the current time for distance calculus
		t0 = rospy.Time.now().to_sec()
		current_distance = 0
		try:
		    # Loop to move the turtle in an specified distance
		    while (current_distance < distance) and not rospy.is_shutdown():
		        # Publish the velocity
		        velocity_publisher.publish(vel_msg)
		        # Takes actual time to velocity calculus
		        t1 = rospy.Time.now().to_sec()
		        # Calculates distancePoseStamped
		        current_distance = speed_z * (t1 - t0)
		    # After the loop, stops the robot
		    vel_msg.linear.x = 0.0
		    vel_msg.linear.y = 0.0
		    vel_msg.linear.z = 0.0
		    # Force the robot to stop
		    velocity_publisher.publish(vel_msg)
		except KeyboardInterrupt:
		    print('Interrupted')
		    vel_msg.linear.x = 0.0
		    vel_msg.linear.y = 0.0
		    vel_msg.linear.z = 0.0
		    # Force the robot to stop
		    velocity_publisher.publish(vel_msg)
		    sys.exit(0)
	else:
            return 0


# Rosservice function to turn on the drone motors
def motor_on():
    rospy.wait_for_service('drone{}/enable_motors'.format(drone_N))
    try:
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
    move(drone_N)
