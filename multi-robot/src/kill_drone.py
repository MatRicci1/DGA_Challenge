#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time
#!/usr/bin/env python
import os
import random
import time
# https://answers.ros.org/question/237862/rosnode-kill/

class DroneShooter:
    def __init__(self, drone_N):
        assert int(drone_N) in {1, 2, 3, 4, 5, 6, 7, 8, 9, 10}, "Please select a valid option"

        self.drone_N = drone_N
        rospy.init_node('kill_drone{}'.format(self.drone_N), anonymous=True)
        os.system("rosnode kill move_drone{}".format(self.drone_N))
        time.sleep(1)

    def shoot_drone(self):
        # First make the drone fall
        self.damage_motors()

        # The damage spreads and makes the CU out of service
        self.damage_control_unit()


    def damage_control_unit(self):
        """
        https://answers.ros.org/question/237862/rosnode-kill/
        """
        time_before_damage_spread = random.random() * 5  # 0 to 5s
        time.sleep(time_before_damage_spread)

        # Finally,neutralize state publisher and camera vision

        os.system("rosnode kill /drone{}/robot_state_publisher".format(self.drone_N))
    # os.system("rosnode kill /drone{}_camera_vision".format(self.drone_N))



    def damage_motors(self):
        vel_msg = Twist()
        vel_msg.linear.z = 0
        vel_msg.linear.x = 1
        vel_msg.linear.y = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 2

        velocity_publisher = rospy.Publisher('drone{}/cmd_vel'.format(self.drone_N), Twist, queue_size=1)
        velocity_publisher.publish(vel_msg)


if __name__ == '__main__':
    try:
        # Receiveing the user's input
        print("The following drone will be shot")
        drone_N = input("Select a drone to be shot. (Options 1, 2, 3, 4, 5, 6, 7, 8, 9, 10): ")
        # Testing our function
        drone_shooter = DroneShooter(drone_N)

        drone_shooter.shoot_drone()
        exit()
    except rospy.ROSInterruptException:
        print('Interruption')
        pass
