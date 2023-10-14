#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState
from config import kill_zone_1, kill_zone_2
from gazebo_msgs.msg import ModelStates


import time
import os
import random
import time


class Block:
    def __init__(self, name, relative_entity_name):
        self._name = name
        self._relative_entity_name = relative_entity_name


class Drone:
    def __init__(self, number, x, y, z):
        self.name = number
        self.x = x
        self.y = y
        self.z = z


class DroneShooter:
    def __init__(self, drone_N):

        self.drone_N = drone_N
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
        time_before_damage_spread = 0  # 0 to 5s
        time.sleep(time_before_damage_spread)

        # Finally,neutralize state publisher and camera vision

        os.system("rosnode kill /drone{}/robot_state_publisher".format(self.drone_N))
        os.system("rosnode kill /drone{}_camera_vision".format(self.drone_N))
        os.system("rosnode kill /drone{}/controller_spawner".format(self.drone_N))



    def damage_motors(self):
        os.system("rosnode kill move_drone{}".format(self.drone_N))

        vel_msg = Twist()
        vel_msg.linear.z = 0
        vel_msg.linear.x = 40
        vel_msg.linear.y = 30
        vel_msg.angular.x = 15
        vel_msg.angular.y = 15
        vel_msg.angular.z = 15

        velocity_publisher = rospy.Publisher('drone{}/cmd_vel'.format(self.drone_N), Twist, queue_size=1)
        velocity_publisher.publish(vel_msg)


class KillZones:
    _dronesDict = {
        'block_a': Block('drone1', 'base_link'),
        'block_b': Block('drone2', 'base_link'),
        'block_c': Block('drone3', 'base_link'),
        'block_d': Block('drone4', 'base_link'),
        'block_e': Block('drone5', 'base_link'),
        'block_f': Block('drone6', 'base_link'),
        'block_g': Block('drone7', 'base_link'),
        'block_h': Block('drone8', 'base_link'),
        'block_i': Block('drone9', 'base_link'),
        'block_j': Block('drone10', 'base_link')
    }

    def __init__(self):

        self.innactive_drones = []
        self.drone_N = 0
        self.drones = []
        rospy.init_node('kill_zones', anonymous=True)
        time.sleep(1)
        rospy.Subscriber("/gazebo/model_states", ModelStates,
                         callback=self.get_drone_positions, queue_size=10)
        self.kill_zones = [kill_zone_1, kill_zone_2]
        self.activate_kill_zones()

    def activate_kill_zones(self):
        while not rospy.is_shutdown():
            self.check_for_drones_in_dead_zones()

    def check_for_drones_in_dead_zones(self):
        for drone in self.drones:
            if self.is_drone_in_dead_zone(drone.x,
                                          drone.y,
                                          drone.z):
                self.innactive_drones.append(drone.name)
                self.drone_in_dead_zone(drone.name)
            else:
                pass

    def get_drone_positions(self, data):
        self.drones = []
        for block in self._dronesDict.itervalues():
            blockName = str(block._name)
            if blockName in data.name and int(blockName[-1]) not in self.innactive_drones:
                drone_index = data.name.index(blockName)
                current_drone = Drone(number=int(blockName[-1]),
                                      x=data.pose[drone_index].position.x,
                                      y=data.pose[drone_index].position.y,
                                      z=data.pose[drone_index].position.z)
                self.drones.append(current_drone)
                # print('\n')
                # print(blockName)
                # print("Position x: " + str(current_drone.x))
                # print("Position y: " + str(current_drone.y))
                # print("Position z: " + str(current_drone.z))

    def is_drone_in_dead_zone(self, x,y,z):
        for kill_zone in self.kill_zones:
            if kill_zone["x0"] < x < kill_zone["x1"] and \
                    kill_zone["y0"] < y < kill_zone["y1"]:

                return True

        return False

    def drone_in_dead_zone(self, drone_number):
        print("Drone {} inside a kill zone".format(drone_number))
        kill_drone = DroneShooter(drone_number)

        kill_drone.shoot_drone()


if __name__ == '__main__':
    try:
        k = KillZones()

    except rospy.ROSInterruptException:
        print('Interruption')
        pass
