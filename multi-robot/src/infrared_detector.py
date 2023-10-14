#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelStates
from std_msgs.msg    import Float64MultiArray
from std_msgs.msg    import Float64
from config import drone_angles
from geometry_msgs.msg import PoseStamped
from Env_strategy import *
import sys
import numpy as np
import time
import os
import random
import time

#################################################################################################################
# Node that identifies people by infrared sensor
# Find hostiles below drone (10 m limit)
# Publishes the position of a person found by the infrared sensor
#################################################################################################################

class Block:
    def __init__(self, name):
        self._name = name

class Drone:
    def __init__(self, number, x, y, z):
        self.name = number
        self.x = x
        self.y = y
        self.z = z

class Person:
    def __init__(self, number, x, y, z):
        self.name = number
        self.x = x
        self.y = y
        self.z = z

class Visible:
    def __init__(self, person, drone):
        self.drone = drone
        self.person = person

class InfraRed:
    _peopleDict = {
        'block_a': Block('hostile_1'),
        'block_b': Block('hostile_2'),
        'block_c': Block('hostile_3'),
        'block_d': Block('hostile_4'),
        'block_e': Block('hostile_5'),
        'block_f': Block('hostile_6'),
        'block_g': Block('hostile_7'),
        'block_h': Block('hostile_8')
    }

    _dronesDict = {
        'block_a': Block('drone1'),
        'block_b': Block('drone2'),
        'block_c': Block('drone3'),
        'block_d': Block('drone4'),
        'block_e': Block('drone5'),
        'block_f': Block('drone6'),
        'block_g': Block('drone7'),
        'block_h': Block('drone8'),
        'block_i': Block('drone9'),
        'block_j': Block('drone10')
    }

    # callback for drone pose
    def callback_sensor_pose(self, data):
        # x-axis
        self.drone_nav_pos[0] = data.pose.position.x + self.drone_pos[0] 
        # y-axis
        self.drone_nav_pos[1] = data.pose.position.y + self.drone_pos[1] 
        # z-axis
        self.drone_nav_pos[2] = data.pose.position.z + self.drone_pos[2]

    # callback for people seen
    def callback_seen(self, data):
        self.seen = data.data


    def __init__(self, drone_number, drone_pos):

        rospy.init_node('infraRed{}'.format(drone_number), anonymous=True)
        # Starts a new node
        self.drone_number = drone_number
        self.drones = []
        self.people = []
        self.people_coord = []
        self.people_list_total = [0]
        self.drone_pos = drone_pos
        
        self.drone_nav_pos = [0,0,0]
        self.people_visible_by_drones = []
        
        rospy.Subscriber("/gazebo/model_states", ModelStates,
                         callback=self.update_positions, queue_size=10)

        time.sleep(1)
        while not rospy.is_shutdown():
            try:
                rospy.sleep(0.1)
                rospy.Subscriber('drone{}/people_seen'.format(self.drone_number), Float64MultiArray, self.callback_seen)
                self.looking_for_people()
                
                #print(self.drone_pos)
            except KeyboardInterrupt:
                print('Interrupted')
                sys.exit(0)
        
    def get_infra_range(self, drone_height):
        width = 2.0 * drone_height * drone_angles["wfov"]/2.0
        height = 2.0 * drone_height * drone_angles["hfov"] / 2.0
        return width, height

    def looking_for_people(self):
        self.people_visible_by_drones = []

        if self.drone_number == 0:
            for drone in self.drones:
                for person in self.people:
                    width, lenght = self.get_infra_range(drone.z)
                    if (np.abs(drone.x - person.x) < width/2.0) and \
                            (np.abs(drone.y - person.y) < lenght / 2.0) and \
                            drone.z < 10.0:

                        self.people_visible_by_drone(person, drone)

        else:
            for drone in self.drones:
                for person in self.people:
                    width, lenght = self.get_infra_range(drone.z)
                    if (np.abs(drone.x - person.x) < width / 2.0) and \
                            (np.abs(drone.y - person.y) < lenght / 2.0) and \
                            drone.z < 10.0 and self.drone_number == drone.name:
                        self.people_visible_by_drone(person, drone)

        ### publishing topics of people already detected
        detected_publisher = rospy.Publisher('drone{}/people_detected_true'.format(self.drone_number), Float64MultiArray, queue_size=1, latch=True)
        
        ### publishing topics of people detected in the exact instant
        seen_publisher = rospy.Publisher('drone{}/people_seen'.format(self.drone_number), Float64MultiArray, queue_size=1, latch=True)
        
        ### publishing topics of a person flag for the finite state machine
        person_flag = rospy.Publisher('drone{}/Person_flag'.format(self.drone_number), Float64, queue_size=1, latch= True)

        ### subscribing topic of a person pose
        rospy.Subscriber('drone{}/nav/sensor_pose'.format(self.drone_number), PoseStamped, self.callback_sensor_pose)

        ### Appending people seen to a list
        people_seen = []
        if self.people_visible_by_drones:
            for people_seen_by_drone in self.people_visible_by_drones:
                self.update_people_position(people_seen_by_drone.person.name, self.drone_nav_pos)
                people_seen.append(people_seen_by_drone.person.name)
                person_flag.publish(1)
                #print("Hostile number {} is being seen by drone number {}".
                      #format(people_seen_by_drone.person.name, people_seen_by_drone.drone.name))
        else:          
            person_flag.publish(0)

        ### List of people
        self.people_list = self.make_list()

        ### Publishing the topic of people seen in the exact moment
        pp_seen = Float64MultiArray()
        pp_seen.data = people_seen
        seen_publisher.publish(pp_seen)
        
         ### Publishing the topic of people detected
        pp_det = Float64MultiArray()
        pp_det.data = list(self.people_list)
        detected_publisher.publish(pp_det)


    def update_positions(self, data):
        self.get_people_positions(data)
        self.get_drone_positions(data)

    def update_people_position(self, person_name, coord):
        Person_j = Person(float(person_name), coord[0], coord[1], coord[2])
        if self.people_coord:
            for Person_i in self.people_coord:
                if Person_i.name == Person_j.name:
                    break
            else:
                self.people_coord.append(Person_j)
        else:
            self.people_coord.append(Person_j)     
    
    ### makes the list of people and their position
    def make_list(self):
        list_p = []
        if self.people_coord:
            for Person_i in self.people_coord:
                if Person_i.name not in self.people_list_total:
                    list_p_info = [float(Person_i.name), Person_i.x, Person_i.y, Person_i.z]
                    list_p = list_p + list_p_info
                    list(self.people_list_total)[0] += 1
        return list_p

    def people_visible_by_drone(self, person, drone):
        visible = Visible(person=person, drone=drone)
        self.people_visible_by_drones.append(visible)

    def get_drone_positions(self, data):
        self.drones = []
        for block in self._dronesDict.itervalues():
            blockName = str(block._name)
            if blockName in data.name:
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

    def get_people_positions(self, data):
        self.people = []
        for block in self._peopleDict.itervalues():
            blockName = str(block._name)
            if blockName in data.name:
                person_index = data.name.index(blockName)
                current_person = Person(number=int(blockName[-1]),
                                        x=data.pose[person_index].position.x,
                                        y=data.pose[person_index].position.y,
                                        z=data.pose[person_index].position.z)

                self.people.append(current_person)

                # print('\n')
                # print(blockName)
                # print("Position x: " + str(current_people.x))
                # print("Position y: " + str(current_people.y))
                # print("Position z: " + str(current_people.z))


if __name__ == '__main__':
    try:
        drone_N = int(sys.argv[1])
        drone_Position = drones_pos[int(sys.argv[1])-1]
        k = InfraRed(drone_N, drone_Position)

    except rospy.ROSInterruptException:
        print('Interruption')
        pass
