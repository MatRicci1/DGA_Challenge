#! /usr/bin/env python

from gazebo_msgs.srv import GetModelState
import rospy
from gazebo_msgs.msg import ModelStates


class Block:
    def __init__(self, name):
        self._name = name


class Robot:
    def __init__(self, number, x, y, z):
        self.name = number
        self.x = x
        self.y = y
        self.z = z


class State:

    _blockListDict = {
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

    def __init__(self):
        rospy.init_node('get_drone_positions', anonymous=True)

    def get_positions(self):
        data = rospy.wait_for_message("/gazebo/model_states", ModelStates)
        for block in self._blockListDict.itervalues():
            blockName = str(block._name)
            if blockName in data.name:
                drone_index = data.name.index(blockName)
                current_drone = Robot(number=int(blockName[-1]),
                                      x=data.pose[drone_index].position.x,
                                      y=data.pose[drone_index].position.y,
                                      z=data.pose[drone_index].position.z)

                print("Drone number : {}".format(current_drone.name))
                print("Drone x : {}".format(current_drone.x))
                print("Drone y : {}".format(current_drone.y))
                print("Drone z : {}".format(current_drone.z))

                return current_drone.x,current_drone.y,current_drone.z


if __name__ == '__main__':
    show = State()
    [x,y,z] = show.get_positions()
