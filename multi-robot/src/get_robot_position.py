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
        'block_a': Block('husky1'),
        'block_b': Block('husky2'),
        'block_c': Block('husky3'),

    }

    def __init__(self):
        rospy.init_node('get_robot_positions', anonymous=True)

    def get_positions(self):
        data = rospy.wait_for_message("/gazebo/model_states", ModelStates)
        for block in self._blockListDict.itervalues():
            blockName = str(block._name)
            if blockName in data.name:
                robot_index = data.name.index(blockName)
                current_robot = Robot(number=int(blockName[-1]),
                                      x=data.pose[robot_index].position.x,
                                      y=data.pose[robot_index].position.y,
                                      z=data.pose[robot_index].position.z)

                print("Robot number : {}".format(current_robot.name))
                print("Robot x : {}".format(current_robot.x))
                print("Robot y : {}".format(current_robot.y))
                print("Robot z : {}".format(current_robot.z))


if __name__ == '__main__':
    show = State()
    show.get_positions()

