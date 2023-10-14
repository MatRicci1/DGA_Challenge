#! /usr/bin/env python

from gazebo_msgs.srv import GetModelState
import rospy

class Block:
    def __init__(self, name, relative_entity_name):
        self._name = name
        self._relative_entity_name = relative_entity_name

class State:

    _blockListDict = {
        'block_a': Block('person_walking_0', 'link'),
        'block_b': Block('person_walking_1', 'link'),
        'block_c': Block('person_standing_2', 'link'),
        'block_d': Block('person_standing_3', 'link'),
        'block_e': Block('hostile_1', 'link'),
        'block_f': Block('hostile_2', 'link'),
        'block_g': Block('hostile_3', 'link'),
        'block_h': Block('hostile_4', 'link'),
        'block_k': Block('hostile_5', 'link'),
        'block_l': Block('hostile_6', 'link'),
        'block_m': Block('hostile_7', 'link'),
        'block_n': Block('hostile_8', 'link')
    }

    def show_gazebo_models(self):
        try:
            model_coordinates= rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            for block in self._blockListDict.itervalues():
                blockName = str(block._name)
                resp_coordinates = model_coordinates(blockName, block._relative_entity_name)
                print('\n')
                print('State success = ', resp_coordinates.success)
                print(blockName)
                print("Position x: " + str(resp_coordinates.pose.position.x))
                print("Position y: " + str(resp_coordinates.pose.position.y))
                print("Position z: " + str(resp_coordinates.pose.position.z))
        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed: {0}".format(e))
            raise e

if __name__ == '__main__':
    show = State()
    show.show_gazebo_models()