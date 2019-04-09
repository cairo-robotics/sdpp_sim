#!/usr/bin/env python

import rospy
import re

from sdpp_modules.sdpp_tools import sdpp_map_meta
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelState, GetModelProperties
from nav_msgs.msg import OccupancyGrid, MapMetaData

object_dict = {"cabinet": {"footprint": (.45, .45), "center": (.225, .225)},
               "table": {"footprint": (1.5, 0.8), "center": (.75, .4)}}


class ConfigSpaceMap(object):

    def __init__(self, map_meta):

        rospy.wait_for_service("/gazebo/get_model_state")
        self.MapMetaData = map_meta
        self.object_dict = object_dict
        self.get_properties_srv = rospy.ServiceProxy("/gazebo/get_model_properties", GetModelProperties)
        rospy.loginfo("GET_model_properties service active")

        self.object_names = []
        self.get_object_list()

        self.occupancy_grid = OccupancyGrid()

    def get_object_list(self):

        rospy.loginfo("waiting for world objects state list")
        world_states = rospy.wait_for_message("/gazebo/model_states", ModelStates)
        object_list = object_dict.keys()
        object_regex = "^" + "|".join(object_list)
        self.object_names = [name for name in world_states.name if re.match(object_regex, name)]

        rospy.loginfo("got world object names")

    def get_object_footprint(self, object):

        object_prop = self.object_dict[object]
        return object_prop


if __name__ == '__main__':

    rospy.init_node("config_space")

    map_meta_data = sdpp_map_meta("map_meta")

    test = ConfigSpaceMap(map_meta_data)

    test.get_object_footprint(test.object_names[0])

