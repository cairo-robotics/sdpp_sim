#!/usr/bin/env python

import rospy
import numpy as np

from pprint import pprint, pformat
from PIL import Image

from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap


class LoadMap(object):
    def __init__(self, file_location=None):

        self.static_odom_map = None

        rospy.loginfo(rospy.get_name() + ": getting map from map server")

        if file_location is None:
            self.static_odom_map = self.load_from_static_map_srv()
        else:
            self.static_odom_map = self.load_from_file_location_map(file_location)

    @staticmethod
    def load_from_static_map_srv(srv_name='/static_map'):
        rospy.wait_for_service(srv_name)
        static_map_srv = rospy.ServiceProxy(srv_name, GetMap)

        try:
            static_odom_msg = static_map_srv().map
            rospy.loginfo(rospy.get_name() + ": received static map")

        except rospy.ServiceException as exc:
            rospy.logerror(rospy.get_name() + ": unable to get static map" + str(exc))
            return None

        return static_odom_msg

    @staticmethod
    def load_from_file_location_map(file_location):

        rospy.logerror(rospy.get_name() + ": load map directly not finished")
        im_frame = Image.open('block_room.png')
        np_frame = np.array(im_frame)
        np_frame = np_frame[:, :, 0]
        np_frame = np.rot90(np_frame, 3)

        return np_frame

    def gridworld_format_data(self):

        width = self.static_odom_map.info.width
        height = self.static_odom_map.info.height

        numpy_array = np.asarray(self.static_odom_map.data)
        numpy_array = numpy_array.reshape(width, height)

        kwags = {"array": numpy_array, "world_bounds_rows": width, "world_bounds_cols": height}
        return kwags

    def __str__(self):
        return pformat(self.static_odom_map)


class MoveBaseAgents(object):

    def __init__(self):
        pass