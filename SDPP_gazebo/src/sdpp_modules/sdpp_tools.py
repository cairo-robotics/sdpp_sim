#!/usr/bin/env python

import rospy
from nav_msgs.msg import MapMetaData

def sdpp_map_meta(param_name):

    map_meta = MapMetaData()
    try:
        param_data = rospy.get_param(param_name)

        map_meta.map_load_time = 0
        map_meta.resolution = param_data['resolution']
        map_meta.width = param_data['width']
        map_meta.height = param_data['height']

        map_meta.origin.position.x = param_data['origin']['position']['x']
        map_meta.origin.position.y = param_data['origin']['position']['y']
        map_meta.origin.position.z = param_data['origin']['position']['z']

        map_meta.origin.orientation.w = param_data['origin']['orientation']['w']
        map_meta.origin.orientation.x = param_data['origin']['orientation']['x']
        map_meta.origin.orientation.y = param_data['origin']['orientation']['y']
        map_meta.origin.orientation.z = param_data['origin']['orientation']['z']

        return map_meta

    except Exception as e:
        rospy.logwarn("get map meta failed")
        return None


if __name__ == "__main__":

    rospy.init_node("test_tools")

    print sdpp_map_meta('mp_meta')

