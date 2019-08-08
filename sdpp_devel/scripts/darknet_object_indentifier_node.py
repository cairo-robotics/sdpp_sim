#!/usr/bin/env python

import rospy

from sdpp_devel.object_identifier import darknetObjectIden


if __name__ == '__main__':

    rospy.init_node("darknet_translate")

    object_list = rospy.get_param("object_list")

    print object_list

    print("waiting")
    refresh_rate = 1

    darknet_obj = darknetObjectIden(object_list, refresh_rate)
