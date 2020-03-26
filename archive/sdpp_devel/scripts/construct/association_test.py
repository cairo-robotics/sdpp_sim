#!/usr/bin/env python

import rospy
import rospkg

from sdpp_devel.object_association import ObjectAssociation




if __name__ == '__main__':

    rospy.init_node("test_node")

    object_list = rospy.get_param("object_list")

    #object_list= ["cup", "bottle", "objC", "objD", "j", "s", "d", "k"]

    test = ObjectAssociation.empty_inference_obj(object_list)

    test.add_assoc_inst("tie", "couch")

    rospack = rospkg.RosPack()

    path = rospack.get_path('sdpp_devel')
    path = path + "/scripts/data/test.pickle"
    test.pickle_me(path)

    test.display_graph()