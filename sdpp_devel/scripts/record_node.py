#!/usr/bin/env python

import rospy
from sdpp_devel.data_parse import PeopleRecorder

if __name__ == '__main__':
    rospy.init_node("test_recorder")

    test = PeopleRecorder("data/sim_test_dict.pickle", picture_tag=False)

    rospy.spin()

