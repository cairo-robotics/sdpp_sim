#!/usr/bin/env python

import rospy
from sdpp_devel.data_parse import PeopleViewer

if __name__ == '__main__':

    rospy.init_node("test_viewer")

    test = PeopleViewer("data/sim_test_dict.pickle")

    test.print_graph(n_tracks=10)
    #test.carry_classify_all()

    #test.pickle_me()
