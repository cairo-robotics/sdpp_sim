#!/usr/bin/env python3


from sdpp_explore.traj_analyzer import TrajAnalyzer

from nav_msgs.srv import GetMap

import rospy


def get_static_map():
    rospy.wait_for_service("/static_map")

    static_map_srv = rospy.ServiceProxy("/static_map", GetMap)
    get_map_msg= static_map_srv()

    static_map = get_map_msg.map

    return static_map


if __name__ == '__main__':

    #TODO (45) load in config data from yaml

    rospy.init_node("traj_analyzer_test")
    
    rospy.loginfo("waiting for static map")
    static_map = get_static_map()
    rospy.loginfo("got static map")

    test_analyzer = TrajAnalyzer(static_map)



    print("yay I did it")