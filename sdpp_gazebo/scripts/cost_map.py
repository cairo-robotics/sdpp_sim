#!/usr/bin/env python
import rospy

from nav_msgs.msg import MapMetaData
from sdpp_modules.sdpp_maps import PredictiveCostMap
from sdpp_modules.sdpp_tools import sdpp_map_meta


if __name__ == '__main__':

    rospy.init_node("test")

    # instantiate the map meta data
    MapMeta = sdpp_map_meta("map_meta")
    
    print MapMeta

    test = PredictiveCostMap(MapMeta, "person_1", "odom")

    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            test.update_step()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

    print "end of script"