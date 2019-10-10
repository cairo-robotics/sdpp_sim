#!/usr/bin/env python

import rospy
from nav_msgs.srv import GetMap

from nav_msgs.msg import OccupancyGrid



if __name__ == '__main__':

    rospy.init_node("repub_map_server")

    get_map = rospy.ServiceProxy('/static_map', GetMap)

    pub = rospy.Publisher("/test", OccupancyGrid, queue_size=1 )


    while not rospy.is_shutdown():

        rospy.sleep(1)

        repub_map = get_map()

        map_tuple = [1] * len(repub_map.map.data)


        repub_map.map.data = tuple(map_tuple)


        pub.publish(repub_map.map)

        print "test"

