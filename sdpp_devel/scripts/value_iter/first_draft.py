#!/usr/bin/env python


import rospy

from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap

import numpy as np

import matplotlib as mpl
from matplotlib import pyplot




if __name__ == '__main__':

    rospy.init_node("value_itier_init")

    rospy.wait_for_service('/static_map')

    print "got map"

    mapProxy = rospy.ServiceProxy('/static_map', GetMap)

    try:
        occupancyGridMsg = mapProxy()

    except rospy.ServiceException as exc:
        print("service did not work")

    print occupancyGridMsg.map.info

    mapInfo = occupancyGridMsg.map.info



    numpyOccupancyGrid = np.empty((mapInfo.width, mapInfo.height))

    for i in range(0, mapInfo.width):
        for j in range (0, mapInfo.height):
            msgValue = occupancyGridMsg.map.data[i*mapInfo.width + j]
            if msgValue == -1:
                numpyOccupancyGrid[i][j] = 0
            elif msgValue == 0:
                numpyOccupancyGrid[i][j] = 0
            elif msgValue == 100:
                numpyOccupancyGrid[i][j] = -1
            else:
                print "unknown case" + str(msgValue)




    print mapInfo.width
    numpyOccupancyGrid[20][20] = 100



    img = pyplot.imshow(numpyOccupancyGrid)

    pyplot.colorbar(img)

    pyplot.show()



