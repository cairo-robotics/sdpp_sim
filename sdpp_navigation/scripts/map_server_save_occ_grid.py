#!/usr/bin/env python3

import rospy
import yaml

from nav_msgs.srv import GetMap



rospy.init_node("test")

da_proxy = rospy.ServiceProxy("/static_map", GetMap)

filename = "narrow_3_goal_10x10_occupancy_grid.yaml"

data = da_proxy()
fp = open(filename, "w")

yaml.dump(data.map, fp)

fp.close()

fp = open(filename, "r")


data = yaml.full_load(fp)

print(data)