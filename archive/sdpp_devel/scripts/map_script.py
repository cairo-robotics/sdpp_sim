#!/usr/bin/env python

import rospy
from nav_msgs.msg import MapMetaData
from sdpp_modules.sdpp_agents import WalkingPerson


if __name__ == '__main__':

	rospy.init_node("grid_node")

	MapMeta = MapMetaData()
	MapMeta.resolution = .1
	MapMeta.width = 200				#in cells
	MapMeta.height = 200 			#in cells
	MapMeta.origin.position.x = -10 #in what?
	MapMeta.origin.position.y = -10 #in what?
	MapMeta.origin.position.z = 0

	agent_1 = WalkingPerson(MapMeta, "person_1")
	agent_1.build_heading_graph((3, 4))
	agent_1.set_update_rate(20)

	try:
		while not rospy.is_shutdown():

			agent_1.update_step()

	except rospy.ROSInterruptException:
		pass

	print "end of trial"