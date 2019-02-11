#!/usr/bin/env python

import rospy
import numpy as np
import math
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState

depth = 3

class ConfigSpaceGraph(object):

	def __init__(self, map_meta_data):

		self.MapMetaData = map_meta_data

		x_cells = map_meta_data.info.width
		y_cells = map_meta_data.info.height

		self.occupancy_grid = np.zeros((x_cells , y_cells))


	def coordinate_to_array(self, x, y):
		return 



class HeadingGraph(object):

	def __init__(self, map_meta_data):

		self.MapMetaData = map_meta_data
		self.GoalCoordinate = [0]*2

		x_cells = map_meta_data.width
		y_cells = map_meta_data.height

		self.heading_graph = np.zeros((x_cells , y_cells))

	def _array_cord_tf(self, m, n):
		"""
		take a array index and transform to coordinate location
		:param m: index location
		:param n: index location
		:return: coordinate list
		"""
		x = n - self.MapMetaData.origin.position.x
		y = m - self.MapMetaData.origin.position.y

		return [x, y]

	def _cord_array_tf(self,x, y):
		"""
		take coordinate location and transform to array index
		:param x: coordinate location
		:param y: coordinate location
		:return: index list
		"""
		m = y + self.MapMetaData.origin.position.y
		n = x + self.MapMetaData.origin.position.x
		return m, n

	def get_heading_from_coordinate(self, x, y):

		m , n = self._cord_array_tf(x, y)

		return self.heading_graph[m, n]

	def get_x_y_magnitude(self, x, y):

		theta = self.get_heading_from_coordinate(x, y)

		x = math.cos(theta)
		y = math.sin(theta)

		return x, y



	def build_heading_graph(self, goal_point):

		self.GoalCoordinate = goal_point

		for index, data in np.ndenumerate(self.heading_graph):
			x, y = self._array_cord_tf(index[0], index[1])

			x = self.GoalCoordinate[0] - x
			y = self.GoalCoordinate[1] - y

			self.heading_graph[index] = math.atan2(y, x)




if __name__ == '__main__':

	rospy.init_node("grid_node")

	person_1_pub = rospy.Publisher("/person_1/walk_goal", Pose, queue_size = 1)

	rospy.wait_for_service("/gazebo/get_model_state")
	person_1_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

	MapMeta = MapMetaData()
	MapMeta.resolution = 1
	MapMeta.width = 10
	MapMeta.height = 10
	MapMeta.origin.position.x = 2
	MapMeta.origin.position.y = 2
	MapMeta.origin.position.z = 0

	goal_one = HeadingGraph(MapMeta)

	goal_one.build_heading_graph((1, 1))


	print goal_one.get_heading_from_coordinate(-2 , 2)

	rate = rospy.Rate(10)
	try:
		#publish map
		while not rospy.is_shutdown():

			person_1_data = person_1_service("person_1", "")

			x = int(person_1_data.pose.position.x)
			y = int(person_1_data.pose.position.y)

			pose = Pose()

			xv, yv = goal_one.get_x_y_magnitude(x, y)

			pose.position.x = xv*.2
			pose.position.y = yv*.2
			person_1_pub.publish(pose)
			rate.sleep()



	except rospy.ROSInterruptException:
		pass

	print "end of trial"




