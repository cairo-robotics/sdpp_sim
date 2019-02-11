#!/usr/bin/env python

import rospy
import numpy as np
import math
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState

import matplotlib.pyplot as plt



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
		"""
		create the basic heading graph necessities
		:param map_meta_data: meta data of gra
		"""

		self.MapMetaData = map_meta_data
		self.GoalCoordinate = [0]*2

		# set array around origin of zero
		x_width = map_meta_data.width*map_meta_data.resolution
		x_max = x_width + map_meta_data.origin.position.x
		x_min = x_max - x_width

		y_height = map_meta_data.height*map_meta_data.resolution
		y_max = y_height + map_meta_data.origin.position.y
		y_min = y_max - y_height

		# create the mesh array to align the
		self.xx, self.yy = np.meshgrid(	np.arange(x_min, x_max, map_meta_data.resolution),
										np.arange(y_min, y_max, map_meta_data.resolution),
										sparse=True)

		# create heading array
		self.heading_array = np.zeros((self.xx.size, self.yy.size))

	def _array_to_coord(self, i, j):
		"""
		take a array index and transform to coordinate location
		:param i: index location
		:param j: index location
		:return: coordinate [x, y]
		"""
		x = self.xx[0][j]
		y = self.yy[i][0]

		return [x, y]

	def _coord_to_array(self,x, y):
		"""
		take coordinate location and transform to nearest array index
		:param x: coordinate location
		:param y: coordinate location
		:return: index [i, j]
		"""
		idj = (np.abs(self.xx[0] - x).argmin())
		idi = (np.abs(self.yy - y).argmin())

		return [idi, idj]

	def build_heading_graph(self, goal_point):

		self.GoalCoordinate = goal_point

		for index, data in np.ndenumerate(self.heading_array):
			x, y = self._array_to_coord(index[0], index[1])
			x = self.GoalCoordinate[0] - x
			y = self.GoalCoordinate[1] - y
			self.heading_array[index] = math.atan2(y, x)

	def get_heading_components(self, x, y):

		idi, idj = self._coord_to_array(x, y)

		m = self.heading_array[idi][idj]
		xh = math.cos(m)
		yh = math.sin(m)

		return xh, yh



if __name__ == '__main__':

	rospy.init_node("grid_node")

	person_1_pub = rospy.Publisher("/person_1/walk_goal", Pose, queue_size = 1)

	rospy.wait_for_service("/gazebo/get_model_state")
	person_1_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

	MapMeta = MapMetaData()
	MapMeta.resolution = .1
	MapMeta.width = 200				#in cells
	MapMeta.height = 200 			#in cells
	MapMeta.origin.position.x = -10 #in what?
	MapMeta.origin.position.y = -10 #in what?
	MapMeta.origin.position.z = 0

	goal_one_heading = HeadingGraph(MapMeta)

	goal_one_heading.build_heading_graph((3, 4))

	rate = rospy.Rate(10)

	try:
		#publish map
		while not rospy.is_shutdown():

			person_1_data = person_1_service("person_1", "")

			x = int(person_1_data.pose.position.x)
			y = int(person_1_data.pose.position.y)

			pose = Pose()

			xh, yh = goal_one_heading.get_heading_components(x, y)

			pose.position.x = xh*.4
			pose.position.y = yh*.4
			person_1_pub.publish(pose)
			rate.sleep()

	except rospy.ROSInterruptException:
		pass

	print "end of trial"


