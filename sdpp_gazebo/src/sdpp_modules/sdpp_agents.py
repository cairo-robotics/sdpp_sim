#!/usr/bin/env python

import rospy
import numpy as np
import math
import tf
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState

from sdpp_modules.sdpp_maps import HeadingMap


class WalkingPerson(HeadingMap):

	def __init__(self, map_meta_data, agent_name):

		# object members
		self.agent = agent_name
		self.MapMetaData = map_meta_data
		self.rate = rospy.Rate(10)
		self.agent_state = ModelState()
		self.walk_pose = Pose()
		self.goal_coord = (4, 5)

		# interfaces
		pub_topic = "/" + agent_name + "/walk_goal"
		self.wlk_cmd_pub = rospy.Publisher(pub_topic, Pose, queue_size=1)

		rospy.wait_for_service("/gazebo/get_model_state")
		self.get_state_srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
		rospy.loginfo("walking_person GET_model_state service active")

		rospy.wait_for_service("/gazebo/set_model_state")
		self.set_state_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
		rospy.loginfo("walking_person SET_model_state service active")

		self.tf_broadcaster = tf.TransformBroadcaster()

		# create headingMap object
		HeadingMap.__init__(self, map_meta_data)

	def set_update_rate(self, rate):
		#TODO make asynchronous

		self.rate = rospy.Rate(rate)

	def reset_pose(self, pose):

		rospy.logwarn("reset_pose function faked")
		reset_pose = ModelState()

		reset_pose.model_name = self.agent
		reset_pose.pose.position.x = -5
		reset_pose.pose.position.y = 0
		reset_pose.pose.position.z = .01

		reset_pose.pose.orientation.w = .707
		reset_pose.pose.orientation.x = 0
		reset_pose.pose.orientation.y = 0
		reset_pose.pose.orientation.z = .707

		reset_pose.twist.linear.x = 0
		reset_pose.twist.linear.y = 0
		reset_pose.twist.linear.z = 0

		self.set_state_srv(reset_pose)


	def update_step(self):

		self.agent_state = self.get_state_srv(self.agent, "")

		#publish in tf tree
		self.model_state_to_tf(self.agent_state)

		x = int(self.agent_state.pose.position.x)
		y = int(self.agent_state.pose.position.y)

		xh, yh = self.get_heading_components(x, y)

		self.walk_pose.position.x = xh*.4
		self.walk_pose.position.y = yh*.4

		diff_x = abs(self.agent_state.pose.position.x - 4)
		diff_y = abs(self.agent_state.pose.position.y - 5)

		if diff_x <= .2 and diff_y <= .2:
			self.reset_pose("bs")
		else:
			self.wlk_cmd_pub.publish(self.walk_pose)

		self.rate.sleep()

	def model_state_to_tf(self, agent_state):
		try:
			translation = [	agent_state.pose.position.x,
							agent_state.pose.position.y,
							agent_state.pose.position.z]
			self.tf_broadcaster.sendTransform(	translation,
												(0.0, 0.0, 0.0, 1.0),
												rospy.Time.now(),
												self.agent,
												"odom")
		except ValueError:
			rospy.logwarn("tf topic not found")