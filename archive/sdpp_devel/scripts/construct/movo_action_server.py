#!/usr/bin/env python

import rospy
import copy

import numpy as np

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped


class MovoExploreActionClient(object):

    def __init__(self):

        self.move_base_client = actionlib.SimpleActionClient("movo_move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...movo_move_base")

        """
        Wait 60 seconds for the action server to become available
        """
        if (self.move_base_client.wait_for_server(rospy.Duration(60))):
            rospy.loginfo("Connected to move base server")
        else:
            rospy.logerr("Could not connect to action server")
            self._shutdown()
            return

    def pub_pose(self, pose):

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.pose = pose
        self.move_base_client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)

    def active_cb(self):
        rospy.loginfo("running goal")

    def feedback_cb(self, feedback):
        rospy.loginfo("executing move pose")

    def done_cb(self, status, result):

        print(result)

        if status == 2:
            rospy.loginfo("goal pose has been preempted")

        elif status == 3:
            rospy.loginfo("goal pose has been reached")

        elif status == 4:
            rospy.loginfo("goal pose was aborted")

        elif status == 5:
            rospy.loginfo("goal pose was aborted by the action server")

        elif status == 8:
            rospy.loginfo("received cancel prior to executing")

        else:
            rospy.loginfo(result)


    def move_lookat(self, move, lookat):
        """
        Move and look at a point all numpy vectors
        :param move: numpy.array[3]
        :param lookat: numpy.array[3]
        :return: None
        """
        vec = self.norm_vec_from_2_points(move, lookat)
        quat = self.vec_to_quat_2D(vec)
        pose = self.pose_msg(move, quat)
        self.pub_pose(pose)


    def norm_vec_from_2_points(self, pnt_a, pnt_b):
        """
        take 2 numpy points and find the unit vector between them
        :param pnt_a:   numpy.array[3]
        :param pnt_b:   numpy.array[3]
        :return:        numpy.array[3]
        """
        vec = pnt_b - pnt_a
        norm = np.linalg.norm(vec)
        vec /= norm
        return vec

    def vec_to_quat_2D(self, vec):
        """
        find the quaternion rotation of 2 points
        :param vec: numpy.array[3]
        :return:    numpy.array[4]
        """
        theta = np.arctan2(vec[1], vec[0])
        quat = np.zeros(4)
        quat[2] = np.sin(theta/2.0)
        quat[3] = np.cos(theta/2.0)
        return quat

    def pose_msg(self, point, quat):
        """
        convert a point and quaternion into a ros pose message
        :param point:
        :param quat:
        :return:
        """
        pose = Pose()

        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        pose.position.x = point[0]
        pose.position.y = point[1]
        pose.position.z = point[2]

        return pose

if __name__ == '__main__':

    rospy.init_node("test")
    rospy.sleep(1)

    point_a = np.asarray([1.0, 0.0, 0.0])
    point_b = np.asarray([-2.0, 0.0, 0.0])


    explore_test = MovoExploreActionClient()

    explore_test.move_lookat(point_a, point_b)

    rospy.loginfo("pose sent")

    rospy.spin()

