#!/usr/bin/env python


import rospy
import copy

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatus

from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler


class HumanMoveBaseClient(object):

    def __init__(self, pose_seq):

        self.goal_cnt = 0

        self.pose_seq = pose_seq
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.goal_pub = rospy.Publisher("/human_goal", String, queue_size=1)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose " + str(self.goal_cnt + 1) + " to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        rospy.spin()

    def movebase_client(self, pose):

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose
        result = self.client.send_goal_and_wait(goal)

        self.done_cb(result, None)

    def active_cb(self):
        rospy.loginfo("Goal pose " + str(self.goal_cnt + 1) + " is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        # To print current pose at each feedback:
        # rospy.loginfo("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))
        rospy.loginfo("Feedback for goal pose " + str(self.goal_cnt + 1) + " received")
        # temporary until object lookup
        self.goal_state_faker()

    def goal_state_faker(self):

        #world 1 based feedback
        if self.goal_cnt % 2 == 0:
            goal_state = "start"

        elif self.goal_cnt == 1:
            goal_state = "fridge"

        elif self.goal_cnt == 3:
            goal_state = "microwave"

        elif self.goal_cnt == 5:
            goal_state = "cabinet"

        elif self.goal_cnt == 7:
            goal_state = "table"

        elif self.goal_cnt == 9:
            goal_state = "coffee_machine"

        else:
            goal_state = "error"

        self.goal_pub.publish(goal_state)

    def done_cb(self, status, result):
        self.goal_cnt += 1
        if status == 2:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached")
            if self.goal_cnt< len(self.pose_seq):
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb)
            else:
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                return

        if status == 4:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
            return

        if status == 5:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
            return

        if status == 8:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started")


if __name__ == '__main__':
    rospy.init_node("move_sequence")

    goal_params =  rospy.get_param("goal_locations")
    pose = Pose()
    pose.orientation.w = 0.707
    pose.orientation.z = 0.707
    pose_array = []

    goal_keys = goal_params.keys()
    for goal in goal_keys:
        if goal == "start":
            continue
        else:
            pose.position.x = goal_params["start"]["x"]
            pose.position.y = goal_params["start"]["y"]
            pose_array.append(copy.deepcopy(pose))
            pose.position.x = goal_params[goal]["x"]
            pose.position.y = goal_params[goal]["y"]
            pose_array.append(copy.deepcopy(pose))

    while not rospy.is_shutdown():

        test = HumanMoveBaseClient(pose_array)
        del test


