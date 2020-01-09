
import rosparam

import rospy
import numpy as np
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseStamped



class AgentActionClient(object):
    """
    action client interface for testing framework
    
    keyword arguments
    -----------------
    frame_id: string
        frame that movement actions are to be published in

    action_client_topic: string
        topic name for agent's action client

    quaternion: list
        base quaternion value for pose

    points: list
        list of tuples for waypoints
    
    """

    def __init__(self, **kwags):

        self.frame_id = "human_sdpp/odom"
        self.action_client_topic = "human_sdpp/move_base",
        self.quaternion =  [0, 0, 0, 1]
        self.points = []
    
        self.__dict__.update(kwags)
     
        self.iterator = 0
        self.max_iterator = len(self.points) -1

        self.move_base_client = actionlib.SimpleActionClient(self.action_client_topic, MoveBaseAction)
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

        self.next_pose()


    def pub_pose(self, pose):

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame_id
        goal.target_pose.pose = pose
        self.move_base_client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)

    def next_pose(self):

        next_point = self.points[self.iterator]

        next_pose = self.pose_msg(next_point, self.quaternion)
        self.pub_pose(next_pose)

        if self.iterator >= self.max_iterator:
            self.iterator = 0
        else:
            self.iterator += 1

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
            self.next_pose()

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