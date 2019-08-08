#!/usr/bin/env python

import rospy

import math
import numpy as np

from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

import tf
import tf2_ros
from tf2_msgs.msg import TFMessage

from geometry_msgs.msg import TransformStamped, PoseStamped

OBJ_CLASS_LIST = ["bottle", "cup"]


class darknetObjectIden(object):

    def __init__(self, class_list, hz):
        self.hz = hz
        self.sleep_time = 1.0/hz
        self.class_list = class_list
        self.bridge = CvBridge()

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        #self.br

        self.bounding_box_topic = "/darknet_ros/bounding_boxes"
        self.depth_topic = "/movo_camera/sd/image_depth"

        self.pub_obj_pose = rospy.Publisher("/object_pose", PoseStamped, queue_size=1)
        self.pub_tf = rospy.Publisher("/tf", TFMessage, queue_size=1)

        self.spin_loop()


    def spin_loop(self):
        """
        the loop that checks bounding box data and matches it with the depth camera
        :return:
        """
        while not rospy.is_shutdown():
            '''loop delay'''
            rospy.sleep(self.sleep_time)
            try:
                bounding_boxes = rospy.wait_for_message(self.bounding_box_topic, BoundingBoxes, 2)
            except rospy.ROSException:
                rospy.logwarn("waiting for bounding boxes 2 second delay")
                rospy.sleep(2)
                continue

            ''' get the depth map to resolve object location'''
            depth_ros = rospy.wait_for_message(self.depth_topic, Image, 2)
            depth_cv = self.bridge.imgmsg_to_cv2(depth_ros, "32FC1")

            '''check all objects and filter out what your not looking for'''
            for obj_bb in bounding_boxes.bounding_boxes:
                if not self.object_filter(obj_bb.Class):
                    continue
                pose = self.obj_pose(obj_bb, depth_cv)
                if pose is not None:
                    self.pub_obj_pose.publish(pose)
                else:
                    rospy.logwarn("object pose unresolveable")

    def obj_pose(self, obj_bb, depth_cv):
        """
        this method is garbage and needs to be fixed, inconsistent object locations
        the forward kinematics are pure poop.
        TODO fix this POS
        :param obj_bb:
        :param depth_cv:
        :return:
        """

        midpoint = np.zeros(2)
        '''run checks for legitimateness'''
        midpoint[0] = int((obj_bb.xmax + obj_bb.xmin) / 2.0)
        midpoint[1] = int((obj_bb.ymax + obj_bb.ymin) / 2.0)
        depth_midpoint = depth_cv[midpoint[0], midpoint[1]]
        if math.isnan(depth_midpoint):
            return None

        try:
            trans = self.tfBuffer.lookup_transform("movo_camera_ir_frame", "odom", rospy.Time())

            quaternion = (
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w,)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            yaw = euler[2]
            x = math.cos(yaw)
            y = math.sin(yaw)

            object_pose = PoseStamped()
            object_pose.header.frame_id = obj_bb.Class

            object_pose.pose.orientation.w = 1.0

            object_pose.pose.position.x = trans.transform.translation.x + x*depth_midpoint
            object_pose.pose.position.y = trans.transform.translation.y + y*depth_midpoint
            object_pose.pose.position.z = 1.0

            return object_pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print "no go on transform" + str(e)
            pass


    def pub_tf(self, child_frame, transform):
        """

        :param child_frame:
        :param transform:
        :return:
        """

        t = TransformStamped()

        t.header.frame_id = "movo_camera_ir_frame"

        t.header.stamp = rospy.Time.now()
        t.child_frame_id = child_frame
        t.transform = transform

        tfm = TFMessage([t])

        self.pub_tf(tfm)


    def object_filter(self, object):

        for name in self.class_list:
            if name == object:
                print name
                return True
        return False


def BB_callback(msg):

    object = msg.bounding_boxes[0]

    midpoint = np.zeros(2)

    midpoint[0] =  (object.xmax + object.xmin)/2

    midpoint[1] = (object.ymax + object.ymin)/2

    print midpoint

    ros_depth_image = rospy.wait_for_message("/movo_camera/sd/image_depth", Image)

    cv_depth_image = bridge.imgmsg_to_cv2(ros_depth_image, "32FC1")

    depth_midpoint =  cv_depth_image[midpoint[0], midpoint[1]]

    print depth_midpoint


    t = geometry_msgs.msg.TransformStamped()

    t.header.frame_id = "movo_camera_ir_frame"
    t.header.stamp = rospy.Time.now()

    t.child_frame_id = "object1"

    t.transform.translation.x = depth_midpoint

    t.transform.rotation.w = 1.0

    tfm = tf2_msgs.msg.TFMessage([t])
    pub_tf.publish(tfm)


if __name__ == '__main__':

    rospy.init_node("darknet_translate")

    print("waiting")
    refresh_rate = 1

    darknet_obj = darknetObjectIden(OBJ_CLASS_LIST, refresh_rate)

    print("received")


    while not rospy.is_shutdown():
        pass

