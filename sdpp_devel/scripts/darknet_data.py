#!/usr/bin/env python


import rospy

import numpy as np

from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

import tf2_ros

import tf2_msgs.msg

import geometry_msgs.msg


class darknetObjectIden():

    def __init__(self, hz):
        self.hz = hz







bridge = CvBridge()
pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

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

    sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, BB_callback)

    print("received")


    while not rospy.is_shutdown():
        pass

