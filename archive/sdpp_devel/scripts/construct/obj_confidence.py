#!/usr/bin/env python

import rospy

import scipy

import math

from geometry_msgs.msg import PoseStamped



class objectStruct(object):

    def __init__(self, object_type, init_pose, key_id, max_size):

        self.key_id = key_id
        self.max = 50
        self.curr = 0
        self.data = []

    def append_pose(self, pose):

        if len(self.data) >= self.max:
            self.data[self.curr] = pose
            self.curr += 1
        else:
            self.data.append(pose)

        print self.data

    def get_poses(self):
        '''order doesn't mater in this case'''
        return self.data





class objectConfidence(object):

    def __init__(self):

        rospy.Subscriber("/object_pose", PoseStamped, self.object_callback)



    def object_callback(self, msg):

        obj_type = msg.header.frame_id

        obj_position = [msg.pose.position.x, msg.pose.position.y]
        print obj_type, obj_position






if __name__ == '__main__':

    rospy.init_node("object_confidence")

    print("initialize object confidence")

    test = objectConfidence()

    rospy.spin()