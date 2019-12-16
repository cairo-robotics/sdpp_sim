#!/usr/bin/env python3

import rospy
import pickle
import random
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

#from sklearn import mixture

#from spencer_tracking_msgs.msg import TrackedPersons
from nav_msgs.msg import Odometry

from sensor_msgs.msg import CompressedImage

from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError

#import cv2

import math



class AgentRecorder(object):
    def __init__(self):
        pass

    def _init_dict(self):
        pass

    def record_callback(self):
        pass
    
    def record_local(self):
        pass

    def record_file(self):
        pass
    


class AgentTrajFactory(object):
    """
    creates AgentTrajectory objects with callback methods that override a base class
    but what base class should that be?
    """

    def __init__(self):
        self._builder = {}

    def register_builder(self, key, builder):
        self._builder[key] = builder
        rospy.loginfo("registered {} builder".format(builder.__name__))

    def create(self, key, **configs):
        builder = self._builder.get(key)
        if not builder:
            raise ValueError(key)
        return builder(**configs)


class AgentTrajGazebo(object):
    def __init__(self, **config):
        
        self.agent_ns = "human_0"
        self.recording = True
        self.file_loc = "test/"
        
        self.__dict__.update(config)
        rospy.loginfo("begin initialize AgentTrajGazebo.{}".format(self.agent_ns))
        
        self.data = []

        sub_topic = self.agent_ns + "/odom"
        rospy.Subscriber(sub_topic, Odometry, self._traj_callback)


    def set_recording(self, bool_record):
        
        if self.recording == True & bool_record == True:
            rospy.loginfo("continue recording {} traj data".format(self.agent_ns))
            self.recording = bool_record

        elif self.recording == True & bool_record == False:
            rospy.loginfo("Stop recording {} traj data".format(self.agent_ns))
            self.recording = bool_record

        elif self.recording == False & bool_record == False:
            rospy.loginfo("Still not recording{ } traj data".format(self.agent_ns))
            self.recording = bool_record

        elif self.recording == False & bool_record == True:
            rospy.loginfo("Start recording {} traj data".format(self.agent_ns))
            self.recording = bool_record

        else:
            if self.recording == True:
                rospy.logerror("cannot change recording true fo type {} invalid argument".format(bool_record))

            elif self.recording == False:
                rospy.logerror("cannot change recording false for type {} invalid argument".format(bool_record))


    def write_data(self, filename="test"):
        self.set_recording(False)
        traj_data = pd.DataFrame(self.data)
        traj_data.to_pickle(filename + ".p")

    def _traj_callback(self, msg):
        if self.recording == True:
            self._record_data(msg)
        else:
            pass
    
    def _record_data(self, msg):
        """this method is kinda meh"""
        self.data.append(msg)
        


class AgentTrajSpencer(object):
    def __init__(self, **config):
        print("initialize AgentTrajSpencer")
        print("currently not initialized")

    def traj_callback(self, msg):
        pass



if __name__ == '__main__':

    rospy.init_node("test")
    #test = PeopleViewer("test_dict.pickle")
    #test.print_graph(n_tracks=11)

    config_1 = {"test_config": "test2"}

    traj_fact = AgentTrajFactory()

    traj_fact.register_builder('gazebo', AgentTrajGazebo)

    traj_fact.create("gazebo", **config_1)

    rospy.spin()

    #test = AgentTrajGazebo(**config_1)



