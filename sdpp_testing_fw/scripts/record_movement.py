#!/usr/bin/env python

import rospy
import pickle
import numpy as np

from dynamic_reconfigure.server import Server
from sdpp_testing_fw.cfg import trajinterfaceConfig
from nav_msgs.msg import Odometry

class TrajAgentInterface(object):
    def __init__(self, kwargs):

        self.format_kwags(kwargs)
        options = {"odom_topic":    "human_sdpp",
                   "hz":            30,
                   "file_prefix":   "default",
                   "recording":     False
        }
        options.update(kwargs)
        self.__dict__.update(options)

        suffix = self.odom_topic.replace("/", "'")
        self.filename = self.file_prefix + suffix
        self.odom_array = np.array(Odometry())

        rospy.Subscriber(self.odom_topic, Odometry, self.traj_callback)

    @staticmethod
    def format_kwags(kwargs):
        if 'odom_topic' not in kwargs:
            rospy.logwarn("default odom topic used for TragAgentInterface")

        if 'file_prefix' not in kwargs:
            rospy.logwarn("default file_prefix topic used for TragAgentInterface")

    def srv_callback(self, config, level):
        rospy.loginfo("""Reconfiugre Request: {record}, {read_pickle}, {write_pickle}""".format(**config))
        self.recording = config["record"]
        if config["read_pickle"]:
            self.read_pickle()
            config["read_pickle"] = False
        if config["write_pickle"]:
            self.write_pickle()
            config["write_pickle"] = False
        return config

    def traj_callback(self, msg):
        if self.recording:
            np.append(self.odom_array, msg)

    def write_pickle(self):
        rospy.loginfo("pickling file: " + self.filename)
        pickle.dump(self.__dict__, open(self.filename, "wb"))
        rospy.loginfo("finished pickling file: " + self.filename)

    def read_pickle(self):
        rospy.loginfo("unpickling file: " + self.filename)
        self.__dict__.update(pickle.load(open(self.filename, "rb")))
        rospy.loginfo("unpicked file: " + self.filename)


class Measurements(object):

    def __init__(self, kwargs):

        options = {"odom_topic_array": ["human_sdpp"],
                   "file_prefix":      "default_",
                   "recording":  False
        }

        options.update(kwargs)
        self.__dict__.update(options)
        rospy.loginfo("recording initializing: " + str(self.recording))

        self.traj_agent_list = []
        traj_agent_kwags = {"file_prefix":  self.file_prefix,
                            "recording":    self.recording
                            }

        for odom_topic in self.odom_topic_array:

            traj_agent_kwags["odom_topic"] = odom_topic

            self.traj_agent_list.append(TrajAgentInterface(traj_agent_kwags))
            rospy.loginfo("initialize traj_agent: " + odom_topic)

        rospy.loginfo("all traj_agents intialized")
        srv = Server(trajinterfaceConfig, self.srv_callback)

    def srv_callback(self, config, level):

        rospy.loginfo("""Reconfiugre Request: {record}, {read_pickle}, {write_pickle}""".format(**config))
        self.recording = config["record"]
        rospy.loginfo("recording status: " + str(self.recording))

        for traj_agent in self.traj_agent_list:
            traj_agent.recording = self.recording

        if config["read_pickle"]:
            for traj_agent in self.traj_agent_list:
                traj_agent.read_pickle()
            config["read_pickle"] = False

        if config["write_pickle"]:
            for traj_agent in self.traj_agent_list:
                traj_agent.write_pickle()
            config["write_pickle"] = False

        return config


if __name__ == "__main__":
    
    rospy.init_node("recorder_node")

    topic_names = ["human_0", "human_sdpp"]

    msm_kwags = {"odom_topic_array": topic_names,
               "file_prefix": "test_",
               "start_recording": False
               }

    test = Measurements(msm_kwags)

    rospy.spin()