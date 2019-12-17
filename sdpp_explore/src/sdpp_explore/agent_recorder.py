#!/usr/bin/env python3

import rospy
import os

import pandas as pd
import numpy as np

from nav_msgs.msg import Odometry
from sdpp_explore.srv import AgentTrajRegister, AgentTrajRegisterResponse

import math
import pickle
import random
import matplotlib.pyplot as plt


class AgentRecorder(object):
    def __init__(self, **configs):

        self.agent_builders = {'gazebo': AgentTrajGazebo}
        self.agent_traj_gazebo_config = {}
        self.timer_callback_delay = 5
        
        self.__dict__.update(configs["AgentRecorder"])

        self.agent_dict = {}
        #start factory and add register builders 
        self.agent_factory = AgentTrajFactory()
        for key in self.agent_builders:
            builder = self.agent_builders[key]
            self.agent_factory.register_builder(key, builder)
        
        
        self.add_new_agents()
        #self.timed = rospy.Timer(rospy.Duration(self.timer_callback_delay), self.add_new_agents())
            
    def add_new_agents(self):
        
        new_agents = self.scan_odom()
        for agent_tuple in new_agents:
            print(agent_tuple)

            agent_object = self.agent_factory.create(agent_tuple[1] )
            self.agent_dict[agent_tuple[0]] = agent_object


    def scan_odom(self):
        """
        scans all the topics for odometry and returns namespace
        of valid new agents
        return: (agent_namespace, agent_node_origin)
        """
        published_topics = rospy.get_published_topics()

        # check for all topics with Odometry message type
        odom_proposals = []
        for topic in published_topics:
            if topic[1] == "nav_msgs/Odometry":
                odom_proposals.append(topic[0])
        
        #filer out truth data odom
        #find namespace and node origin
        new_agents = []
        for odom_topic in odom_proposals:
            if "odom" in odom_topic:
                agent_ns = odom_topic.replace('/odom', '')
                if agent_ns not in self.agent_dict:
                    agent_node = self._check_topic_node_origin(odom_topic)
                    new_agents.append((agent_ns, agent_node))
                    rospy.loginfo("agent {} of {} node found".format(agent_ns, agent_node))
                else:
                    rospy.loginfo("agent {} already built".format(agent_ns))
            else:
               pass
        return new_agents

    def _check_topic_node_origin(self, topic):
        """
        checks topic to find originating node for 

        """
        cmd = "rostopic info /human_0/odom"
        unused_currently = os.system(cmd)
        return "gazebo"


class AgentTrajFactory(object):
    """
    creates AgentTrajectory objects
    includes service builder registration
    for dynamic loading of ROS traj nodes
    """
    def __init__(self):
        self._builder = {}
        reg_srv = rospy.Service('agent_traj_register', 
                                AgentTrajRegister,  
                                self.register_builder_server)
    
    def register_builder(self, key, builder):
        self._builder[key] = builder
        rospy.loginfo("registered {} builder".format(builder.__name__))

    def create(self, key, **configs):
        builder = self._builder.get(key)
        if not builder:
            raise ValueError(key)
        return builder(**configs)

    def register_builder_server(self, req):

        if req.builder ==  AgentTrajGazebo.__name__ & req.key == "gazebo":
            self.register_builder(req.key, AgentTrajGazebo)
            return AgentTrajRegisterResponse(True, "gazebo traj registered")


        else:
            return AgentTrajRegister(False, "not valid Agent Trag {} ".format(req))
    
    

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



