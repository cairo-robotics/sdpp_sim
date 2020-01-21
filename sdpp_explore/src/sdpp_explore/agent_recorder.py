#!/usr/bin/env python3

import rospy
import rospkg
import os

import pandas as pd
import numpy as np

from nav_msgs.msg import Odometry
from sdpp_explore.srv import AgentTrajRegister, AgentTrajRegisterResponse

import math
import pickle
import yaml
import random


class AgentRecorder(object):
    """
    state machine that controlls the process of recording agent trajectories


    keyword arguments
    -----------------
    gazebo: dict
        dictionary of gazebo configs 
        (see AgentTraGazebo class below)
    
    spencer: dict
        dictionary of spencer people tracker configs

    agent_dict: dict
        dictionary containing the active agent recording objects.
        keys for agents are the agent's ROS namespace

    AgentRecorder: dict
        dictionary of Agent recorder default attributes all keyword arguments
        below are keys of this dictionary

    agent_builders: dict
        key and obj builder pairs of valid agent builders
        
    timer_callback_delay: int
        how often (seconds) to callback to state machine

    #TODO (60) add service from record movement script for stopping starting recorders

    methods
    -------
    #TODO (60) pickle_data(folder)]
        pickle the data for later use.

    """

    def __init__(self, **configs):
        
        #degault attributes
        self.agent_builders = {'gazebo': AgentTrajGazebo}
        self.agent_traj_gazebo_config = {}
        self.timer_callback_delay = 5
        self.agent_recorder_state = "start"
        # update atributes from config
        self.__dict__.update(configs["AgentRecorder"])

        self.agent_dict = {}
        #start factory and register degault builders 
        self.agent_factory = AgentTrajFactory()
        for key in self.agent_builders:
            builder = self.agent_builders[key]
            self.agent_factory.register_builder(key, builder)
              
        self.timed = rospy.Timer(rospy.Duration(self.timer_callback_delay), 
                                 self.agent_recorder_state_machine())


    def agent_recorder_state_machine(self):
        """
        state machine for agent recorder currently only calls add_new_agents()
        """
        self.add_new_agents()
        pass

    def add_new_agents(self):
        """
        adds any new agents to the agent dictionary and starts recording
        """
        
        new_agents = self.scan_odom()
        for agent_tuple in new_agents:
            print(agent_tuple)

            agent_object = self.agent_factory.create(agent_tuple[1] )
            self.agent_dict[agent_tuple[0]] = agent_object


    def scan_odom(self):
        """
        scans all the topics for odometry and returns namespace
        of valid new agents


        Returns
        -------

        new_agents: list, tuple
            tuple list of (agent_namespace, agent_node_origin) that are not 
            within agent_dict
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
    creates AgentTraj objects for tracking agents. Each agent gets thier own
    tracker that must be initilalized and registered here. Designed for
    dynamic loading and unloading of AgentTraj objects
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
    """
    Records trajectories of agents in a gazebo environmets

    keyword args
    ------------
        file_loc: string
                location of where to record traj data
        
        recording: bool
                whether recording callback is active
        
        agent_ns: string
                ROS namesapce of agent to be recorded

        agent_name: string
                same as ROS namespace unless named

        room: string
                location where traj was recorded


    attributes
    ----------
        start_time: int
                in seconds based on ROS time

        stop_time: int
                in seconds based on ROS time

    #TODO (15) add room value to data set
        
    """
    def __init__(self, **config):
        #default attributes
        self.agent_ns = "human_0"
        self.recording = True
        self.file_loc = "test/"
        self.agent_name = self.agent_ns
        self.start_time = None
        self.stop_time = None
        # update attributes from config
        self.__dict__.update(config)
        rospy.loginfo("begin initialize AgentTrajGazebo.{}".format(self.agent_ns))
        
        self.data = {"list_odom": [],
                     "agent_name": self.agent_name}

        sub_topic = self.agent_ns + "/odom"
        rospy.Subscriber(sub_topic, Odometry, self._traj_callback)

    def set_recording(self, bool_record):
        """
            method to modify callback recording attribute. give log feedback execution
            arguments
            ---------
            bool_record: bool
                conditional for trajectory recording

            #TODO (30) add stop and start times
        """
        
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
                rospy.logerr("cannot change recording true \
                for type {} invalid argument".format(bool_record))

            elif self.recording == False:
                rospy.logerr("cannot change recording false \
                    for type {} invalid argument".format(bool_record))

    def write_data(self, filename="test", pkg="sdpp_explore"):
        """
        write data to specified filename currently only writes to location
        where script was launched from

        keyword args
        ------------
            filename: string
                filename to record data too auto appends file extension

        #TODO (60)expand saved data types to defined structure
        """
        self.set_recording(False)

        data_path = self._file_path_data(pkg)

        full_path = data_path + filename

        self._write_yaml(self.data, full_path)

    
    def _file_path_data(self, pkg):
        """
        returns the file path to the config folder of a package.
        parameters
        ----------
        pkg: string
            name of package to get filepath for
        return
        ------
        filepath: string
            the absolute path of desired package config folder
        """
        r = rospkg.RosPack()
        filepath = r.get_path(pkg)
        filepath = filepath + "/data/"
        
        return filepath



    def _write_yaml(self, data, filename):
        """
        save the array as a yaml file for later use
    
        parameters
        ----------
        filename: string
            filename to save *dict_array_assoc* under
       """
        
        filename = self._yaml_exentsion(filename)
        with open(filename, "w") as outfile:
            yaml.dump(data, outfile, default_flow_style=False)

        

    def _yaml_exentsion(self, filename):
        """
        add ".yaml" extension to filename if needed
        parameters
        ----------
        filename: string
            input filename for checking
        return
        ------
        filename: string
            filename with .yaml extension
        """

        if filename.endswith(".yaml"):
            pass
        else:
            filename += ".yaml"
        return filename

        


    def _traj_callback(self, msg):
        if self.recording == True:
            self._record_data(msg)
        else:
            pass
    
    def _record_data(self, msg):
        """this method is kinda meh"""
        self.data["list_odom"].append(msg)
        


class AgentTrajSpencer(object):
    def __init__(self, **config):
        print("initialize AgentTrajSpencer")
        print("currently not initialized")

    def set_rcording(self, bool_record):
        pass

    def write_data(self, msg):
        pass
    
    def _traj_callback(self, msg):
        pass



if __name__ == '__main__':

    rospy.init_node("test")
    #test = PeopleViewer("test_dict.pickle")
    #test.print_graph(n_tracks=11)

    config_1 = {"agent_ns": "human_0",
                "file_loc": ""}
    config_2 {"agent_ns": "human_0",
                "file_loc": ""}

    traj_fact = AgentTrajFactory()

    traj_fact.register_builder('gazebo', AgentTrajGazebo)

    human_0_obj = traj_fact.create("gazebo", **config_1)

    rospy.sleep(5)

    human_0_obj.write_data()



    #test = AgentTrajGazebo(**config_1)
