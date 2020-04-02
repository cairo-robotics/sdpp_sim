#!/usr/bin/env python

import rosparam
import rospy

import numpy as np

from sdpp_experiments.agent_action_client import AgentActionClient

if __name__ == '__main__':

    #TODO (45) fully load configs from yamls
    

    rospy.init_node("move_agents")

    experiment_info = rosparam.get_param("experiment_info")


    agent_list = []
    for agent_info in experiment_info:
        agent_list.append(AgentActionClient(**agent_info))
    

    while not rospy.is_shutdown():
        pass

