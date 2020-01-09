#!/usr/bin/env python

import rosparam
import rospy

import numpy as np

from sdpp_testing_fw.agent_action_client import AgentActionClient

if __name__ == '__main__':

    rospy.init_node("move_agents")

    waypoints_human_0 = rosparam.get_param("human_0/waypoints")
    waypoints_human_1 = rosparam.get_param("human_1/waypoints")

    #print(waypoints_human_0)
    #print(waypoints_human_sdpp)
    
    human_0_kwags = {"frame_id":             "human_0/odom",
                     "action_client_topic":  "human_0/move_base",
                     "points":               waypoints_human_0
                     }

    human_1_kwags = {"frame_id":             "human_1/odom",
                     "action_client_topic":  "human_1/move_base",
                     "points":               waypoints_human_1}

    
    move_human_0 = AgentActionClient(**human_0_kwags)
    
    move_human_1 = AgentActionClient(**human_1_kwags)
    


    while not rospy.is_shutdown():
        pass

