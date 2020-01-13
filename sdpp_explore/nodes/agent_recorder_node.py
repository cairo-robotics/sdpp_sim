#!/usr/bin/env python3

from sdpp_explore.agent_recorder import AgentTrajFactory, AgentTrajGazebo, AgentRecorder
import rospy

if __name__ == '__main__':

    #TODO (45) load in config data from yaml

    rospy.init_node("test")


    agent_recorder_configs = {"test": "agent_record_test"}
    agent_traj_gazebo_configs = {"test": "gazebo_test"}

    config_test= {  "AgentRecorder": agent_recorder_configs,
                    "AgentTrajGazebo": agent_traj_gazebo_configs}


    agent_recoder = AgentRecorder(**config_test)

    """
    traj_fact = AgentTrajFactory()

    traj_fact.register_builder('gazebo', AgentTrajGazebo)

    traj_fact.create("gazebo", **config_1)

    """

    #test = AgentTrajGazebo(**config_1)

    rospy.spin()

