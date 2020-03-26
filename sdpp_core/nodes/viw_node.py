#!/usr/bin/env python3

import rospy
import rospkg
import yaml
from sdpp_core.value_iter_weighting import ValueIterationWeightingMulti as VIWM



if __name__ == "__main__":
    rospy.init_node("VIWM_node")


    filename = "vi_maps.yaml"

    rospack = rospkg.RosPack()

    path = rospack.get_path('sdpp_core')
    path += "/config/vi_maps/"


    fp = open(path + filename, "r")
    vi_maps = yaml.full_load(fp)


    
    VIWM_config = {"agent_list": ["human_0"]
    }

    test = VIWM(vi_maps, **VIWM_config)

    rospy.spin()
