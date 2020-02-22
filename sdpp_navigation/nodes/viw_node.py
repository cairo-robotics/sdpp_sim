#!/usr/bin/env python3

import rospy
import rospkg
import yaml
from sdpp_navigation.value_iter_weighting import ValueIterationWeightingMulti as VIWM



if __name__ == "__main__":
    rospy.init_node("VIWM_node")

    VIWM_config = {"agent_list": ["human_0"]

    }

    filename = "vi_maps.yaml"

    rospack = rospkg.RosPack()

    path = rospack.get_path('sdpp_navigation')
    path += "/config/"


    fp = open(path + filename, "r")

    vi_maps = yaml.full_load(fp)

    test = VIWM(vi_maps = vi_maps, **VIWM_config)

    rospy.spin()
