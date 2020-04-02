#!/usr/bin/env python3

import rospy
import rospkg
import yaml

from sdpp_core.value_iteration import ValueIterationMapManager as VIMM


VIMM_config0 = {"iter_max": 1000, "gamma": .99, "epsilon": .001}


if __name__ == "__main__":

    filename = "narrow_3_goal_10x10_occupancy_grid.yaml"
    
    rospy.init_node("build_map")
    rospack = rospkg.RosPack()
    path = rospack.get_path("sdpp_gazebo")
    path += "/config/maps/"
    fp = open(path + filename)  

    occupancy_grid_data = yaml.full_load(fp)

    VIMM_obj = VIMM(occupancy_grid_data, **VIMM_config0)

    map_set = VIMM_obj.build_vi_map_set([[40, 20], [100, 20], [160, 20]])






    VIMM_obj.save_maps(map_set)
