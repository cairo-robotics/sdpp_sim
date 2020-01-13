#!/usr/bin/env python

import rospy
import pickle
import numpy as np
import rospkg

import matplotlib.pyplot as plt

from nav_msgs.msg import OccupancyGrid
from sdpp_navigation.grid_world import GridWorld, ValueIterationAlgo, ValueIterWeighting
from sdpp_navigation.ros_wrappers import LoadMap



class sdppWorld(object):

    """
    encompasing mindstate for sdpp planning
     

    keyword args
    -----------

    static_map: 
    
    sdpp_pub_topic:


    methods
    -------

    TODO (60) 1 build_room()
        build the room based off goal locations

    TODO (30) 1 save_room()
        save the room 

    TODO (30) 1 load_room()
        load room from file

    TODO (30) 1 init_VIW_instance()

    TODO (90) 1 Miscelanious level 1 work

    TODO (120) 2 miscelnaious level 2 work




    """

    def __init__(self):
        pass

def array_to_costmap(array):

    CostMap= OccupancyGrid()
    CostMap.info.resolution = .05
    CostMap.info.width = 200
    CostMap.info.height = 200

    new_range = 100

    world_arr = array
    world_arr_flat = np.asanyarray(world_arr).flatten()
    max_value = np.amax(world_arr_flat)
    min_value = np.amin(world_arr_flat)
    old_range = max_value-min_value
    world_arr_flat = world_arr_flat / (old_range / new_range)
    world_arr_flat = world_arr_flat.astype(np.int8)
    world_arr_flat = world_arr_flat.tolist()
    map_tuple = tuple(world_arr_flat)
    CostMap.data = map_tuple

    return CostMap


def test_routine(human_goals):
    pass

def load_and_build_vi():
    static_map_obj = LoadMap()
    static_map_dict = static_map_obj.gridworld_format_data()

    goal_loc = rospy.get_param("/goal_loc")
    
    print goal_loc

    for idx, goal in enumerate(goal_loc):
        for idy, item in enumerate(goal):
            goal_loc[idx][idy] = int(item*20)
    
    print goal_loc

    
    kwags_VIW = {"goals_loc":   goal_loc,
                 "epsilon":     0.001,
                 "gamma":       .99,
                 "iter_max":    500,
                 "plot":        None}

    return ValueIterWeighting(static_map_dict=static_map_dict, **kwags_VIW)



if __name__ == '__main__':

    rospy.init_node("nav_grid")
    
    if True:
        VIW_obj = load_and_build_vi()
        filename = "narrow_3_goal_10x10.p"
        VIW_obj.pickle_obj_dict(filename)

    if False:
        test = ValueIterWeighting(pickle_file="narrow_3_goal_10x10.p")

        test.grid_worlds_array[1].plot_world()

        test.bayesian_path_matching("human_0")

        map_pub0 = rospy.Publisher("/test0", OccupancyGrid, queue_size=1)
        costmap_0 = test.grid_worlds_array[0].value_as_array()
        costmap_0 = array_to_costmap(costmap_0)

        map_pub1 = rospy.Publisher("/test1", OccupancyGrid, queue_size=1)
        costmap_1 = test.grid_worlds_array[1].value_as_array()
        costmap_1 = array_to_costmap(costmap_1)

        while not rospy.is_shutdown():
            rospy.sleep(1)
            map_pub0.publish(costmap_0)
            map_pub1.publish(costmap_1)