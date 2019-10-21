#!/usr/bin/env python

import rospy
import numpy as np

import matplotlib.pyplot as plt

from nav_msgs.msg import OccupancyGrid
from sdpp_navigation.grid_world import GridWorld, ValueIterationAlgo, ValueIterWeighting
from sdpp_navigation.ros_wrappers import LoadMap


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


if __name__ == '__main__':

    rospy.init_node("nav_grid")
    map_pub = rospy.Publisher("/test", OccupancyGrid, queue_size=1)

    static_map_obj = LoadMap()
    static_map_dict = static_map_obj.gridworld_format_data()

    kwags_VIW = {"goals_loc":   [(60, 20), (20, 20)],
                 "epsilon":     0.001,
                 "gamma":       .99,
                 "iter_max":    200,
                 "plot":        None}

    #test = ValueIterWeighting(static_map_dict=static_map_dict, **kwags_VIW)
    #test.pickle_obj_dict("test1.p")

    test = ValueIterWeighting(pickle_file="test1.p")

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
