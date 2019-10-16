#!/usr/bin/env python

import rospy
import numpy as np

import matplotlib.pyplot as plt

from nav_msgs.msg import OccupancyGrid
from sdpp_navigation.grid_world import GridWorld, ValueIterationAlgo
from sdpp_navigation.ros_wrappers import LoadMap

#class value_iter_bayes(object):


def done(world, prev_world):
    epsilon = 0.00001
    try:
        diff = np.abs(world.as_array() - prev_world)
    except RuntimeWarning:
        pass
    return not (diff > epsilon).any()


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


def plot_world(world_arr):
    plt.imshow(world_arr, cmap="plasma")
    plt.colorbar()
    plt.show()


if __name__ == '__main__':

    gamma = .99
    epsilon = 0.0001
    iter_max = 3
    iter_cnt = 0

    rospy.init_node("nav_grid")
    map_pub = rospy.Publisher("/test", OccupancyGrid, queue_size = 1)

    static_map_obj = LoadMap()
    static_map_kwags = static_map_obj.gridworld_format_data()

    world = GridWorld(**static_map_kwags)

    world.add_reward_block(40 ,40)

    world.plot_world(type="wall_map")
    world.plot_world(type="reward_map")

    algo = ValueIterationAlgo(gamma, world)



    while(True):
        print 'iteration {}'.format(iter_cnt).center(72, '-')

        for cell in world:
            algo.update(cell)

        algo.update_values(world)

        if iter_cnt >= iter_max:
            break

        iter_cnt += 1
    
    world.plot_world()
    world_arr = world.as_array()
    CostMap = array_to_costmap(world_arr)

    '''
    while not rospy.is_shutdown():
        rospy.sleep(1)
        map_pub.publish(CostMap)
    '''