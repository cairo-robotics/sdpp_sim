#!/usr/bin/env python

from sdpp_navigation.grid_world import GridWorld, ValueIterationAlgo
import matplotlib.pyplot as plt
from pprint import pprint, pformat
from gym.envs.toy_text import discrete
import rospy
from nav_msgs.msg import OccupancyGrid


from PIL import Image
import numpy as np



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


    max_value =255
    min_value = 0
    old_range = max_value-min_value
    new_range = 100


    world_arr = array
    world_arr_flat = np.asanyarray(world_arr).flatten()
    world_arr_flat = world_arr_flat * (old_range / new_range)
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

    rospy.init_node("nav_grid")

    map_pub = rospy.Publisher("/test", OccupancyGrid, queue_size = 1)
    gamma = .99

    im_frame = Image.open('block_room.png')
    im_frame.show()
    np_frame = np.array(im_frame)
    np_frame = np_frame[: , :, 0]
    print np_frame.shape


    #print np_frame
    #world_input = np.array([[0,0, 255, 100], [0,0 ,0,  0], [0,0,0,0]])
    #world_reward = np.array([[0,0,0,100], [0,0 ,0, 0], [0,0,0,0]])

    world = GridWorld(np_frame, 200, 200)

    plot_world(world.walls_as_array())

    for i in range(115, 125):
        for j in range(35, 45):
            world.cells[i][j].is_terminal = True
            world.cells[i][j].value = 225
            world.cells[i][j].reward = 225
            world.cells[i][j].blocks = True


    assert world.cells[120][40].blocks == True


    print world.walls_as_array()
    #plot_world(world.walls_as_array())

    world_arr = world.as_array()

    #plot_world(world_arr)
    algo = ValueIterationAlgo( gamma, world)

    iter_cnt = 0
    while(True):
        print 'iteration {}'.format(iter_cnt).center(72, '-')

        for cell in world:
            algo.update(cell)

        if iter_cnt >= 1:
            break

        iter_cnt += 1


    world_arr = world.as_array()
    CostMap = array_to_costmap(world_arr)

    #plot_world(world_arr)

    while not rospy.is_shutdown():
        rospy.sleep(1)
        map_pub.publish(CostMap)








