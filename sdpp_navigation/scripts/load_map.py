#!/usr/bin/env python

from sdpp_navigation.grid_world import GridWorld, ValueIterationAlgo
import matplotlib.pyplot as plt
from pprint import pprint, pformat
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap

from PIL import Image
import numpy as np


class LoadMap(object):
    def __init__(self, file_location=None):

        self.static_odom_map = None

        rospy.loginfo(rospy.get_name() + ": getting map from map server")

        if file_location is None:
            self.static_odom_map = self.load_from_static_map_srv()
        else:
            self.static_odom_map = self.load_from_file_location_map(file_location)

    @staticmethod
    def load_from_static_map_srv(srv_name = '/static_map'):
        rospy.wait_for_service(srv_name)
        static_map_srv = rospy.ServiceProxy(srv_name, GetMap)

        try:
            static_odom_msg = static_map_srv().map
            rospy.loginfo(rospy.get_name() + ": received static map")

        except rospy.ServiceException as exc:
            rospy.logerror(rospy.get_name() + ": unable to get static map" + str(exc))
            return None

        return static_odom_msg

    @staticmethod
    def load_from_file_location_map(file_location):

        rospy.logerror(rospy.get_name() + ": load map directly not finished")
        im_frame = Image.open('block_room.png')
        np_frame = np.array(im_frame)
        np_frame = np_frame[:, :, 0]
        np_frame = np.rot90(np_frame, 3)

        return np_frame

    def gridworld_format_data(self):

        width = self.static_odom_map.info.width
        height = self.static_odom_map.info.height

        numpy_array = np.asarray(self.static_odom_map.data)
        numpy_array = numpy_array.reshape(width, height)

        kwags = {"array": numpy_array, "world_bounds_rows": width, "world_bounds_cols": height}
        return kwags



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

    rospy.init_node("nav_grid")

    test = LoadMap()

    test.gridworld_format_data()

    map_pub = rospy.Publisher("/test", OccupancyGrid, queue_size = 1)
    gamma = .99

    kwags = test.gridworld_format_data()

    world = GridWorld(**kwags)

    #plot_world(world.walls_as_array())

    world.cells[40][40].reward = 500
    world.cells[40][40].value = 500

    '''
    for i in range(115, 125):
        for j in range(35, 45):
            world.cells[i][j].is_terminal = True
            world.cells[i][j].value = 225
            world.cells[i][j].reward = 225
            world.cells[i][j].blocks = True
    '''

    world_arr = world.as_array()

    algo = ValueIterationAlgo(gamma, world)

    iter_cnt = 0
    '''
    while(True):
        print 'iteration {}'.format(iter_cnt).center(72, '-')

        for cell in world:
            algo.update(cell)

        algo.update_values(world)

        if iter_cnt >= 20:
            break

        iter_cnt += 1
    


    world_arr = world.as_array()
    CostMap = array_to_costmap(world_arr)

    #print CostMap.data
    #plot_world(CostMap.data)

    while not rospy.is_shutdown():
        rospy.sleep(1)
        map_pub.publish(CostMap) 
    '''








