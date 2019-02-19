#!/usr/bin/env python

import math
import numpy as np
from scipy.stats import multivariate_normal

import rospy
import tf

from nav_msgs.msg import OccupancyGrid, MapMetaData
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

from gazebo_msgs.srv import GetModelState,SetModelState



class HeadingMap(object):

    def __init__(self, map_meta_data):

        self.MapMetaData = map_meta_data
        self.GoalCoordinate = [0, 0]

        # set array around origin of zero
        x_width = map_meta_data.width * map_meta_data.resolution
        x_max = x_width + map_meta_data.origin.position.x
        x_min = x_max - x_width

        y_height = map_meta_data.height * map_meta_data.resolution
        y_max = y_height + map_meta_data.origin.position.y
        y_min = y_max - y_height

        # create the mesh array to align the
        self.xx, self.yy = np.meshgrid(np.arange(x_min, x_max, map_meta_data.resolution),
                                       np.arange(y_min, y_max, map_meta_data.resolution),
                                       sparse=True)

        # create heading array
        self.heading_array = np.zeros((self.xx.size, self.yy.size))


    def _array_to_coord(self, i, j):
        """
        take a array index and transform to coordinate location
        :param i: index location
        :param j: index location
        :return: coordinate [x, y]
        """
        x = self.xx[0][j]
        y = self.yy[i][0]

        return [x, y]

    def _coord_to_array(self, x, y):
        """
        take coordinate location and transform to nearest array index
        :param x: coordinate location
        :param y: coordinate location
        :return: index [i, j]
        """
        idj = (np.abs(self.xx[0] - x).argmin())
        idi = (np.abs(self.yy - y).argmin())

        return [idi, idj]

    def build_heading_graph(self, goal_point):
        """
        create a heading graph for a given goal point
        :param goal_point: [x, y] goal point
        :return: nothing
        """
        self.GoalCoordinate = goal_point

        for index, data in np.ndenumerate(self.heading_array):
            x, y = self._array_to_coord(index[0], index[1])
            x = self.GoalCoordinate[0] - x
            y = self.GoalCoordinate[1] - y
            self.heading_array[index] = math.atan2(y, x)

    def get_coord_value(self, x, y):
        """
        get the value at a coordinate
        :param x: coordinate location
        :param y: coordinate location
        :return: location value
        """
        try:
            idi, idj = self._coord_to_array(x, x)
            return self.map[idi][idj]
        except Exception as e:
            rospy.logwarn(e)

    def get_heading_components(self, x, y):
        """
        get heading coordinates at a point
        :param x: x coordinate location
        :param y: y coordinate location
        :return: xh, yh heading components in x and y
        """

        idi, idj = self._coord_to_array(x, y)

        m = self.heading_array[idi][idj]
        xh = math.cos(m)
        yh = math.sin(m)

        return xh, yh

    def set_coord_value(self, x, y, value):
        """
        Set the coordinate value
        :param x: coordinate location
        :param y: coordinate location
        :param value: value to set
        :return: True false is succesful
        """
        try:
            idi, idj = self._coord_to_array(x, y)
            self.map[idi][idj] = value
            return True

        except Exception as e:
            rospy.logwarn(e)

class PredictiveCostMap(object):

    def __init__(self, map_meta_data, agent, parent):

        #interfaces
        self.tf_listener = tf.TransformListener()
        self.grid_pub = rospy.Publisher("test", OccupancyGrid, queue_size=1)

        #meta data
        self.goal_points = [[4, 5], [4, 2]]
        self.MapMetaData = map_meta_data
        self.agent_topic = agent
        self.parent_frame = parent

        #initialize grid
        self.occupancy_grid = OccupancyGrid()
        self.occupancy_grid.info = self.MapMetaData
        self.occupancy_grid.header.frame_id = self.parent_frame

        #shortest path graphs
        #for point in self.goal_points:




    def gaussian_at_loc(self, mu, eps):

        x, y = np.mgrid[-10:10:.1, -10:10:.1]
        pos = np.dstack((x, y))
        rv = multivariate_normal([mu[0], mu[1]], eps)
        some_grid = rv.pdf(pos)
        some_grid /= np.max(np.abs(some_grid))
        some_grid *= 100
        #change this to a return statement
        self.map = some_grid.astype(int).T

    def update_step(self):

        if self.tf_listener.frameExists("person1"):
            t = self.tf_listener.getLatestCommonTime("person1", "map")
            trans, rot = self.tf_listener.lookupTransform('/map', '/person1', t)
            self.gaussian_at_loc(np.array([trans[0], trans[1]]), np.array([[.1, 0], [0, .1]]))
            self.occupancy_grid.header.stamp = t
            self.occupancy_grid.data = test.map.flatten('C').tolist()
            self.grid_pub.publish(self.occupancy_grid)
            #self.tf_listener.LookupTwist('/map', '/person1', t)
        else:
            print("tf frames missing")
