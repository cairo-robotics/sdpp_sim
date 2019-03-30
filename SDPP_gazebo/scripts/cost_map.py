#!/usr/bin/env python
import rospy

from nav_msgs.msg import OccupancyGrid, MapMetaData
import tf
import numpy as np
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from scipy.stats import multivariate_normal

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

        #inlcude vector graph

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

if __name__ == '__main__':

    rospy.init_node("test")

    # instantiate the map meta data
    MapMeta = MapMetaData()
    MapMeta.resolution = .1
    MapMeta.width = 200
    MapMeta.height = 200
    MapMeta.origin.position.x = -10
    MapMeta.origin.position.y = -10
    MapMeta.origin.position.z = 0

    person_1_tracker = track_model("person_1")
    test = PredictiveCostMap(MapMeta, "person1", "map")

    rate = rospy.Rate(20)
    try:
        while not rospy.is_shutdown():
            test.update_step()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

    print "end of script"