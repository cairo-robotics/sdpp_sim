#!/usr/bin/env python3

import rospy
import yaml
import time
import collections

import numpy as np

from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from scipy.special import softmax
from scipy.stats import multivariate_normal


class ValueIterationWeightingMulti(object):
    """"

    methods
    -------

    """

    def __init__(self, vi_maps, agent_list=["human_0"],
                 mask_threshold_index=10,
                 viw_buffer_size=200):
        
        #VIW maps for use with algo
        self.vi_maps = vi_maps
        
        # initializations for VIW algorithm
        self.agent_list = agent_list
        self.mask_threshold_index = mask_threshold_index
        self.viw_buffer_size = viw_buffer_size
        
        self.pub_map = ("/human_sdpp/move_base/global_costmap/"
                        "costmap_injection_layer/costmap_injection")
        
        #value iterative weighted agents being tracked
        self.list_viw_agents = []
        for agent in self.agent_list:
            viw_agent = self.spawn_agent(agent, self.viw_buffer_size)
            self.list_viw_agents.append(viw_agent)
        self.run_agents(self.list_viw_agents)

        self.map_pub = rospy.Publisher(self.pub_map, OccupancyGrid, 
                                       queue_size=10)
        
        while(True):
            for agent in self.agent_list:
                start = time.time()
                self.timed_callback(self.list_viw_agents, self.vi_maps, 
                                    self.mask_threshold_index)
                num_agents = len(self.list_viw_agents)
                elapsed_time = time.time() - start
                print(f"---{elapsed_time} seconds for {num_agents} agents---")

    def timed_callback(self, list_viw_agent, vi_maps, mask_threshold_index):
        
        # for each agent create a goal costmap
        list_agent_costmaps = []
        for viw_agent in list_viw_agent:
            
            list_softmax, list_goal_locs = self.get_softmax_agent(viw_agent)
            # not enough samples for softmax so make all zeros array
            # TODO is this still necesary? 
            if len(list_softmax) == 0:
                list_softmax = [0] * len(list_goal_locs)
            
            list_goal_costmaps = []
            #for each goal create a costmap
            for index, scale in enumerate(list_softmax):
                costmap_goal = self.costmap_goal(viw_agent, 
                                                  vi_maps[index],
                                                  mask_threshold_index,
                                                  list_goal_locs[index])

                costmap_goal = np.multiply(costmap_goal, scale)
                list_goal_costmaps.append(costmap_goal)

            #sum all goal costmaps and add to list of all agent costmaps
            costmap_total = self.costmap_sum_goals(list_goal_costmaps)
            list_agent_costmaps.append(costmap_total)

        #TODO method for combining multiple agents
        costmap_pub = list_agent_costmaps[0]
        occ_grid = ValueIterationWeighting.costmap_to_occ_grid(costmap_pub)
        self.map_pub.publish(occ_grid)
        return None

    def costmap_sum_goals(self, list_goal_costmaps):
        costmap_total = list_goal_costmaps[0]
        for costmap in list_goal_costmaps[1::]:
            costmap_total +=  costmap
        return costmap_total

    def costmap_goal(self, viw_agent, dict_vi_map, threshold_index, goal_loc):
        vi_map = dict_vi_map["grid_world_array"]
        agent_pose = self.get_pose_agent(viw_agent.agent)
        estimated_path = self.path_from_vi_map(dict_vi_map, agent_pose)
        path_mask = self.path_mask_grid_new(estimated_path, 
                                            vi_map,  
                                            threshold_index)
        
        
        #TODO create new costmap from mask
        agent_pose_index = self._loc_to_index(agent_pose, 
                                              dict_vi_map["resolution"],)
        costmap = self.pose_spread_gaussian(agent_pose_index, path_mask, 
                                            goal_loc)
        #costmap = np.multiply(path_mask, vi_map)
        return costmap

    def pose_spread_gaussian(self, agent_pose_index, path_mask, goal_loc):

        cov_pose = [[500, 0], [0, 500]]
        cov_goal = [[200, 0], [0, 200]]
        x, y = np.mgrid[0:200:1, 0:200:1]
        pos = np.dstack((x, y))

        var_pose = multivariate_normal(agent_pose_index, cov_pose)
        var_goal = multivariate_normal(goal_loc, cov_goal)


        gaussian_cost = np.add(var_pose.pdf(pos), var_goal.pdf(pos))
        costmap = np.multiply(path_mask, gaussian_cost)
        return costmap


    def path_mask_grid_new(self, path, vi_map_grid, threshold_index):
        rows, cols = len(vi_map_grid), len(vi_map_grid[0])
        vi_mask = np.zeros((rows, cols))
        dist_scale = len(path)/10

        for index, point in enumerate(path):
            radius = threshold_index + (index/dist_scale)

            edge_points = self.calc_edge_of_circle(point, radius)
            for edge  in edge_points:
                vi_mask[edge[0], edge[1]] = 1
        return vi_mask

    def calc_edge_of_circle(self, point, radius):
        x0 = point[0]
        y0 = point[1]
        edge_points = []

        x_ = np.arange(x0 - radius - 1, x0 + radius + 1, dtype=int)
        y_ = np.arange(y0 - radius - 1, y0 + radius + 1, dtype=int)
        x, y = np.where((x_[:,np.newaxis] - x0)**2 + (y_ - y0)**2 <= radius**2)

        for x, y in zip(x_[x], y_[y]):
                 edge_points.append((x, y))
        return edge_points

    def path_from_vi_map(self, dict_vi_map, pose):

        resolution = dict_vi_map["resolution"]
        grid_world_array = dict_vi_map["grid_world_array"]
        max_value = np.asarray(grid_world_array).max()
        agent_map_index = self._loc_to_index(pose, resolution)
        index = agent_map_index
        list_path = []
        while True:
            index, value = self.next_index(index, grid_world_array)
            prev_value = 0
            #TODO fix the end point issue
            if value >= (max_value - 100):
                break
            else:
                prev_value = value
                list_path.append(index)

        return list_path

    def next_index(self, index, grid_world_array):
        """
        returns next highest index value adjecent        
        Parameters
        ----------
        pose : [type]
            [description]
        """
        value = grid_world_array[index[0]][index[1]]

        for i in range(-1, 2):
            for j in range(-1, 2):
                row = index[0] + i
                col = index[1] + j

                new_val = grid_world_array[row][col]

                if new_val >= value:
                    value = new_val
                    max_index = [row, col]
                
        return max_index, value

    def _loc_to_index(self, loc, resolution):
        index_x = int(loc[0]/resolution)
        index_y = int(loc[1]/resolution)

        return (index_x, index_y)

    def get_pose_agent(self, agent):

        odom_msg = rospy.wait_for_message(agent+"/odom", Odometry)
        (x, y) = odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y
        return (y, x)
   
    def get_softmax_agent(self, viw_agent):
    
        list_softmax, list_goal_locs = viw_agent.softmax_goals()
        
        return list_softmax, list_goal_locs
     
    def spawn_agent(self, agent_name, buffer_size):        
        viw_agent = ValueIterationWeighting(self.vi_maps, 
                                                buffer_size=buffer_size,
                                                agent=agent_name)
        return viw_agent

    def run_agents(self, list_viw_agents):
        for viw_agent in list_viw_agents:
            viw_agent.start_callback()
        num_agents = len(list_viw_agents)
        print(f"----{num_agents} agents have been started-----")


class ValueIterationWeighting(object):
    """
    records agents trajectory creates softmax of predicted goal location
    """
 
    def __init__(self, vi_maps, buffer_size=200, agent="test"):
        """
        initialize VIW agent
        
        Parameters
        ----------
        vi_maps : dict
            dictionary object that contains all VI maps and configs
        buffer_size : int, optional
            length of agent traj, by default 200
        agent : str, optional
            agent name, by default "test"
        """
        self.vi_maps = vi_maps
        self.buffer_size = buffer_size
        self.agent = agent
        #buffer of agent path
        self.agent_path_buffer = collections.deque(maxlen=self.buffer_size)

    def start_callback(self):
        """
        start the odom subscriber for VIW agent
        """
        sub_topic = self.agent + "/odom"
        self.sub = rospy.Subscriber(sub_topic, Odometry, self.odom_callback)

    def odom_callback(self, msg):
        """
        [summary]
        
        Parameters
        ----------
        msg : nav_msgs:Odometry
            agent odometry message
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.agent_path_buffer.appendleft((x, y))

    def softmax_goals(self):
        """
        uses path scoring algorithm to create softmax estimates of intended 
        goal location
        
        Returns
        -------
        list
            list of softmax values and corresponding goal locations
        """
        list_scores = []
        list_goal_locs = []
        #iterate through all goal maps and score them
        for dict_vi_map in self.vi_maps:
            score = self.score_path(dict_vi_map, self.agent_path_buffer)
            list_scores.append(score)
            list_goal_locs.append(dict_vi_map["goal"])

        np.set_printoptions(precision=5)
        list_scores = np.asarray(list_scores)
        list_scores = list_scores / list_scores.max()
        list_softmax = softmax(list_scores)
        print(list_softmax)
        print(list_goal_locs)
        return list_softmax, list_goal_locs
    
    def score_path(self, dict_vi_map, agent_path_buffer):
        """
        algorithm to score the agents trajectory for a goal
        
        Parameters
        ----------
        dict_vi_map : dict
            dictionary object of goal VI map and configs
        agent_path_buffer : collection
            circular buffer of agent 
        
        Returns
        -------
        int
            score for path from goal vi map
        """
        #TODO make directional!
        np_array_values = np.asarray(dict_vi_map["grid_world_array"])
        total_value = 0
        #make agent_path_buffer a list t0 prevent mutation during callback
        for loc in list(agent_path_buffer):
            index = self._loc_to_index(loc, dict_vi_map["resolution"])
            total_value += np_array_values[index[0], index[1]]
        return total_value
        
    def _loc_to_index(self, loc, resolution):
        """
        convert a coordinate location to an array index
        
        Parameters
        ----------
        loc : tuple
            location in meters (x, y)
        resolution : float
            meters per array index
        
        Returns
        -------
        tuple
            index on array of location
        """
        index_x = int(loc[0]/resolution)
        index_y = int(loc[1]/resolution)
        return (index_x, index_y)

    @staticmethod
    def costmap_to_occ_grid(array):
        """
        convert costmap to ros_nav:OccupancyGrid type
        
        Parameters
        ----------
        array : 
            2D array costmap
        
        Returns
        -------
        ros_nav:OccupancyGrid
            ros_nav:OccupancyGrid
        """

        CostMap = OccupancyGrid()
        CostMap.info.resolution = .05
        CostMap.info.width = 200
        CostMap.info.height = 200

        new_range = 100

        world_arr = array
        world_arr_flat = np.asanyarray(world_arr).flatten()
        max_value = np.amax(world_arr_flat)
        min_value = np.amin(world_arr_flat)
        old_range = max_value - min_value
        world_arr_flat = world_arr_flat / (old_range / new_range)
        world_arr_flat = world_arr_flat.astype(np.int8)
        world_arr_flat = world_arr_flat.tolist()
        map_tuple = tuple(world_arr_flat)
        CostMap.data = map_tuple

        return CostMap
