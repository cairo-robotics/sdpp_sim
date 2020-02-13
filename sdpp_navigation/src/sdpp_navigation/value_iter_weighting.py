#!/usr/bin/env python3

import rospy
import collections
import pickle
import yaml
import numpy as np
#from sdpp_navigation.value_iteration import ValueIterationAlgo
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid

from scipy.special import softmax


# TODO (180) miscelanious level one work (will be found later)
    
# TODO (120) 2 miscelanious level 2 work

class ValueIterationWeightingMulti(object):
    """"
   

    methods
    -------

  
    """

    def __init__(self, vi_maps=None, **configs):
        
        self.pub_map = "/test_map"
        self.agent_list = ["human_0", "human_1"]
        self.VIW_config = {"buffer_size": 10
        }

        self.__dict__.update(configs)

        self.vi_maps = vi_maps
        self.list_VIW_agents = []
        for agent in self.agent_list:
            self.list_VIW_agents.append(self.spawn_agent(agent))
    
    def spawn_agent(self, agent_name):
        
        local_VIW_config = self.VIW_config.copy()
        local_VIW_config["agent"] = agent_name
        spawned_agent = ValueIterationWeighting(self.vi_maps, **local_VIW_config)
        return spawned_agent



    def run_agents(self):
        for viw_obj in self.list_VIW_agents:
            viw_obj.setup_0()


    def test_function(self, test_value):
        return True



class ValueIterationWeighting(object):
    """
    ranks likelihood of goals

    keyword arguments
    -----------------
    goals_loc: list of tuple

    epsilon: float
    
    gamma: float
    
    iter_max: float

    plot: bool

    buffer_size: int

    pub_map: str
        topic to publish map too

    """

    def __init__(self, vi_maps=None, **config):       
        self.epsilon =  0.0001
        self.gamma =     0.99
        self.iter_max = 100
        self.plot =     None
        self.buffer_size =  200
        self.filename = ""
        self.agent = "test"
        self.samples_per_likelihood = 5
        self.samples = 0
        self.vi_maps = vi_maps
        self.__dict__.update(**config)

        self.circ_buff_agnt_pos = collections.deque(maxlen = self.buffer_size)


    def setup_0(self):

        self.num_goals = len(self.vi_maps)
        rospy.Subscriber(self.agent + "/odom", Odometry, self.odom_callback)


    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.circ_buff_agnt_pos.appendleft((x, y))
        self.samples += 1

        if self.samples >= self.samples_per_likelihood:
            self.samples = 0
            self.list_softmax, self.list_goals = self.score_goals()
        else:
            return


    def score_goals(self):
        """
        features:
        ranking likelihood by goal
        
        """
        list_likelihoods = []
        list_goals = []
        #iterate through all goal maps and score them
        for dict_map in self.vi_maps:
            likelihood = self.total_points_map_odom(dict_map, self.circ_buff_agnt_pos)
            goal_loc = dict_map["goal"]
            list_likelihoods.append(likelihood)
            list_goals.append(goal_loc)
        list_softmax = self._softmax_likelihood_list(list_likelihoods)
        return list_softmax, list_goals
    
    def _softmax_likelihood_list(self, list_likelihoods):
        
        np.set_printoptions(precision=5)
        np_softmax = softmax(np.asarray(list_likelihoods))
        return np_softmax


    def total_points_map_odom(self, dict_map, iter_odom):

        np_array_values = np.asarray(dict_map["grid_world_array"])

        total_value = 0
        for loc in iter_odom:
            index = self._loc_to_index(loc, dict_map["resolution"])
            total_value += np_array_values[index[0], index[1]]

                
        return total_value
        


    def _loc_to_index(self, loc, resolution):
        index_x = int(loc[0]/resolution)
        index_y = int(loc[1]/resolution)

        return (index_x, index_y)


         
    def load_maps(self, filename):

        fp = open(filename)

        return yaml.full_load(fp)




    def bayes_callback(self, msg):

        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        x = int(x*20)
        y = int(y*20)

        #wtf why are my maps all flipped D:
        map_0_cost = self.grid_worlds_array[0].cells[y][x].value
        map_1_cost = self.grid_worlds_array[1].cells[y][x].value


        self.likelihood_buffer[0].append(map_0_cost)
        self.likelihood_buffer[1].append(map_1_cost)

        likelihood_cost = [0, 0]
        likelihood_cost[0] = sum(value for value in self.likelihood_buffer[0])
        likelihood_cost[1] = sum(value for value in self.likelihood_buffer[1])

        max_value = max(likelihood_cost)
        index_max = likelihood_cost.index(max_value)
        percent = (max_value + .1)/(sum(likelihood_cost)+.1) * 100

        sum_cost = sum(likelihood_cost)+.1
        likelihood_cost = np.asarray(likelihood_cost)
        percent_array = np.add(likelihood_cost, 0.1)/sum_cost

        print("likely goal: " + str(index_max) + " by: "+ str(percent) + " percent")

        map_0 = self.grid_worlds_array[0].value_as_array()
        map_1 = self.grid_worlds_array[1].value_as_array()

        print(percent_array)

        map_0 = map_0*percent_array[0]
        map_1 = map_1*percent_array[1]

        map_pub = np.add(map_0, map_1)

        self.pub_map.publish(self.array_to_costmap(map_pub))


    def array_to_costmap(self, array):

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


