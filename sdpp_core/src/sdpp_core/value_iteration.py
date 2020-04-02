#!/usr/bin/env python

import collections
import pickle
import yaml
import numpy as np
from pprint import pprint, pformat
import matplotlib.pyplot as plt

import rospy

from nav_msgs.srv import GetMap

from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid


import copy



class ValueIteration(object):

    def __init__(self, state_space, goal, gamma=0.01, epsilon=0.001):
        
        self.gamma = gamma
        self.epsilon = epsilon


        
        vp, ss = self.state_space_to_vi_space(state_space, goal)
        self.vp = vp  #value policy
        self.ss = ss

    def state_space_to_vi_space(self, state_space, goal):
        
        rows, cols = state_space.shape

















class Cell(object):
    def __init__(self):
        self.is_terminal = False
        self.policy = ''
        self.value = 0
        self.value_prev = 0
        self.reward = 0
        self.blocks = False

    def __str__(self):
        return pformat(self.__dict__)

    def __repr__(self): return self.__str__()


class GridWorld(object):

    def __init__(self, static_map, world_bounds_cols, world_bounds_rows):

        self.world_bounds_rows = world_bounds_rows
        self.world_bounds_cols = world_bounds_cols

        self.array = static_map

        cells = []
        for row_idx, row in enumerate(self.array):
            cells.append([])
            for col_idx, value in enumerate(row):
                cell = Cell()
                cell.row = row_idx
                cell.col = col_idx

                #white
                if value == 0:
                    cell.value = 0
                #black
                elif value == 100:
                    cell.value = 0
                    cell.blocks = True

                else:
                    cell.value_prev = value
                    cell.value = value
                    cell.reward = value
                    cell.is_terminal = True

                cells[row_idx].append(cell)
            assert len(cells[row_idx]) == self.world_bounds_cols
        assert len(cells) == self.world_bounds_rows
        self.cells = cells

    def __iter__(self):
        for row in self.cells:
            for cell in row:
                yield cell

    def add_reward_block(self, x, y, size=1, reward=1000):

        offset = int(size/2.0)

        for i in range(0, size):
            for j in range(0, size):
                x_pos = x + i - offset
                y_pos = y + j - offset
                self.add_reward(x_pos, y_pos, reward=reward)

    def add_reward(self, x, y, reward = 255):
        self.cells[x][y].reward = reward
        self.cells[x][y].value = reward
        self.cells[x][y].is_terminal = True

    def north(self, cell):
        row = cell.row; col = cell.col
        if row > 0:
            row -= 1
        return self._next_cell_if_not_blocked(cell, row, col)

    def south(self, cell):
        row = cell.row; col = cell.col
        if row < self.world_bounds_rows-1:
            row += 1
        return self._next_cell_if_not_blocked(cell, row, col)

    def west(self, cell):
        row = cell.row; col = cell.col
        if col > 0:
            col -= 1
        return self._next_cell_if_not_blocked(cell, row, col)

    def east(self, cell):
        row = cell.row; col = cell.col
        if col < self.world_bounds_cols-1:
            col += 1
        return self._next_cell_if_not_blocked(cell, row, col)

    def _next_cell_if_not_blocked(self, cell, row, col):
        n = self.cells[row][col]
        if n.blocks:
            return cell
        return n

    def policy(self):
        return [[c.policy for c in rows] for rows in self.cells]

    def value_prev_as_array(self):
        a = [[cell.value_prev for cell in row] for row in self.cells]
        return np.array(a)

    def value_as_array(self):
        a = [[cell.value for cell in row] for row in self.cells ]
        return np.array(a)

    def walls_as_array(self):
        a = [[cell.blocks for cell in rows] for rows in self.cells]
        return np.array(a).astype(int)

    def reward_as_array(self):
        a = [[cell.reward for cell in rows] for rows in self.cells]
        return np.array(a).astype(int)

    def plot_world(self, type="value_map"):

        if type is "value_map":
            plt.imshow(self.value_as_array(), cmap="plasma")

        elif type is "wall_map":
            plt.imshow(self.walls_as_array(), cmap="plasma")

        elif type is "reward_map":
            plt.imshow(self.reward_as_array(), cmap="plasma")

        else:
            print("plot_world requires value_map, wall_map, reward_map selection")
            return

        plt.colorbar()
        plt.show()

    def __str__(self):
        return pformat(self.value_as_array())

    def __repr__(self): return self.__str__()


class ValueIterationAlgo(object):
    """
        Value iteration algorithm. Uses grid world object to instanciate grid word

        keyword arguments
        -----------------

        world: GrirWorld
        
        discount_factor: float
        
        epsilon: float

    """
    def __init__(self, gamma, world, epsilon=0.001):
        self.gamma = gamma
        self.world = world
        self.epsilon = epsilon

    def update(self, state):
        if state.is_terminal or state.blocks:
            return
        max_pv = 0 # probability * value, as used in V(s) calculation
        # Moves in NSEW order

        moves_pvs = [self.world.north(state).value_prev,
                     self.world.south(state).value_prev,
                     self.world.east(state).value_prev,
                     self.world.west(state).value_prev]

        """
        moves_pvs = [
            0.8*self.world.north(state).value_prev + 
            0.1*self.world.west(state).value_prev + 
            0.1*self.world.east(state).value_prev,

            0.8*self.world.south(state).value_prev + 
            0.1*self.world.west(state).value_prev + 
            0.1*self.world.east(state).value_prev,

            0.8*self.world.east(state).value_prev + 
            0.1*self.world.north(state).value_prev + 
            0.1*self.world.south(state).value_prev,

            0.8*self.world.west(state).value_prev + 
            0.1*self.world.north(state).value_prev + 
            0.1*self.world.south(state).value_prev
        ]"""

        moves_directions = ['north', 'south', 'east', 'west']
        max_idx = np.argmax(moves_pvs)
        max_pv = moves_pvs[max_idx]
        policy = moves_directions[max_idx]
        state.value = self.gamma * max_pv + state.reward
        state.policy = policy

    def update_values(self, world):

        for row_idx, row in enumerate(world.cells):
            for col_idx, cell in enumerate(row):
                cell.value_prev = cell.value

    def done(self):
        prev = self.world.value_prev_as_array()
        current = self.world.value_as_array()
        diff = np.abs(prev - current)
        return not (diff > self.epsilon).any()

    def __str__(self):
        return pformat(self.__dict__)

    def __repr__(self): return self.__str__()


class ValueIterationMapManager(object):
    """
        Manages the VI maps to allow for the rapid creation and design of the 
        worlds

        maps are stored in a dictionary with the goal loc being the key
    """

    def __init__(self, ros_occupancy_grid, iter_max=10000, epsilon=0.0001, gamma=0.99,
                 goal_locs =None):
        self.ros_occupancy_grid = ros_occupancy_grid
        self.iter_max = iter_max
        self.epsilon = epsilon
        self.gamma = gamma
        self.goal_locs = goal_locs

        self.LoadMap_obj = LoadMap()
        self.grid_world_map = self.gridworld_format_data(self.ros_occupancy_grid)

    def build_vi_map_set(self, list_goals):
        
        goal_vi_maps = []
        for goal in list_goals:
            goal_vi_maps.append(self.build_vi_map(goal))
        return goal_vi_maps

    def build_vi_map(self, goal):
        
        map_array = self.grid_world_map["array"]
        world_bounds_cols = self.grid_world_map["world_bounds_cols"]
        world_bounds_rows = self.grid_world_map["world_bounds_rows"]

        grid_world = GridWorld(map_array, world_bounds_cols, world_bounds_rows)
        grid_world.add_reward_block(goal[0], goal[1])
        algo = ValueIterationAlgo(self.gamma, grid_world, self.epsilon)

        iter_cnt = 0
        while True:
                print('iteration {}'.format(iter_cnt).center(72, '-'))

                for cell in grid_world:
                    algo.update(cell)

                if iter_cnt >= self.iter_max:
                    break
                if algo.done():
                    pass
                    break

                algo.update_values(grid_world)
                iter_cnt += 1
                
        
        grid_world_array = grid_world.value_as_array()
        
        #print(grid_world)

        #grid_world.plot_world()
        return {"grid_world_array": grid_world_array.tolist(), 
                "goal": goal,
                "gamma": self.gamma,
                "epsilon": self.epsilon,
                "iterations": self.iter_max,
                "world_bounds_cols": world_bounds_cols,
                "world_bounds_rows": world_bounds_rows,
                "resolution": self.grid_world_map["resolution"]}
    
    
    def gridworld_format_data(self, odometry_map):

        width = odometry_map.info.width
        height = odometry_map.info.height
        resolution = odometry_map.info.resolution

        numpy_array = np.asarray(odometry_map.data)
        numpy_array = numpy_array.reshape(width, height)

        grid_world = {"array": numpy_array, 
                      "world_bounds_rows": width, 
                      "world_bounds_cols": height,
                      "resolution": resolution}

        return grid_world

    def load_maps(self):
        pass
        

    def save_maps(self, list_vi_maps):

        fp = open("test_vi_maps.yaml", "w")
        yaml.dump(list_vi_maps, fp)
        fp.close()


class LoadMap(object):
    def __init__(self, file_location=None):

        self.static_odom_map = None

        rospy.loginfo(rospy.get_name() + ": getting map from map server")
        """
        if file_location is None:
            self.static_odom_map = self.load_from_static_map_srv()
        else:
            self.static_odom_map = self.load_from_file_location_map(file_location)
        """

    @staticmethod
    def load_from_static_map_srv(srv_name='/static_map'):
        rospy.wait_for_service(srv_name)
        static_map_srv = rospy.ServiceProxy(srv_name, GetMap)

        try:
            static_odom_msg = static_map_srv().map
            rospy.loginfo(rospy.get_name() + ": received static map")

        except rospy.ServiceException as exc:
            rospy.logerr(rospy.get_name() + ": unable to get static map" 
                        + str(exc))
            return None

        return static_odom_msg

    @staticmethod
    def load_from_file_location_map(file_location):

        rospy.loginfo(rospy.get_name() + ": load map directly not finished")
        im_frame = Image.open('block_room.png')
        np_frame = np.array(im_frame)
        np_frame = np_frame[:, :, 0]
        np_frame = np.rot90(np_frame, 3)

        return np_frame

 

    def __str__(self):
        return pformat(self.static_odom_map)
   



if __name__ == '__main__':

    rospy.init_node("test_node")

    data = LoadMap.load_from_file_location_map("narrow_3_goal_10x10.p")

    print(data)





