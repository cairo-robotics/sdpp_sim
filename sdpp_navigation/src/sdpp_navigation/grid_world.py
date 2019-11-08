#!/usr/bin/env python

import collections
import pickle
import numpy as np
from pprint import pprint, pformat
import matplotlib.pyplot as plt

import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid



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
                    cell.value = value
                    cell.reward = value

                cells[row_idx].append(cell)
            assert len(cells[row_idx]) == self.world_bounds_cols
        assert len(cells) == self.world_bounds_rows
        self.cells = cells

    def __iter__(self):
        for row in self.cells:
            for cell in row:
                yield cell

    def add_reward_block(self, x, y, size=5, reward=255):

        offset = int(size/2.0)

        for i in range(0, size):
            for j in range(0, size):
                x_pos = x + i - offset
                y_pos = y + j - offset
                self.add_reward(x_pos, y_pos, reward=reward)

    def add_reward(self, x, y, reward = 255):
        self.cells[x][y].reward = reward
        self.cells[x][y].value = reward

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
    def __init__(self, discount_factor, world, epsilon=0.001):
        self.discount_factor = discount_factor
        self.world = world
        self.epsilon = epsilon

    def update(self, state):
        if state.is_terminal or state.blocks:
            return
        max_pv = 0 # probability * value, as used in V(s) calculation
        # Moves in NSEW order
        moves_pvs = [
            0.8*self.world.north(state).value_prev + 0.1*self.world.west(state).value_prev + 0.1*self.world.east(state).value_prev,
            0.8*self.world.south(state).value_prev + 0.1*self.world.west(state).value_prev + 0.1*self.world.east(state).value_prev,
            0.8*self.world.east(state).value_prev + 0.1*self.world.north(state).value_prev + 0.1*self.world.south(state).value_prev,
            0.8*self.world.west(state).value_prev + 0.1*self.world.north(state).value_prev + 0.1*self.world.south(state).value_prev
        ]

        moves_directions = ['north', 'south', 'east', 'west']
        max_idx = np.argmax(moves_pvs)
        max_pv = moves_pvs[max_idx]
        policy = moves_directions[max_idx]
        state.value = self.discount_factor * max_pv + state.reward
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


class ValueIterWeighting(object):

    def __init__(self, static_map_dict=None, pickle_file=None,  **kwags):

        options = {
            "goals_loc":    None,  # list of tuple pairs (x, y)
            "epsilon":      0.0001,
            "gamma":        0.99,
            "iter_max":     100,
            "plot":         None,
            "buffer_size":  200,
            "pub_map":      "/fuse_test"}  # list of strings

        options.update(kwags)

        self.__dict__.update(options)
        self.test_sum = [0] * 2
        self.likelihood_buffer = [collections.deque(maxlen=self.buffer_size), collections.deque(maxlen=self.buffer_size) ]
        self.pub_map = rospy.Publisher(self.pub_map, OccupancyGrid, queue_size=1)


        if pickle_file is not None:
            self._unpickle_obj_dict(pickle_file)
            return

        if static_map_dict is None:
            print("need static map to run algorithm")
            return

        self.static_map_dict = static_map_dict

        if self.goals_loc is None:
            print("No goals for ValueIterWeighting")
            return

        num_goals = len(self.goals_loc)
        print("ValueIterWeighting started with %d goals" % num_goals)


        map_array = static_map_dict["array"]
        world_bounds_cols = static_map_dict["world_bounds_cols"]
        world_bounds_rows = static_map_dict["world_bounds_rows"]

        self.grid_worlds_array = []
        self.algo_array = []
        for goal in self.goals_loc:
            grid_world = GridWorld(map_array, world_bounds_cols, world_bounds_rows)

            grid_world.add_reward_block(goal[0], goal[1])

            algo = ValueIterationAlgo(self.gamma, grid_world)

            iter_cnt = 0

            while True:
                print('iteration {}'.format(iter_cnt).center(72, '-'))

                for cell in grid_world:
                    algo.update(cell)

                if iter_cnt >= self.iter_max:
                    break
                if algo.done():
                    break

                algo.update_values(grid_world)
                iter_cnt += 1

            if self.plot is not None:
                for type in self.plot:
                    grid_world.plot_world(type)

            self.grid_worlds_array.append(grid_world)


    def bayesian_path_matching(self, agent):

        sub_topic = agent + "/odom"
        self.agent_subscriber = rospy.Subscriber(sub_topic, Odometry, self.bayes_callback)

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

        print "likely goal: " + str(index_max) + " by: "+ str(percent) + " percent"

        map_0 = self.grid_worlds_array[0].value_as_array()
        map_1 = self.grid_worlds_array[1].value_as_array()

        print percent_array

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



    def pickle_obj_dict(self, filename):
        print("dumping pickle")
        self.pub_map = None
        pickle.dump(self.__dict__, open(filename, "wb"))
        print("pickle dumped")

    def _unpickle_obj_dict(self, filename):

        print("pickle loading")
        self.__dict__.update(pickle.load(open(filename, "rb")))
        self.pub_map = rospy.Publisher(self.pub_map, OccupancyGrid, queue_size=1)
        print("pickle loaded")


    def spawn_goal_world(self, goal_loc):
        pass

