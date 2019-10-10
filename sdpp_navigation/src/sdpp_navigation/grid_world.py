#!/usr/bin/env python
import numpy as np
from pprint import pprint, pformat

class Cell(object):
    def __init__(self):
        self.is_terminal = False
        self.policy = ''
        self.value = 0
        self.reward = 0
        self.blocks = False

    def __str__(self):
        return pformat(self.__dict__)
    def __repr__(self): return self.__str__()


class GridWorld(object):
    def __init__(self, array, world_bounds_rows, world_bounds_cols):
        self.world_bounds_rows = world_bounds_rows
        self.world_bounds_cols = world_bounds_cols

        cells = []
        for row_idx, row in enumerate(array):
            cells.append([])
            for col_idx, value in enumerate(row):
                cell = Cell()
                cell.row = row_idx
                cell.col = col_idx

                #wall
                if value == 255:
                    cell.value = 0

                elif value == 0:
                    cell.value = 0
                    cell.block = True

                else:
                    cell.value = value
                    cell.reward = value

                cells[row_idx].append(cell)
            assert len(cells[row_idx]) == world_bounds_cols
        assert len(cells) == world_bounds_rows

        self.cells = cells

    def policy(self):
        return [[c.policy for c in rows] for rows in self.cells]

    def __iter__(self):
        for row in self.cells:
            for cell in row:
                yield cell

    def as_array(self):
        a = [ [cell.value for cell in row] for row in self.cells ]
        return np.array(a)

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

    def __str__(self):
        return pformat(self.as_array())
    def __repr__(self): return self.__str__()


class ValueIterationAlgo(object):
    def __init__(self, discount_factor, world):
        self.discount_factor = discount_factor
        self.world = world

    def update(self, state):
        if state.is_terminal or state.blocks: return
        max_pv = 0 # probability * value, as used in V(s) calculation
        # Moves in NSEW order
        moves_pvs = [
            0.8*self.world.north(state).value + 0.1*self.world.west(state).value + 0.1*self.world.east(state).value,
            0.8*self.world.south(state).value + 0.1*self.world.west(state).value + 0.1*self.world.east(state).value,
            0.8*self.world.east(state).value + 0.1*self.world.north(state).value + 0.1*self.world.south(state).value,
            0.8*self.world.west(state).value + 0.1*self.world.north(state).value + 0.1*self.world.south(state).value
        ]
        moves_directions = ['north', 'south', 'east', 'west']
        max_idx = np.argmax(moves_pvs)
        max_pv = moves_pvs[max_idx]
        policy = moves_directions[max_idx]
        state.value = self.discount_factor * max_pv + state.reward
        state.policy = policy
##        print locals()

    def __str__(self):
        return pformat(self.__dict__)
    def __repr__(self): return self.__str__()




def done():
    global world, prev_world
    # If NaNs are in the inputs, we may get a warning. Ignore it and it won't affect the epsilon comparison because np.nan > epsilon is False
    try:
        diff = np.abs(world.as_array() - prev_world)
    except RuntimeWarning:
        pass
    return not (diff > epsilon).any()

epsilon = 0.0001
discount_factor = 0.9

def part_b():
    global world, prev_world
    world = GridWorld(world_input)
    prev_world = np.ones_like(world_input) # init to something other than input
    algo = ValueIterationAlgo(discount_factor, world);
    iter_cnt = 0
    while(True):
        print 'iteration {}'.format(iter_cnt).center(72, '-')
        pprint(prev_world)
        prev_world = world.as_array()
        pprint(prev_world)
        for cell in world:
            algo.update(cell)
        ##if done(): break
        if iter_cnt >= 2:
            break

        iter_cnt += 1

    world_arr = world.as_array()
    pprint(world_arr)
    pprint(np.round(world_arr))
    pprint(world.policy())

def part_c():
    global world, prev_world
    discount_factor = 0.9
    while(True):
        world = GridWorld(world_input)
        prev_world = np.ones_like(world_input) # init to something other than input
        algo = ValueIterationAlgo(discount_factor, world)
        iter_cnt = 0
        while(True):
            print 'iteration {}, discount={}'.format(
                iter_cnt, discount_factor).center(72, '-')
            pprint(prev_world)
            if done(): break
            prev_world = world.as_array()
            pprint(prev_world)
            for cell in world:
                algo.update(cell)
            iter_cnt += 1
        world_arr = world.as_array()
        pprint(world_arr)
        pprint(np.round(world_arr))
        pprint(world.policy())
        if world.cells[2][3].policy != 'west': break
        discount_factor -= 0.01


#print 'part b:'
#part_b()

#print 'part c:'
#part_c()