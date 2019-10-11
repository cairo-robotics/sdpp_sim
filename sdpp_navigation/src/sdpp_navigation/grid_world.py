#!/usr/bin/env python
import numpy as np
from pprint import pprint, pformat

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

                #white
                if value == 255:
                    cell.value = 0
                #black
                elif value == 0:
                    cell.value = 0
                    cell.blocks = True

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

    def walls_as_array(self):
        a = [[cell.blocks for cell in rows] for rows in self.cells]
        return np.array(a).astype(int)

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
##        print locals()

    def update_values(self, world):

        for row_idx, row in enumerate(world.cells):
            for col_idx, cell in enumerate(row):
                cell.value_prev = cell.value


    def __str__(self):
        return pformat(self.__dict__)
    def __repr__(self): return self.__str__()



def value_iteration(world, world_reward,gamma = .99):

    v = np.zeros(world.shape)
    print world
    print world.shape
    print v.shape

    max_iterations = 20
    eps = 1e-20

    for i in range(max_iterations):
        prev_v = np.copy(v)
        for i in range(0, world.shape[0]):
            for j in range(0, world.shape[1]):
                #value in each action
                q_sa = [0] * np.zeros(len(nA))
                for index, A in enumerate(nA):
                    row_A = i + A[0]
                    col_A = j + A[1]
                    try:
                        #if
                        q_sa[index] = world_reward[i, j] + gamma * (A[2]*prev_v[row_A, col_A])

                    except IndexError as error:
                        q_sa[index] = -1

                v[i, j] = max(q_sa)

    plt.imshow(v, cmap="plasma")
    plt.colorbar()
    plt.show()