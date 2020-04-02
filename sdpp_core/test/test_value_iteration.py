#!/usr/bin/env python3

import unittest
import numpy as np

from sdpp_core.value_iteration import ValueIterationMapManager



vimm_config0 = {"iter_max": 100, "gamma": .9, "epsilon": 10}

np_world_array = np.asarray(
[[1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
 [1, 0, 0, 0, 0, 0, 0, 0, 0, 1], 
 [1, 0, 0, 0, 0, 0, 0, 0, 0, 1], 
 [1, 0, 0, 1, 1, 1, 1, 0, 0, 1], 
 [1, 0, 0, 1, 1, 1, 1, 0, 0, 1], 
 [1, 0, 0, 1, 1, 1, 1, 0, 0, 1], 
 [1, 0, 0, 0, 0, 0, 0, 0, 0, 1], 
 [1, 0, 0, 0, 0, 0, 0, 0, 0, 1], 
 [1, 0, 0, 0, 0, 0, 0, 0, 0, 1], 
 [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]])

np_world_array = np.multiply(np_world_array, 100)

grid_world_map = {"array": np_world_array, 
                  "world_bounds_rows": 10, 
                  "world_bounds_cols": 10,
                  "resolution": 0.1}

goals = [(2, 1), (5, 1), (9, 1)]

class test_value_iteration(unittest.TestCase):
    
        
        

    
    
    def test_1(self):
        vimm_obj = ValueIterationMapManager(**vimm_config0)

        vimm_obj.grid_world_map = grid_world_map
        vimm_obj.goal_locs = goals
        

        vimm_obj.build_vi_map(goals[1])


if __name__ == "__main__":
    
    unittest.main()
