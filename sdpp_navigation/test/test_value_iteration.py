#!/usr/bin/env python3

import sys
import unittest
import yaml

from sdpp_navigation.value_iteration import ValueIterationMapManager as VIMM
from sdpp_navigation.value_iteration import LoadMap


VIMM_config0 = {"iter_max": 10

}

fp = open("test_map_occupancy_grid.yaml", "r")
ros_occupancy_grid_narrow_3_goal = yaml.full_load(fp)


class TestLoadMap(unittest.TestCase):
    
    LoadMap_obj = LoadMap()

    def test_gridworld_format_data(self):

        self.LoadMap_obj.gridworld_format_data(ros_occupancy_grid_narrow_3_goal)
        
        



class TestValueIterationMapManager(unittest.TestCase):
    
    VIMM_obj = VIMM(ros_occupancy_grid_narrow_3_goal, **VIMM_config0)

    map_set = []
    @unittest.skip("skipping single instance")
    def test_build_map(self):

        self.VIMM_obj.build_vi_map([1, 1])
        
    @unittest.skip("skipping test build vi map set")
    def test_build_vi_map_set(self):
        VIMM_obj = VIMM(ros_occupancy_grid_narrow_3_goal, **VIMM_config0)


        VIMM_obj.build_vi_map_set([[1, 1], [2, 2]])

    
    def test_save_maps(self):

        VIMM_obj = VIMM(ros_occupancy_grid_narrow_3_goal, **VIMM_config0)

        map_set = VIMM_obj.build_vi_map_set([[5, 5], [5, 50], [5, 95]])
        
        self.assertEqual(len(map_set), 3)

        
        self.VIMM_obj.save_maps(map_set)
        

        

        

    
    def test_load_maps(self):
        pass




        

        


if __name__ == '__main__':   
    
    unittest.main()
