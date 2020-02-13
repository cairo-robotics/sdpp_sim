#!/usr/bin/env python3

import sys
import unittest
import yaml
import rospy

from sdpp_navigation.value_iter_weighting import ValueIterationWeightingMulti as VIWM
from sdpp_navigation.value_iter_weighting import ValueIterationWeighting as VIW

from nav_msgs.msg import Odometry

VIW_conifg_0 = {"buffer_size": 10
}

VIWM_config_0 = {

}

fp = open("test_vi_maps.yaml", "r")


class TestValueIterWeighting(unittest.TestCase):
    
    @unittest.skip("don't wanna load the map")
    def test_functionality(self):
        narrow_3_goal_vi_maps = yaml.full_load(fp)
    

        VIW_obj = VIW(vi_maps=narrow_3_goal_vi_maps)
        pass

    @unittest.skip("temp skip")   
    def test_odom_callback(self):
        vi_maps_set = yaml.load(fp)

        VIW_obj = VIW(vi_maps=vi_maps_set, **VIW_conifg_0)
        VIW_obj.setup_0()
        pub_odom = rospy.Publisher("/test/odom", Odometry, queue_size=10)

        for i in range(10):
            rospy.sleep(.1)
            send_msg = Odometry()
            send_msg.pose.pose.position.x = i
            send_msg.pose.pose.position.y = i
            pub_odom.publish(send_msg)

    @unittest.skip("temp skip")
    def test_loc_to_index(self):
        VIW_obj = VIW()
        self.assertEquals((1, 1), VIW_obj._loc_to_index((.1, .1), .1))

        

class TestValueIterWeightingMulti(unittest.TestCase):

    def test_run(self):
        vi_map_set = yaml.load(fp)
        VIWM_obj = VIWM(vi_mamps = vi_map_set, **VIWM_config_0)
        pass







if __name__ == '__main__':   
    
    rospy.init_node("test_VIW_node")
    
    unittest.main()

    