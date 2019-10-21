#!/usr/bin/env python

import rospy
import numpy as np

import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry



class HumanMetrics(object):

    def __init__(self, **kwags):

        options = {
            "human_agent": "human_0",
            "robot_agent": "robot_0"
        }

        self.__dict__.update(options)
