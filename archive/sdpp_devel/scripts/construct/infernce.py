#!/usr/bin/env python

import math
import numpy as np
import rospy

from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String
from nav_msgs.msg import Odometry

class PathInference(object):

    def __init__(self, goal_dict):
        self.goal_locations = []

        for key in goal_dict:
            self.goal_locations.append(goal_dict[key]["loc"])

    def ros_odometry_to_line(self, odom):
        a = odom.twist.twist.linear.x
        b = odom.twist.twist.linear.y
        c = -(a*odom.pose.pose.position.x + b * odom.pose.pose.position.y)
        return np.array([a, b, c])

    def dist_point_line(self, line, point):
        pass

    def traj_likelihood(self, traj):
        pass

    def goal_points_norm_diff(self, X, Y):

        euler_diff = np.zeros(len(self.goal_locations))
        for index, goal in enumerate(self.goal_locations):
            diff = math.sqrt((goal[0] - X)**2 + (goal[1] - Y)**2 )
            if diff == 0.0:
                diff = .01
            euler_diff[index] = diff

        temp = sum(euler_diff)/euler_diff
        norm_diff = temp/sum(temp)

        return norm_diff

class GoalLikelihood(object):

    def __init__(self):
        pass


def distance_func(point, line):

    denom = math.sqrt(line[0]**2 + line[1]**2)

    numer = abs(line[0]*point[0] + line[1]*point[1] + line[2])

    return numer/denom


if __name__ == '__main__':

    rospy.init_node("test")
    # association goes [fridge, microwave, coffee_machine, cabinet, tables]
    goal_proposals = {1: {"obj": ["fridge"], "loc": [-3.5, 4.3],
                          "assoc": [.65, .1, .1, 0.075, 0.075]},
                      2: {"obj": ["microwave"], "loc": [-1.0, 4.3],
                          "assoc": [0.18, .65, .05, .06, .07 ]},
                      3: {"obj": ["coffee_machine"], "loc": [0.5, 4.3],
                          "assoc": [.06, .06, .8, .06, .06]},
                      4: {"obj": ["cabinet"], "loc": [1.1, 4.3],
                          "assoc": [.06, .06, .06, .8, .06]},
                      5: {"obj": ["table"], "loc": [-1.0, 1.5],
                          "assoc": [.06, .06, .06, .06, .8]}
                      }
    #test = PathInference(goal_proposals)

    while not rospy.is_shutdown():

        goal = rospy.wait_for_message("human_goal", String)



        model_states = rospy.wait_for_message("/gazebo/model_states", ModelStates)

        human_index = model_states.name.index("human_agent")

        vel_x = model_states.twist[human_index].linear.x
        vel_y = model_states.twist[human_index].linear.y

        pos_x1 = model_states.pose[human_index].position.x
        pos_y1 = model_states.pose[human_index].position.y

        pos_x2 = pos_x1 + vel_x
        pos_y2 = pos_y1 + vel_y

        a = pos_y2 - pos_y1
        b = pos_x2 - pos_x1
        c = pos_x2*pos_y1 - pos_y2*pos_x1
        line = [a, b, c]

        all_dist = np.zeros(len(goal_proposals))
        for index, item in enumerate(goal_proposals):
            item_dict = goal_proposals[item]
            item_name = item_dict["obj"]
            item_loc =  item_dict["loc"]
            distance = distance_func(item_loc, line)
            all_dist[index] = distance

            if item_name[0] == goal.data:
                assoc_dist = item_dict["assoc"]

            elif goal.data ==  "start":
                assoc_dist = [0.0, 0.0, 0.0, 0.0, 0.0]




        temp = sum(all_dist)/all_dist
        distance_like = temp/sum(temp)

        assoc_like = np.zeros(len(goal_proposals))

        assoc_like = distance_like * assoc_dist

        traj_max = -1000.0
        assoc_max = -1000.0
        for index, item in enumerate(goal_proposals):

            item_dict = goal_proposals[item]
            item_name = item_dict["obj"]
            print item_name[0]
            print "traj likelihood: ", distance_like[index]
            print "w/ assoc likelihood: ", assoc_like[index]

            if   >= traj_max:
                traj_max = distance_like
                traj_winner = item_name[0]

            """
            if assoc_like[index] > assoc_max:
                assoc_max = assoc_like[index]
                assoc_winner = item_name[0]
            """



        print "winner traj: ", traj_winner
        print "winner assoc:", assoc_winner
        print "true goal:", goal.data


        rospy.sleep(1)











    print("end")