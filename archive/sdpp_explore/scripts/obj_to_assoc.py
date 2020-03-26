#!/usr/bin/env python3

import rospy
import rospkg
import math
import itertools

from sdpp_explore.object_association import ObjectAssoc




class SceneAssociation(object):
    """
    associate objects with nearby objects to find goals

    keyword arguments:
    ------------------
    goal_threshold: float
        threshold for objects to be a part of a goal association

    """

    def __init__(self, goals, obj_locations, **configs):

        self.obj_assoc_config = {"filename": "obj_assoc_test.yaml",
                                 "pkg_name": "sdpp_explore"}
        self.goal_threshold = 0.5

        self.__dict__.update(configs)

        self.obj_assoc_interface = ObjectAssoc(**self.obj_assoc_config)
        self.load_assoc_array()

        print(self.obj_assoc_interface.dict_array_assoc)

        #temporararty static method
        self.static_method(goals, obj_locations)


    def static_method(self, goals, obj_locations):

        self.obj_to_goal_grouping(goals, obj_locations)
        self.save_assoc_array()



    def obj_to_goal_grouping(self, goal_list, obj_list):
        """
        checks through all goals and returns which goals the object is a member off
        
        Parameters
        ----------
        obj : [type]
            [description]
        """
        for goal in goal_list:
            assoc_obj_list = self.obj_from_goal(goal, obj_list, self.goal_threshold)
            # remove duplicates
            assoc_obj_list = list(dict.fromkeys(assoc_obj_list))
            #find all unique pairs of objects
            obj_pairs = list(itertools.combinations(assoc_obj_list, 2))

            for pair in obj_pairs:
                self.add_assoc(pair[0], pair[1])
                print("added_assoc")


    def obj_from_goal(self, goal, obj_list, threshold):
        """
            uses the goal to check which objects are within its threshold
        
        Parameters
        ----------
        goal : [type]
            [description]
        obj_list : [type]
            [description]
        """
        assoc_obj_list = []

        for obj in obj_list:
            dist = self.calculate_distance(obj[1:3], goal)
            if dist <= threshold:
                assoc_obj_list.append(obj[0])        
        return assoc_obj_list


    def calculate_distance(self, loc_0, loc_1):
        loc_0 = [float(i) for i in loc_0]
        loc_1 = [float(i) for i in loc_1]        

        dist = math.sqrt((loc_0[0] - loc_1[0])**2 + (loc_0[1] - loc_1[1])**2 )

        print(dist)
        return dist



    def load_assoc_array(self):
        self.obj_assoc_interface.load_from_config()
        
    def save_assoc_array(self):
        self.obj_assoc_interface.save_to_config()

    def add_assoc(self, obj1, obj2):
        self.obj_assoc_interface.add_assoc(obj1, obj2)

    def sub_assoc(self, obj1, obj2):
        self.obj_assoc_interface.sub_assoc(obj1, obj2)

if __name__ == "__main__":

    goals = rospy.get_param("/goal_locations")
    obj_locations = rospy.get_param("/object_locations")

    test = SceneAssociation(goals, obj_locations)