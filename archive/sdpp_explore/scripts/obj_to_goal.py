#!/usr/bin/env python3

import rospy
import rospkg
import math
import itertools

from sdpp_explore.object_association import ObjectAssoc



class GoalFromObjects(object):

    def __init__(self, **config):
        
        self.obj_assoc_config = {"filename": "obj_assoc_test.yaml",
                                 "pkg_name": "sdpp_explore"}
        
        self.float_obj_goal_threshold = 0.5

        self.__dict__.update(config)

        self.obj_assoc_interface = ObjectAssoc(**self.obj_assoc_config)
        self.load_assoc_array()

        self.list_objects = self.get_object_list()
        goal_proposals = self.goal_proposals_0()

        print(goal_proposals)


    def goal_proposal_method_0(self):

        goal_proposals = []

        for obj in self.list_objects:
            list_obj_nearby = self.find_nearest_objects(obj, 
                                                        self.list_objects)
            if list_obj_nearby:
                list_assoc_obj = self.check_associations(list_obj_nearby)
            else:
                print(str(obj) + "no near neighbors")
                continue
            
            if list_assoc_obj:
                median_pairs = self.median_of_pairs(list_assoc_obj)
            else:
                print(str(obj) + "no associated objects")
                continue

            N = len(median_pairs)

            x_tot = 0
            y_tot = 0
            for pair in median_pairs:
                x_tot += pair[0]
                y_tot += pair[1]
            x_median = x_tot/N
            y_median = y_tot/N
            goal_proposals.append((x_median, y_median))

        return goal_proposals


    def median_of_pairs(self, list_obj_pairs):
        
        median_pairs = []
        for obj_pair in list_obj_pairs:
            obj_pair[0]
            obj_pair[1]

            median_x = (obj_pair[0][1] + obj_pair[1][1])/1.0
            median_y = (obj_pair[0][2] + obj_pair[1][2])/1.0

            median_pairs.append((median_x, median_y))
        return median_pairs




    def check_associations(self, list_obj):
        
        obj_pairs = []
        for obj1 in list_obj:
            for obj2 in list_obj:
                bool_assoc = self.obj_assoc_interface.check_assoc(obj1[0], obj2[1])
                if bool_assoc:
                    obj_pairs.append([obj1, obj2])

                else:
                    pass
        return obj_pairs



    def find_nearest_objects(self, obj_test, obj_list, threshold=0.5):
        """
            object is tuple of (object, x, y)
            obj_list is a list of these tuples
        """
        
        list_obj_nearby = []

        for obj in obj_list:
            dist = self.calculate_distance(obj_list[1:3], obj[1:3])
            if dist > threshold:
                pass
            else:
                list_obj_nearby.append(obj)
        
        return list_obj_nearby



    def calculate_distance(self, loc_0, loc_1):
        loc_0 = [float(i) for i in loc_0]
        loc_1 = [float(i) for i in loc_1]        

        dist = math.sqrt((loc_0[0] - loc_1[0])**2 + (loc_0[1] - loc_1[1])**2 )

        print(dist)
        return dist 

            

    def get_object_list(self):
        """
        use a rosparam method for now
        """

        return rospy.get_param("/object_locations")


    def load_assoc_array(self):
        self.obj_assoc_interface.load_from_config()
        
    def save_assoc_array(self):
        self.obj_assoc_interface.save_to_config()
    
    def add_assoc(self, obj1, obj2):
        self.obj_assoc_interface.add_assoc(obj1, obj2)

    def sub_assoc(self, obj1, obj2):
        self.obj_assoc_interface.sub_assoc(obj1, obj2)




if __name__ == "__main__":

    test = GoalFromObjects()