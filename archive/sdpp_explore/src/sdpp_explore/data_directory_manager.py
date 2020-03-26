#!/usr/bin/env python3

import rospy
import rospkg

import os 


from datetime import date


class DataDirectoryManager(object):

    def __init__(self, **configs):
        """
        heirarchy defined:
        - os path to data folder
         - date
          - run of day
            - data
        """

        self.date = None
        self.os_path = None
        self.run_of_day = None
        
        self.__dict__.update(**configs)        

        if self.os_path == None:
            r = rospkg.RosPack()
            package_path = r.get_path("sdpp_explore")
            self.os_path = package_path.replace("sdpp_explore", "") + "data/"
        else:
            rospy.loginfo("custom os path used")

        if self.date == None:
            today = date.today()
            self.date = today.strftime("%d-%m-%y") + "/"
        else:
            rospy.loginfo("custom date path used")

        if self.run_of_day == None:
            search_directory = self.os_path + self.date

            self.create_directory(search_directory)

            if not self.empty_directory(search_directory):
                list_of_runs = self.subdirectories_in_directory(search_directory)
                max_run = 1
                for run in list_of_runs:
                    if int(run) >= max_run:
                        max_run = int(run)          
                
                self.run_of_day = int(run+1) + "/"

            else:
                self.run_of_day = "1/"


    def is_directory(self, directory):
        return os.path.isdir(directory)


    def subdirectories_in_directory(self, directory):
        return next(os.walk(directory))[1]

    def empty_directory(self, directory):
        if len(os.listdir(directory)) == 0 or not self.is_directory(directory):
            return True
        else:
            return False

    def create_directory(self, directory):
        if self.is_directory(directory):
            return
        else:
            try:
                os.mkdir(directory)
            except OSError:
                rospy.logerr("could not create directory {}".format(directory))


            

if __name__ == '__main__':
    rospy.init_node("test_write_node")

    test = FileHeirarchy()





