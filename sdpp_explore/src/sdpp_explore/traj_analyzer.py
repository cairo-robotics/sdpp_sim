#!/usr/bin/env python3

import rospy
import rospkg

import math
import yaml
import os
import numpy as np

from sklearn.mixture import GaussianMixture


class TrajAnalyzer(object):
    """
    does the analysis of traj data for the 

    all data is pulled as a list of individual trajectories

    keyword_arguments
    -----------------
    folder: string
        location of trajectory files (relative or absolute?)

    num_goal_proposals: int
        a proposed number of goal locations in a space
    
    static_map: nav_msgs/OccupancyGrid
        world map for use with graphing. if empty no map will be used

    attributes
    ----------
    list_traj_dirty: list
        list of trajectories to be used unsorted

    list_traj_filtered: list
        list of trajectories after being cleaned 
    
    goal_proposals: list
        array of guassian models where goals are likely to be


    methods
    -------

    #TODO (60) traj_proposals()
        return a array of gaussian models of size num_goal_proposals
        using GMM
    
    #TODO (120) traj_smoothing()
        proposed method for removing walking data from traj_array
    
    #TODO (45) proposal_graph(goals)
        take proposed goals and show graph of locations. use seaborn it looks nice

    #TODO (15) write_goals(name, to_file, to_param)
        write goals to config folder or param server based on arguments
    
    """
    def __init__(self, static_map, **config):

        self.pkg = "sdpp_explore"
        self.sub_folder = "/data/"
        self.int_goal_proposals = 5
        self.static_map = static_map

        #update the config
        self.__dict__.update(config)
        
        if "traj_data_config" in config:
            traj_data_config = config["traj_data_config"]
            # i think this will work?
            self.data_interface = TrajDataInterface(**traj_data_config)
        else:
            self.data_interface = TrajDataInterface()


        goal_prop_config = {"pkg": "sdpp_explore"}
        self.goal_interface = GoalProposalInterface(**goal_prop_config)


        goal_loc = self.process_0()

        self.goal_interface.save_goals(goal_loc)



    def process_0(self):
        """
        initial process for goal proposals

        will return goal locs
        """
        raw_data = self.pull_data(self.sub_folder)

        prepped_data = self._prep_data(raw_data)

        print(len(prepped_data))


        gmm = GaussianMixture(5)

        gmm.fit(prepped_data)

        return gmm.means_


    def traj_proposals(self, data, int_goal_proposals = 5):
        
        gmm = GaussianMixture(int_goal_proposals)

        gmm.fit(data)

        return gmm.means_


    def traj_smoothing(self):
        pass


    def proposal_graph(self):
        pass
    

    def pull_data(self, sub_folder):
        """
        wrapping function for TrajDataInterface class to pull data
        
        Parameters
        ----------
        sub_folder : str
            sub folder within package to pull data from
        
        Returns
        -------
        list
            list of raw trajectories
        """        

        data = self.data_interface.load_files_subdirect(sub_folder)

        return data


    def _prep_data(self, data):

        list_trajectories_poses = []
        for traj in data:
            list_poses = []
            for odom in traj["list_odom"]:
                list_poses.append(self._pose_from_odom(odom))    
            list_trajectories_poses.append(list_poses)
        
        prepped_data = self._XY_list(list_trajectories_poses)


        return prepped_data


    def _XY_list(self, data):
        
        list_XY = []

        for traj in data:
            for time_step in traj:
                list_XY.append([time_step[0], time_step[1]])

        return list_XY


    def _pose_from_odom(self, odom):
        """
        takes nav_msg/odometry and extracts [x, y, z] pose
        
        Parameters
        ----------
        odom : nav_msgs/odometry
            ROS odometry msg
        
        Returns
        -------
        list
            list of x, y, z pose
        """                
        pose = odom.pose.pose.position
        return [pose.x, pose.y, pose.z]


    def _twist_from_odom(self, odom):
        """
        takes nav_msg/odometry and extracts [x, y, z] velocity
        
        Parameters
        ----------
        odom : nav_msgs/odometry
            ROS odometry msg
        
        Returns
        -------
        list
            list of x, y, z velocity
        """                
        twist = odom.twist.twist.linear
        return [twist.x, twist.y, twist.z]



class GoalProposalInterface(object):

    def __init__(self, **configs):

        self.pkg = "sdpp_explore"

        self.__dict__.update(configs)


    def save_goals(self, goals):
        print(goals)

        pkg_path = self._pkg_path(self.pkg)
        pathname = pkg_path + "/config/"
        pathname += "goals.yaml"

        self._save_data_yaml(goals.tolist(), pathname)


    def _save_data_yaml(self, data, pathname):
        """
        saves data as a yaml file to the given pathname

        Parameters
        ----------
        data : list or dict
            the data to be written
        pathname : str
            full filepath to save yaml too
        """        
        pathname = self._yaml_extension(pathname)
        with open(pathname, "w") as outfile:
            yaml.dump(data, outfile, default_flow_style=False)


    def _pkg_path(self, pkg):
        """
        returns the file path of a package.

        Parameters
        ----------
        pkg: string
            name of package to get filepath for
        
        Returns
        ------
        pkg_path: string
            the absolute path of desired package config folder
        """
        r = rospkg.RosPack()
        pkg_path = r.get_path(pkg)        
        return pkg_path

    def _yaml_extension(self, string):
        """
        add ".yaml" extension to string if needed

        Parameters
        ----------
        string: str
            input filename for checking

        Returns
        ------
        string: str
            filename with .yaml extension
        """
        if string.endswith(".yaml"):
            pass
        else:
            string += ".yaml"
        return string



class TrajDataInterface(object):
    """
    Interface class designed to unify loading and saving of traj data objects
    
    keyword arguments
    -----------------
    pkg: string
        ROS package from where to load and save data from
    """
  
    def __init__(self, **configs):
        """
        initialize the object. currently only keeps track of pkg location

        """        
        self.pkg = "sdpp_explore"

        self.__dict__.update(configs)


    def set_package(self, pkg):
        """
        set the package attribute. used for finding relative pathing
        
        Parameters
        ----------
        pkg : str
            name of ROS package
        """        
        self.pkg = pkg


    def load_files_subdirect(self, subdirect):

        list_data = []

        full_path = self._pkg_path(self.pkg) + subdirect

        all_files = self.list_of_files(full_path)

        yaml_files = self._purge_except_yaml(all_files)

        for item in yaml_files:

            list_data.append(self._load_data_yaml(item))

        return list_data
    

    def list_files_yaml_of_subdirect(self, subdirect):
        """
        lists all the yaml files within a subdirectory of the pkg attribute
        
        Parameters
        ----------
        subdirect : str
            the name of the subdirectory
        """                
        pkg_path = self._pkg_path(self.pkg)
        all_files = self.list_of_files(pkg_path + subdirect)
        yaml_files = self._purge_except_yaml(all_files)

        return yaml_files


    def list_of_files(self, dirname):
        """
        returns array of all file locations within the dirname 
       
        Parameters
        ----------
        dirname : str
            directory to search for files in
            
        
        Returns
        -------
        list of str
            list of strings with paths
        """        

        list_of_files = os.listdir(dirname)
        all_files = []

        for entry in list_of_files:
            full_path = os.path.join(dirname, entry)

            if os.path.isdir(full_path):
                all_files = all_files + self.list_of_files(full_path)
            else:
                all_files.append(full_path)

        return all_files


    def load_file(self, filepath):
        """
        loads and returns the data from a filepath
        currently only suports yaml
        
        Parameters
        ----------
        filepath : str
            the location of a file
        
        Returns
        -------
        list or dict
            dictionary or list of data from within file
        """
        filepath = self._yaml_extension(filepath)
        data = self._load_data_yaml(filepath)
        return data


    def _purge_except_yaml(self, list_files):
        """
        removes all files that do not have yaml extension
        
        Parameters
        ----------
        list_files : list of str
            flist filepaths to purge
        
        Returns
        -------
        list of str
            list of filepaths with yaml extension
        """        
        yaml_files = []
        for filepath in list_files:
            if filepath.endswith(".yaml"):
                yaml_files.append(filepath)

        return yaml_files


    def _load_data_yaml(self, pathname):
   
        """
        open and load yaml file
        
        Returns
        -------
        list or dict
            contents of yaml file
        """        
        pathname = self._yaml_extension(pathname)

        with open(pathname) as file:
            traj_data = yaml.load(file, Loader=yaml.FullLoader)
        
        return traj_data


    def _save_data_yaml(self, data, pathname):
        """
        saves data as a yaml file to the given pathname

        Parameters
        ----------
        data : list or dict
            the data to be written
        pathname : str
            full filepath to save yaml too
        """        
        pathname = self._yaml_extension(pathname)
        with open(pathname, "w") as outfile:
            yaml.dump(data, outfile, default_flow_style=False)

    
    def _yaml_extension(self, string):
        """
        add ".yaml" extension to string if needed

        Parameters
        ----------
        string: str
            input filename for checking

        Returns
        ------
        string: str
            filename with .yaml extension
        """
        if string.endswith(".yaml"):
            pass
        else:
            string += ".yaml"
        return string

    
    def _pkg_path(self, pkg):
        """
        returns the file path of a package.

        Parameters
        ----------
        pkg: string
            name of package to get filepath for
        
        Returns
        ------
        pkg_path: string
            the absolute path of desired package config folder
        """
        r = rospkg.RosPack()
        pkg_path = r.get_path(pkg)        
        return pkg_path

