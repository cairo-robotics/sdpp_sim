#!/usr/bin/env python3

import rospy

import math
import numpy as np




class trajAnalyzer(object):
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
    #TODO (45) pull_file_data(filename)
        pull data from a file

    #TODO (20) pull_folder_data(folder)
        pull data from a whole folder

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
    def __init__(self):
        pass



class trajLoader(object):
    """
    loads trajectories from saved pickled files
    #TODO create loader

    all data is pulled as a list of individual trajectories

    keyword arguments
    -----------------
    folder: string
        folder of where trajectory files are stored

    scan_recursive: bool
        whether to scan recursively through file structure
    
    methods
    -------

    #TODO (45) pull_file(filename):
        returns array of data from within specified file
    
    #TODO (45) pull_all():
        returns array of all data from within the folder
    
    #TODO (60) file_list(sub_folder):
            returns array of file locations within the folder/sub_folder
            sub_folder can be blank. locations based on top folder
    
    
    """
        def __init__(self):
            pass