#!/usr/bin/env python
import rospkg
import rospy

import yaml

import numpy as np
import matplotlib.plot as plt
import seaborn as sns



class ObjectAssocManager(object):
    """
        manages the object associtaion array for
        don't think I'm going to do this anymore
        might consider doing this at a later time.

        Keyword arguments
        -----------------



    """
    def __init__(self):
        pass




class ObjectAssoc(object):
    """
    Object association array

    keyword arguments:
    ------------------
    init_array: np.array(NxN)
        an initial array to work from

    Note
    ----

    TODO (30) 1 marginal_assoc(obj1, obj2):
        marginal probability of assoctiation

    TODO (30) 1 conditional_assoc(obj1, obj2):
        conditional association

    """

    def __init__(self, **config):
        #degault attributes
        self.list_objects = ["table", "glass", "coffee machine"]
        self.filename = "default"
        self.pkg_name = "sdpp_explore"
        self.dict_array_assoc = None             #do checks for null
        #update attributes from config
        self.__dict__.update(config)



    def new_array(self, list_objects):
        """
        create new array based on object list

        parameters
        ----------
        list_objects: list
            ordred list of objects to be associated

        return
        ------
        array_assoc: dict
            dictionary containing *object_list* and *array*
        """

        length = len(list_objects)

        return {"array": np.zeros((length, length)),
                "list_objects": list_objects}
        
    def file_path_config(self, pkg):
        """
        returns the file path to the config folder of a package.

        parameters
        ----------
        pkg: string
            name of package to get filepath for

        return
        ------
        filepath: string
            the absolute path of desired package config folder
        """
        r = rospkg.RosPack()
        filepath = r.get_path(pkg)
        filepath = filepath + "/config/"        

        return filepath

    def save_to_config(self, pkg, filename):
        """
        saves object's *dict_array_assoc* to specified pakcage config filder

        parameters
        ----------
        pkg: string
            package to save configs under
        
        filename: string
            name of config file to be used
        """
        config_path = self.file_path_config(pkg) + "/config/"
        self._save_dict_array_yaml(config_path + "filename", self.dict_array_assoc)

    def load_from_config(self, pkg, filename):
        """
        loads desired *dict_array_assoc* array to the object attribute

        parameters
        ----------
        pkg: string
            package to save configs under
        
        filename: string
            name of config file to be used
        """
        config_path = self.file_path_config(pkg) + "/config/"
        self._load_dict_array_yaml(config_path + filename)

    def _yaml_extension(self, filename):
        """
        add ".yaml" extension to filename if needed

        parameters
        ----------
        filename: string
            input filename for checking

        return
        ------
        filename: string
            filename with .yaml extension

        """
        if filename.endswith(".yaml"):
            pass
        else:
            filename += ".yaml"
        return filename
        
    def _load_dict_array_yaml(self, filename):
        """
        
        parameters
        ----------
        filename: string
            filename to load obejct under

        return
        ------
        dict_array_assoc: dict
            dictionary containing *list_objects* and *array*

        """
        filename = self._yaml_extension(filename)

        fp = open(filename)
        data = yaml.load(fp)
        return data

    def _save_dict_array_yaml(self, filename, dict_array):
        """
        save the array as a yaml file for later use
    
        parameters
        ----------
        filename: string
            filename to save *dict_array_assoc* under

       """
        filename = self._yaml_extension(filename)

        with open(filename, "w") as outfile:
            yaml.dump(dict_array, outfile, default_flow_style=False)


    def _obj_index(self, obj):
        """
        finds index of object based on object name

        parameters
        ----------
        obj: srting
            name of object to be checked for index

        Return
        ------
        index: int
            interger index of object within list

        """

        return self.dict_array_assoc["list_objects"].index(obj)
       
    def add_assoc(self, obj1, obj2):
        """
        adds instance of 2 objects association

        parameters
        ----------
        obj1: string
            first objects

        obj2: string
            second object
        """
        index_obj1 = self._obj_index(obj1)
        index_obj2 = self._obj_index(obj2)

        self.dict_array_assoc["assoc_array"][index_obj1, index_obj2] += 1
        self.dict_array_assoc["assoc_array"][index_obj2, index_obj1] += 1

    def sub_assoc(self, obj1, obj2):
        """
        removes instance of 2 objects association

        parameters
        ----------
        obj1: string
            first objects

        obj2: string
            second object
        """
        index_obj1 = self._obj_index(obj1)
        index_obj2 = self._obj_index(obj2)

        self.dict_array_assoc["assoc_array"][index_obj1, index_obj2] -= 1
        self.dict_array_assoc["assoc_array"][index_obj2, index_obj1] -= 1


    def conditional_assoc(self, obj1, obj2):
        """
        the conditional probability that obj1 is associated with obj2
        P(obj1 | obj2)

        parameters
        ----------
        obj1: string
            first objects

        obj2: string
            second object

        return
        ------
        int
            value

        """
        index_obj1 = self._obj_index(obj1)
        index_obj2 = self._obj_index(obj2)

        assoc_1_2 = self.dict_array_assoc["assoc_array"][index_obj1, index_obj2]

        counts_2 = np.sum(self.dict_array_assoc["assoc_array"][index_obj2])

        return assoc_1_2/counts_2




    def display_graph(self):

        array = self.assoc_obj["assoc_array"]
        labels = self.assoc_obj["list_objects"]

        ax = sns.heatmap(array, annot=True)
        ax.set_xticklabels(labels)
        ax.set_yticklabels(labels)
        ax.tick_params(top=True, bottom=False,
                       labeltop=True, labelbottom=False)

        plt.setp(ax.get_xticklabels(), rotation=-30, ha="right",
                 rotation_mode="anchor")

        plt.show()



