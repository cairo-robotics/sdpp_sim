#!/usr/bin/env python

from logger import ROSErrorLogger, PrintErrorLogger
import pickle
import numpy as np

import matplotlib

import matplotlib.pyplot as plt
import seaborn as sns


class ObjectAssociation(object):

    def __init__(self, assoc_obj, file_path="default.pickle", ros_error=True, ):
        self.assoc_obj = assoc_obj
        self.file_path = file_path

        if ros_error:
            self.logger = ROSErrorLogger()

        else:
            self.logger = PrintErrorLogger()

    @classmethod
    def load_pickle_obj(cls, file_path="default.pickle", ros_error=True, ):
        file = open(file_path, "r")
        assoc_obj = pickle.load(file)
        return cls(assoc_obj, ros_error=ros_error, file_path=file_path)

    @classmethod
    def empty_inference_obj(cls, objects, ros_error=True):
        length = len(objects)
        assoc_obj = {"assoc_array": np.zeros((length, length)),
                     "object_list": objects}
        return cls(assoc_obj, ros_error=ros_error)

    def _obj_index(self, obj):
        try:
            return self.assoc_obj["object_list"].index(obj)

        except ValueError as e:
            message = "obj index lookup error: " + str(e)
            self.logger.log_error(message)
            return None

    def add_assoc_inst(self, frm, to):

        self.assoc_obj["assoc_array"]

        indexFrm = self._obj_index(frm)
        indexTo = self._obj_index(to)

        if indexFrm is None or indexTo is None:
            self.logger.log_error("unable to resolve obj assoc index bad")
            return None

        self.assoc_obj["assoc_array"][indexFrm, indexTo] += 1


    def pickle_me(self, file_path=None):

        if file_path == None:
            file_path = self.file_path

        with open(file_path, 'w+') as handle:
            pickle.dump(self.assoc_obj, handle,  protocol=pickle.HIGHEST_PROTOCOL)

    def print_array(self):

        self.logger.log_info(self.assoc_obj["assoc_array"])



    def display_graph(self):

        array = self.assoc_obj["assoc_array"]
        labels = self.assoc_obj["object_list"]

        ax = sns.heatmap(array, annot=True)
        ax.set_xticklabels(labels)
        ax.set_yticklabels(labels)
        ax.tick_params(top=True, bottom=False,
                       labeltop=True, labelbottom=False)

        plt.setp(ax.get_xticklabels(), rotation=-30, ha="right",
                 rotation_mode="anchor")

        plt.show()



