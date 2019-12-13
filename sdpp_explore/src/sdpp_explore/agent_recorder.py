#!/usr/bin/env python

import rospy
import pickle
import random
import matplotlib.pyplot as plt
#from sklearn import mixture

#from spencer_tracking_msgs.msg import TrackedPersons
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError

import cv2

import numpy as np
import math



class AgentRecorder(object):
    def __init__(self):
        pass

    def _init_dict(self):
        pass

    def record_callback(self):
        pass
    
    def record_local(self):
        pass

    def record_file(self):
        pass
    


class AgentTrajFactory(object):
    """
    creates AgentTrajectory objects with callback methods that override a base class
    but what base class should that be?
    """

    def __init__(self):
        self._builder = {}

    def register_format(self, key, builder):
        self._builder[key] = builder

    def create(self, key, **kwags):
        builder = self._builder.get(key)
        if not builder:
            raise ValueError(key)
        return builder(**kwags)


class AgentTrajSpencer(object):
    def __init__(self, **config):
        print("initialize AgentTrajSpencer")
        print("currently not initialized")


    def traj_callback(self, msg):
        pass


class AgentTrajGazebo(object):
    def __init__(self, **cargs):
        
        rospy.loginfo("initialize AgentTrajGazebo")
        #default configs for trajectory publisher
        self. = "test"

        self.__dict__.update(cargs)
        #rospy.loginfo("AgentTrajGazebo initialized with:")

        print(self.__dict__)




    def traj_callback(self, msg):
        pass

    def init_world(self):
        pass


class PeopleRecorder(object):

    def __init__(self, dict_path = None,  picture_tag = False, sim=True ):

        self.track_dict, self.dict_path = self._init_dict(dict_path)
        self.pic_topic = None
        self.pic_freq = 1.0
        self.pic_cntr = 0

        if not sim:
            self.sub = rospy.Subscriber("/spencer/perception/tracked_persons", TrackedPersons, self.ppl_track_callback)

            if picture_tag:
                self.pic_topic = "/spencer/perception_internal/people_detection/rgbd_front_top/upper_body_detector/image/compressed"
                rospy.wait_for_message(self.pic_topic, CompressedImage)
                rospy.loginfo("fot imaage")

        else:
            self.sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
            self.track_num = random.randint(0, 200)
            self.sub = rospy.Subscriber("/track_num", Int32, self.track_num_update)

        rospy.on_shutdown(self.kill)


    def _init_dict(self, dict_path):
        """
        initialize the recording dictionary
        :param dict_path:   path of the dictionary to be recored to
        :return:            dictionary with tracking info, dictionary path
        """
        if dict_path is not None:
            try:
                file = open(dict_path, "r")
                track_dict = pickle.load(file)
                rospy.loginfo("loaded pickle from {}".format(dict_path))
            except:
                rospy.logwarn("created fresh dictionary save as: {}".format(dict_path))
                track_dict = {}


        else:
            dict_path = "new_tracks.pickle"
            rospy.logwarn("created fresh dictionary save as: {}".format(dict_path))
            track_dict = {}
        return track_dict, dict_path

    def ppl_track_callback(self, data):
        """
        track people
        :param data:
        :return:
        """
        for person in data.tracks:
            if person.track_id not in self.track_dict.keys():
                self.track_dict[person.track_id] = {}
                self.track_dict[person.track_id]["track"] = []
                self.track_dict[person.track_id]["images"] = []
                rospy.loginfo("new person id: {}".format(person.track_id))

            x = person.pose.pose.position.x
            y = person.pose.pose.position.y
            self.track_dict[person.track_id]["track"].append((x, y))

            '''add image of tracked person for object recognition'''
            if self.pic_cntr == int(30.0/self.pic_freq):
                self.pic_cntr = 0
                image = rospy.wait_for_message(self.pic_topic, CompressedImage)
                self.track_dict[person.track_id]["images"].append(image)
                rospy.loginfo("record pic")

            if self.pic_topic is not None:
                self.pic_cntr += 1

    def track_num_update(self, data):
        self.track_num = data

    def odom_callback(self, data):

        if self.track_num not in self.track_dict.keys():
            self.track_dict[self.track_num] = {}
            self.track_dict[self.track_num]["track"] = []
            self.track_dict[self.track_num]["images"] = []
            rospy.loginfo("new person id: {}".format(self.track_num))

        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        self.track_dict[self.track_num]["track"].append((x, y))


    def kill(self):
        self.sub.unregister()
        self.pickle_me()

    def pickle_me(self):
        with open(self.dict_path, 'wb') as handle:
            pickle.dump(self.track_dict, handle,  protocol=pickle.HIGHEST_PROTOCOL)


class PeopleViewer(object):

    def __init__(self, dict_path):

        self.track_dict, self.dict_path = self._init_dict(dict_path)
        self.bridge = CvBridge()

    def _init_dict(self, dict_path):
        """
        initialize the recording dictionary
        :param dict_path:   path of the dictionary to be recored to
        :return:            dictionary with tracking info, dictionary path
        """
        if dict_path is not None:
            file = open(dict_path, "r")
            track_dict = pickle.load(file)
            rospy.loginfo("loaded pickle from {}".format(dict_path))

        else:
            dict_path = "new_tracks.pickle"
            rospy.logwarn("created fresh dictionary save as: {}".format(dict_path))
            track_dict = {}
        return track_dict, dict_path

    def print_graph(self, n_tracks = 10):

        x_edge = np.linspace(-4, 10, num=50)
        y_edge = np.linspace(-4, 10, num=50)

        keys = self.track_dict.keys()
        print len(keys)

        if n_tracks > len(keys):
            rospy.logwarn("requested {} graphs only have {}".format(n_tracks, len(keys)))
            n_tracks = len(keys)

        fig = plt.figure(figsize=(15, 10))

        if n_tracks <= 5:
            subplot_width = n_tracks

        else:
            subplot_width = 5


        subplot_height = int(math.ceil(n_tracks/5.0))

        for i in range(0, n_tracks):

            ax = fig.add_subplot(subplot_height, subplot_width, i + 1)
            data = self._fetch_track(keys[i])
            x, y = zip(*data)
            test = np.array(list(zip(x, y)))

            length = np.size(test)
            ax.set_title("path:" + str(length))

            H, xedges, yedges = np.histogram2d(x, y, bins=(x_edge, y_edge))

            plt.imshow(H, interpolation='nearest', origin='low')

        plt.show()

    def _fetch_track(self, id):
        return self.track_dict[id]["track"]


    def carry_classify_all(self):

        keys = self.track_dict.keys()
        for i in range(0, len(keys)):
            images = self.track_dict[keys[i]]["images"]
            objects = self.visual_track_carry_classify(images)
            print objects
            self.track_dict[keys[i]]["objects"] = objects

    def visual_track_carry_classify(self, images):
        obj_list = []
        for i in range(0, len(images)):
            np_arr = np.fromstring(images[i].data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            cv2.imshow("image", image_np)
            cv2.waitKey(100)

            obj_list.append(input("what is the user carrying?\n"))
            cv2.waitKey(3)

        return self.unique(obj_list)

    def unique(self, list):

        unique_list = []
        for x in list:
            if x not in unique_list:
                unique_list.append(x)

        return unique_list

    def pickle_me(self):
        with open(self.dict_path, 'wb') as handle:
            pickle.dump(self.track_dict, handle,  protocol=pickle.HIGHEST_PROTOCOL)

    def find_goal_points(self):
        f = open("test_list.pickle", "r")
        list_data = pickle.load(f)

        list_data = np.array(list(zip(list_data[0], list_data[1])))

        print np.shape(list_data)

        print list_data[0]

        clf = mixture.GaussianMixture(n_components=4, covariance_type='full')

        clf.fit(list_data)

        print clf.means_


if __name__ == '__main__':

    #rospy.init_node("test")
    #test = PeopleViewer("test_dict.pickle")
    #test.print_graph(n_tracks=11)

    config_1 = {"test_config": "test2"}
    test = AgentTrajGazebo(**config_1)



