#!/usr/bin/env python

import rospy

class ROSErrorLogger(object):

    def __init__(self):
        rospy.loginfo("using ros error logger")

    @staticmethod
    def test_print():
        rospy.loginfo("ros error logger")
        print("ros error logger")

    @staticmethod
    def log_debug(message):
        rospy.logdebug(message)

    @staticmethod
    def log_info(message):
        rospy.loginfo(message)

    @staticmethod
    def log_warn(message):
        rospy.logwarn(message)

    @staticmethod
    def log_error(message):
        rospy.logerr(message)

    @staticmethod
    def log_fatal(message):
        rospy.logfatal(message)


class PrintErrorLogger(object):

    def __init__(self):
        print("using stdout error logger")

    @staticmethod
    def test_print():
        print("print error logger")

    @staticmethod
    def log_debug(message):
        print(message)

    @staticmethod
    def log_info(message):
        print(message)

    @staticmethod
    def log_warn(message):
        print(message)

    @staticmethod
    def log_error(message):
        print(message)

    @staticmethod
    def log_fatal(message):
        print(message)



