#!/usr/bin/env python


import numpy as np



import matplotlib as mpl
from matplotlib import pyplot


GAMMA = .99
EPSILON = 1e-3

class numpyTest(np.ndarray):
    def __new__(subtype, shape, dtype=float, buffer=None, offset=0,
                strides=None, order=None, info=None):
        # Create the ndarray instance of our type, given the usual
        # ndarray input arguments.  This will call the standard
        # ndarray constructor, but return an object of our type.
        # It also triggers a call to InfoArray.__array_finalize__
        obj = super(numpyTest, subtype).__new__(subtype, shape, dtype,
                                                buffer, offset, strides,
                                                order)
        # set the new 'info' attribute to the value passed
        obj.info = info
        # Finally, we must return the newly created object:
        return obj




class GridWorld(object):

    def __init__(self):
        cells = np.zeros((5, 5))


        print cells




if __name__ == '__main__':

    print "test"

    test = GridWorld()

    print EPSILON

