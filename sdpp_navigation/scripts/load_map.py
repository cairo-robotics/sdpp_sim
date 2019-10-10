#!/usr/bin/env python

from sdpp_navigation.grid_world import GridWorld, ValueIterationAlgo
import matplotlib.pyplot as plt
from pprint import pprint, pformat
from gym.envs.toy_text import discrete


from PIL import Image
import numpy as np



def done():
    global world, prev_world
    # If NaNs are in the inputs, we may get a warning. Ignore it and it won't affect the epsilon comparison because np.nan > epsilon is False
    try:
        diff = np.abs(world.as_array() - prev_world)
    except RuntimeWarning:
        pass
    return not (diff > epsilon).any()

epsilon = 0.000001

#action array
nA = [(-1, 0, .1), (0, 1, .1), (1, 0, .8)]

def value_iteration(world, world_reward,gamma = .99):

    v = np.zeros(world.shape)
    print world
    print world.shape
    print v.shape

    max_iterations = 20
    eps = 1e-20

    for i in range(max_iterations):
        prev_v = np.copy(v)
        for i in range(0, world.shape[0]):
            for j in range(0, world.shape[1]):
                #value in each action
                q_sa = [0] * np.zeros(len(nA))
                for index, A in enumerate(nA):
                    row_A = i + A[0]
                    col_A = j + A[1]
                    try:
                        #if
                        q_sa[index] = world_reward[i, j] + gamma * (A[2]*prev_v[row_A, col_A])

                    except IndexError as error:
                        q_sa[index] = -1

                v[i, j] = max(q_sa)

    plt.imshow(v, cmap="plasma")
    plt.colorbar()
    plt.show()




def plot_world(world_arr):
    plt.imshow(world_arr, cmap="plasma")
    plt.colorbar()
    plt.show()


if __name__ == '__main__':

    gamma = .99

    im_frame = Image.open('block_room.png')
    np_frame = np.array(im_frame)
    np_frame = np_frame[: , :, 0]
    print np_frame.shape


    #print np_frame
    #world_input = np.array([[0,0, 255, 100], [0,0 ,0,  0], [0,0,0,0]])
    #world_reward = np.array([[0,0,0,100], [0,0 ,0, 0], [0,0,0,0]])

    world = GridWorld(np_frame, 200, 200)
    for i in range(115, 125):
        for j in range(35, 45):
            world.cells[i][j].is_terminal = True
            world.cells[i][j].value = 225
            world.cells[i][j].reward = 225

    world_arr = world.as_array()

    plot_world(world_arr)
    algo = ValueIterationAlgo( gamma, world)

    iter_cnt = 0
    while(True):
        print 'iteration {}'.format(iter_cnt).center(72, '-')

        for cell in world:
            algo.update(cell)

        if iter_cnt >= 50:
            break

        iter_cnt += 1


    world_arr = world.as_array()
    plot_world(world_arr)


