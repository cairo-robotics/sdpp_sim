#!/usr/bin/env python3

#%%

import yaml
import matplotlib.pyplot as plt

#%%
with open("vi_maps.yaml") as file:

    vi_maps = yaml.load(file, Loader=yaml.FullLoader)

# %%

length = len(vi_maps)
ax_list =  []
fig = plt.figure()

ax1 =fig.add_subplot(1, 1, 1)

ax1.imshow(vi_maps[2]["grid_world_array"])



# %%
