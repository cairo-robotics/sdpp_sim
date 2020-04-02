#!/usr/bin/env python3

#%%%
from scipy.stats import multivariate_normal
import matplotlib.pyplot as plt
import yaml
import numpy as np

mean = [120, 160]
cov = [[200, 0], [0, 200]]

#%%
with open("path_mask_test.yaml") as file:

    path_mask = yaml.load(file, Loader=yaml.FullLoader)

path_mask = np.asarray(path_mask)

#%%


x, y = np.mgrid[0:200:1, 0:200:1]
pos = np.dstack((x, y))

var = multivariate_normal(mean, cov)

#%%
values = var.pdf(pos)
print(values)

# %%

fig = plt.figure()

ax = fig.add_subplot(1, 1, 1)

values = np.multiply(path_mask, values)
ax.contourf(x, y, values)

# %%
