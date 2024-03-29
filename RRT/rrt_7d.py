# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.

#Originally forked from SZanlongo https://github.com/motion-planning/rrt-algorithms
#Modified by Curtis Johnson, Dec 2019

import numpy as np
import matlab.engine
from rrt import RRT
from search_space import SearchSpace
#from plotting import Plot

sim = matlab.engine.start_matlab()
X_dimensions = np.array([(0, 1), (0, 1), (0, 1), (0, 1), (0, 1), (0, 1), (0, 1)])  # dimensions of Search Space, this should be joint limits and n dimensions.

# obstacles (q1 lower, q2 lower, ..., qn lower, q1 upper, q2 upper,..., qn upper), each obstacle should have 2n coordinates
# Obstacles = np.array([
# 	(20, 20, 20, 20, 20, 20, 20, 40, 40, 40, 40, 40, 40, 40),
# 	(10, 10, 10, 10, 10, 10, 10, 18, 18, 18, 18, 18, 18, 18)
# 	])

x_init = (0, 0, 0, 0, 0, 0, 0)  # starting location
x_goal = (0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5)  # goal location

Q = np.array([(.05, .05, .05, .05, .05, .05, .05)])  # length of tree edges
r = .01  # length of smallest edge to check for intersection with obstacles
max_samples = 100  # max number of samples to take before timing out
prc = 0.1  # probability of checking for a connection to goal

# create search space
#X = SearchSpace(X_dimensions, Obstacles)
X = SearchSpace(X_dimensions) #can set obstacles to none.

# create rrt_search
rrt = RRT(X, Q, x_init, x_goal, max_samples, r, prc)
print('rrt built. Searching for path...')
path = rrt.rrt_search()

#TODO: figure out plotting with plotly
# plot
#plot = Plot("rrt_2d")
#plot.plot_tree(X, rrt.trees)
#if path is not None:
#    plot.plot_path(X, path)
#plot.plot_obstacles(X, Obstacles)
#plot.plot_start(X, x_init)
#plot.plot_goal(X, x_goal)
#plot.draw(auto_open=True)

if path is not None:
    path = np.asarray(path)
    path.tolist()
    q = []
    for i in range(len(path)):
        for j in range(len(path[i])):
            val = path[i][j]
            q.append(val.item())


    print("sending path to matlab...")
    sim.baxter_simulator(q,nargout=0)
