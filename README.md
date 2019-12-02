## RoboticsProject
This will implement obstacle avoidance with Baxter. 

RRT directory contains files needed to run rrt_7d.py, which computes a 7d path through obstacles defined as follows:
(q1_lower, q2_lower,..., qNlower, q1_upper, q2_upper,...,qN_upper)

The rest of the files were forked from SZanlongo https://github.com/motion-planning/rrt-algorithms and modified to create the 7d path planner. 

## Requirements
- [Python 3+](https://www.python.org/downloads/)
- [NumPy](http://www.numpy.org/)
- [Rtree](https://pypi.python.org/pypi/Rtree/)
- [Plotly](https://plot.ly/python/getting-started/) (only needed for plotting)

INSTALLATION NOTES -pip3 install rtree didn't work for me, but sudo apt install python3-rtree did. pip3 install worked for the other dependencies.

This is the error message that came with pip install:
Command "python setup.py egg_info" failed with error code 1 in /tmp/pip-build-541ysx/rtree/


MATLAB 2019a and Peter Corke Robotics Toolbox
