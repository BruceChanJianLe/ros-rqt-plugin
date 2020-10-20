#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ros_rqt_plugin'],
    package_dir={'': 'src'},
    # Only uncomment this line if you want it to be in the global bin
    # http://wiki.ros.org/rqt/Tutorials/Create%20your%20new%20rqt%20plugin#Install_.26_Run_your_plugin
    # scripts=['scripts/ros_rqt_plugin']
)

setup(**d)
