#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    install_requires=['rospy'],
   packages=['rover_driver_base'],
   package_dir={'': 'src'}
    )

setup(**d)
