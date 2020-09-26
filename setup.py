#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup


setup(**generate_distutils_setup(
    packages=[
        'aioros_tf2',
    ],
    package_dir={'': 'src/'}
))
