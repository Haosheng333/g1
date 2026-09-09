#!/usr/bin/env python3
# Installs the g1_slam Python package (src/g1_slam) so nodes and tests can
# `import g1_slam.tf_chain`. Driven by catkin_python_setup() in CMakeLists.txt;
# never run this directly.
from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

setup(**generate_distutils_setup(
    packages=["g1_slam"],
    package_dir={"": "src"},
))
