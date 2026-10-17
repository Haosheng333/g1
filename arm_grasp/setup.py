from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['arm_grasp'],
    package_dir={'arm_grasp': 'src/arm_grasp'},
)

setup(**d)
