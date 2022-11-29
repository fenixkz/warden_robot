from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# This file runs with `catkin_make`, do not run it manually!
setup_args = generate_distutils_setup(
    packages=['warden_robot'],
    package_dir={'': 'utils'}
)

setup(**setup_args)
