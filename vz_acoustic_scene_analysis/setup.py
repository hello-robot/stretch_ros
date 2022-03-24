from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['vz_acoustic_scene_analysis'],
    package_dir={'': 'scripts'}
)
setup(**d)