from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['clopema_robot'],
    package_dir={'': 'src_python'}
)

setup(**d)
