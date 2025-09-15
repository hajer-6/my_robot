from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['imu_message','imu_bno'],
    package_dir={'': 'src'}
)
setup(**d)
