from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages = ['smb_msf'],
    package_name='smb_msf'
)

setup(**setup_args)