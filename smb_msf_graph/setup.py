from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    package_dir={"": "smb_msf_graph"},
)

setup(**setup_args)