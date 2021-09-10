from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup


d = generate_distutils_setup(
    packages=['osim_python', 'opensim_environment', 'opensim_environment_externalForce'],
    package_dir={'': 'src'}
)


#d = generate_distutils_setup(
#    packages=['osim_python', 'opensim_environment'],
#    package_dir={'': 'src'}
#)

setup(**d)
