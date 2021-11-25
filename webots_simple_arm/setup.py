"""webots_ros2 package setup file."""

import os
import fnmatch
from glob import glob

from setuptools import setup

package_name = 'webots_simple_arm'
worlds = glob('worlds/*.wbt')
worlds.extend(glob("worlds/*.wbo"))
launchers = glob('launch/*.launch.py')

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name+ "/launch", launchers))
data_files.append(('share/' + package_name+ "/config", glob('config/*.*')))
data_files.append(('share/' + package_name + '/worlds', worlds))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name +'/protos', ['protos/panda.proto']))

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Florent Audonnet',
    maintainer_email='2330834a@student.gla.ac.uk',
    description='Run a pick and place task and a throw task on Webots.',
    license='BSD 3-Clause License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'panda_controller = webots_simple_arm.moveit2:main',
            'panda_trajectory = webots_simple_arm.panda_trajectory:main',
        ],
    }
)
