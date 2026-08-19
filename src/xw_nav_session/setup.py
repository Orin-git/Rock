from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'xw_nav_session'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/xw_nav_session']),
    ('share/xw_nav_session', ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*.xml')),
]

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xiaowei',
    maintainer_email='dev@xiaowei.local',
    description='Navigation session with Nav2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_session_node = xw_nav_session.nav_session_node:main',
        ],
    },
)
