from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'xw_nav_session'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/xw_nav_session']),
        ('share/xw_nav_session', ['package.xml']),
]

# install public web assets
if package_name == 'xw_web':
    for dirpath, _, filenames in os.walk('public'):
        if not filenames:
            continue
        install_dir = os.path.join('share', package_name, dirpath)
        files = [os.path.join(dirpath, f) for f in filenames]
        data_files.append((install_dir, files))

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xiaowei',
    maintainer_email='dev@xiaowei.local',
    description='Navigation session skeleton',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'nav_session_node = xw_nav_session.nav_session_node:main',
        ],
    },
)
