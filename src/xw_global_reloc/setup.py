from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'xw_global_reloc'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xiaowei',
    maintainer_email='dev@xiaowei.local',
    description='Phase2A PoC global relocalization',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_contract_node = xw_global_reloc.sensor_contract_node:main',
            'keyframe_db_builder = xw_global_reloc.keyframe_db_builder_node:main',
            'global_reloc_poc = xw_global_reloc.global_reloc_poc_node:main',
            'offline_retrieval = xw_global_reloc.offline_retrieval:main',
        ],
    },
)
