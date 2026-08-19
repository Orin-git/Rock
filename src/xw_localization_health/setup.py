from setuptools import find_packages, setup

package_name = 'xw_localization_health'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/xw_localization_health']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xiaowei',
    maintainer_email='dev@xiaowei.local',
    description='Gen2 localization health 0-3 and self-heal',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'localization_health_node = xw_localization_health.localization_health_node:main',
        ],
    },
)
