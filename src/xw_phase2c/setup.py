from setuptools import find_packages, setup

package_name = 'xw_phase2c'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/xw_phase2c']),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/launch',
            [
                'launch/phase2c_c1.launch.py',
                'launch/phase2c_c2_boot.launch.py',
                'launch/phase2c_c3_lost.launch.py',
            ],
        ),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='xiaowei',
    maintainer_email='dev@xiaowei.local',
    description='Phase2C helpers + BOOT cascade + LOST recovery',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'last_good_pose_writer = xw_phase2c.last_good_pose_writer_node:main',
            'charger_prior_node = xw_phase2c.charger_prior_node:main',
            'boot_localizer = xw_phase2c.boot_localizer_node:main',
            'lost_recovery = xw_phase2c.lost_recovery_node:main',
        ],
    },
)
