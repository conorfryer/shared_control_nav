from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'shared_control_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fryerc',
    maintainer_email='fryerc@union.edu',
    description='Shared control navigation for TurtleBot3.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'shared_control = shared_control_nav.shared_control:main',
            'shared_teleop = shared_control_nav.shared_teleop:main',
            'status_monitor = shared_control_nav.status_monitor:main',
        ],
    },
)
