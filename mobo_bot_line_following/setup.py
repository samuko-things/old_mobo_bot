from setuptools import setup

import os
from glob import glob

package_name = 'mobo_bot_line_following'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samuko95',
    maintainer_email='samuel.c.agba@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'executable_name = package_name.python_file_name:main'
            'read_camera = mobo_bot_line_following.read_camera:main',
            'follow_line = mobo_bot_line_following.follow_line:main',
            'command_robot = mobo_bot_line_following.command_robot:main',
        ],
    },
)
