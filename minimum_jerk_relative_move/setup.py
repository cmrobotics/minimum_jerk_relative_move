from setuptools import setup
import os
from glob import glob 

package_name = 'minimum_jerk_relative_move'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py')),
        (os.path.join('share', package_name), glob('params/*_params.yaml')),
    ],
    zip_safe=True,
    maintainer='Erwin Lejeune',
    maintainer_email='erwin.lejeune@cm-robotics.com',
    description='Uses minimum jerk trajectories to plan relative movement',
    license='TODO: License declaration',
    tests_require=['pytest', 'unittest'],
    install_requires=["setuptools", "numpy==1.21.5"
                    ],
    entry_points={
        'console_scripts': [
            'minimum_jerk_relative_move = minimum_jerk_relative_move.main:main',
        ],
    },
)
