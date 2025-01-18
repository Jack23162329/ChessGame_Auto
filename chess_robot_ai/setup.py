from setuptools import setup
import os
from glob import glob

package_name = 'chess_robot_ai'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'requests'
    ],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Chess AI integration for chess_manipulator',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'chess_robot_node = chess_robot_ai.chess_robot_node:main',
            'chess_robot_node_ur = chess_robot_ai.chess_robot_node_UR:main',
        ],
    },
)