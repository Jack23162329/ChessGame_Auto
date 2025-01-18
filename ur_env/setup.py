from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ur_env'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Denis Stogl',
    maintainer_email='denis@stoglrobotics.de',
    description='Example and configuration files for Gazebo Classic simulation of UR manipulators.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'example_game = ur_env.example_game:main',
        ],
    },
)