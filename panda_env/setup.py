from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'panda_env'

models_paths = []
directories= glob('models/')+glob('models/*/')+glob('models/*/*/')
for directory in directories:
    models_paths.append((os.path.join('share',package_name,directory),glob(f'{directory}/*.*')))
    
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #Add all xacro and URDF files that descripe the robot to the share directory
        (os.path.join('share',package_name,'description'),glob('description/*.xacro')),
        #Adding visual and collision mehses to the share directory
        (os.path.join('share',package_name,'meshes','visual'),glob('meshes/visual/*.dae')),
        (os.path.join('share',package_name,'meshes','collision'),glob('meshes/collision/*.stl')),
        #Adding Rviz2 configuration files
        (os.path.join('share',package_name,'rviz'),glob('rviz/*.rviz')),
        #Adding launch files
        (os.path.join('share',package_name,'launch'),glob('launch/*.launch.*')),
        (os.path.join('share',package_name,'config'),glob('config/*.yaml')),
        (os.path.join('share',package_name,'worlds'),glob('worlds/*')),
        (os.path.join('lib',package_name), glob('panda_env/RobotClass.py'))
        ]+models_paths,
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JackChen',
    maintainer_email='jack23162329@gmail.com',
    description='Panda manipulator chess world',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = panda_env.controller:main',
            'multi_point_controller = panda_env.multi_point_controller:main',
            'example_game = panda_env.example_game:main',
        ],
    },
)
