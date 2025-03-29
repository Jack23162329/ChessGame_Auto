from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ur_robot_env'

models_paths = []
directories= glob('models/')+glob('models/*/')+glob('models/*/*/')
for directory in directories:
    models_paths.append((os.path.join('share',package_name,directory),glob(f'{directory}/*.*')))
    
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share',package_name,'launch'),glob('launch/*.launch.*')),
        (os.path.join('share',package_name,'config'),glob('config/*.yaml')),
        (os.path.join('share',package_name,'worlds'),glob('worlds/*')),


    ]+models_paths,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jack',
    maintainer_email='jack23162329@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "ur_move = ur_robot_env.example_game:main", 
            "multi_point_controller = ur_robot_env.multi_point_controller:main",
        ],
    },
)
