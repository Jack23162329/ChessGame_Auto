from setuptools import setup

package_name = 'ur_chess_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='UR10 chess control package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = ur_chess_controller.controller:main',
            'example_game = ur_chess_controller.example_game:main',
            'multi_point_controller = ur_chess_controller.multi_point_controller:main',
        ],
    },
)