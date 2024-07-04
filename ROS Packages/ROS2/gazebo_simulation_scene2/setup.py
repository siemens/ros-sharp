from setuptools import find_packages, setup
from glob import glob
from os import path

package_name = 'gazebo_simulation_scene2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mehmet Emre Cakal',
    maintainer_email='emre.cakal@siemens.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_to_twist2 = gazebo_simulation_scene2.joy_to_twist2:main',
        ],
    },
)
