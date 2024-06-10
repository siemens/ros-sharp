from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'unity_simulation_scene2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mehmet Emre Cakal',
    maintainer_email='emre.cakal@siemens.com',
    description='Unity Simulation Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mouse_to_joy2 = unity_simulation_scene2.mouse_to_joy2:main'
        ],
    },
)
