from setuptools import find_packages, setup
import os               # <--- AGGIUNGI QUESTO
from glob import glob   # <--- AGGIUNGI QUESTO

package_name = 'real_franka_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 1. Install all the files in the 'launch' folder
        # Destination: share/real_franka_robot/launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        
        # 2. Install all the files in the 'calibration' folder
        # Destination: share/real_franka_robot/calibration
        (os.path.join('share', package_name, 'calibration'), glob('calibration/*.yaml')),
        
        # 3. Install all the files in the 'rviz' folder
        # Destination: share/real_franka_robot/calibration
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='well-i-am',
    maintainer_email='wn.notaro@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	        # command_name = package.file:function
            'main = real_franka_robot.main:main',
        ],
    },
)

