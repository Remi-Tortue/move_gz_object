from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'move_gz_object'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/srv/', ['ModifyObjectPose.srv']),
        # (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'std_msgs',
        'geometry_msgs',
        # Add other dependencies as needed
    ],
    zip_safe=True,
    maintainer='rporee',
    maintainer_email='remi.poree@laas.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_gazebo_object = move_gz_object.move_gazebo_object:main',
        ],
    },
)
