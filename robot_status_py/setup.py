from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_status_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kdeepak27',
    maintainer_email='deepakkmlknnn@gmail.com',
    description='Robot status demo (fake sensor, reset service, turtlesim action) in Python.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'status_publisher = robot_status_py.status_publisher:main',
            'drive_to_point_server = robot_status_py.drive_to_point_server:main',
        ],
    },
)
