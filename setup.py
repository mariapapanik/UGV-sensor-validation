from glob import glob

from setuptools import setup

package_name = 'husky_sensor_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marpa',
    maintainer_email='mariap@gmail.com',
    description='Husky sensor verification sim for Gazebo Fortress',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sensor_check = husky_sensor_sim.sensor_check:main',
            'validation_run = husky_sensor_sim.validation_run:main',
        ],
    },
)
