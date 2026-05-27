from glob import glob

from setuptools import find_packages, setup

package_name = 'sailboat_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/dashboard',
            glob('dashboard/*')
        ),
        (
            'share/' + package_name + '/launch',
            glob('launch/*.launch.py')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='szekelyzalan',
    maintainer_email='szekelyzalan03@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'actuator_node = sailboat_control.actuator_node:main',
            'teleop_node = sailboat_control.sailboat_teleop_node:main',
            'apparent_wind_sensor = sailboat_control.apparent_wind_sensor:main',
            'buoy_detector = sailboat_control.buoy_detector:main',
            'course_manager = sailboat_control.course_manager:main',
            'sailboat_dashboard = sailboat_control.dashboard_node:main',
            'sailboat_autonomy = sailboat_control.sailboat_autonomy:main',
        ],
    },
)
