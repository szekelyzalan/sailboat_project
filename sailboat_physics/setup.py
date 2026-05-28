from setuptools import find_packages, setup

package_name = 'sailboat_physics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='szekelyzalan',
    maintainer_email='szekelyzalan03@gmail.com',
    description=(
        'Simplified sailboat dynamics node for driving the Gazebo simulation '
        'from wind, baum, and rudder commands.'
    ),
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'game_physics_node = sailboat_physics.sailboat_game_physics:main',
        ],
    },
)
