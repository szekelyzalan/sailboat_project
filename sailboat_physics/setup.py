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
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'force_test_node = sailboat_physics.force_test_node:main',
            'boat_motion_node = sailboat_physics.boat_motion_node:main',
            'boat_physics_node = sailboat_physics.boat_physics_node:main',
        ],
    },
)
