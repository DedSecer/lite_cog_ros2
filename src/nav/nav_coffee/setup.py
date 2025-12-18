from setuptools import setup
import os
from glob import glob

package_name = 'nav_coffee'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dedsecer',
    maintainer_email='dedsecer@todo.todo',
    description='Coffee delivery waypoint navigation package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_navigator = nav_coffee.waypoint_navigator:main',
            'nav_supervisor = nav_coffee.nav_supervisor:main'
        ],
    },
)

