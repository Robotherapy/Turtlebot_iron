import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'my_first_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tb3',
    maintainer_email='tb3@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'emptynode = my_first_package.myfirstnode:main',
        'paramnode = my_first_package.param_node:main',
        'tf_listener = my_first_package.tf_listener:main',
        'turtlebot_follower =my_first_package.follower:main'
    ],
    },
)
