from setuptools import find_packages, setup

package_name = 'tb3_teleop'

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
    maintainer='tb3',
    maintainer_email='oliver.tiessen@alumni.fh-aachen.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'joy_twist_node = tb3_teleop.joy_twist_node:main',
        'joy_twist_dead = tb3_teleop.joy_twist_dead:main',
        'joy_twist_dead_end = tb3_teleop.joy_twist_dead_end:main',
        ],
    },
)
