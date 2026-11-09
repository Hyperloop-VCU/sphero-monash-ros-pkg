from setuptools import find_packages, setup

package_name = 'sphero-monash-ros-pkg'

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
    maintainer='pi1',
    maintainer_email='pi1@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
        'teleop_node = sphero_ros.nodes.teleop_node:main',
        'robot_node = sphero_ros.nodes.robot_wrapper_node:main'
        ],
    },
)
