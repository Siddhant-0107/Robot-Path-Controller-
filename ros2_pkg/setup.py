from setuptools import setup

package_name = 'ros2_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS2 integration package for Path Smoothing demo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_publisher = ros2_pkg.trajectory_publisher:main',
            'controller_node = ros2_pkg.controller_node:main',
        ],
    },
)
