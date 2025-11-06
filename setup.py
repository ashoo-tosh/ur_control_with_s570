from setuptools import find_packages, setup

package_name = 's570_ros2_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/s570_ros2_bridge/launch', ['launch/s570_ur10_bridge.launch.py']),
        ('share/s570_ros2_bridge/launch', [
        'launch/s570_ur10_rviz.launch.py'
    ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ashootosh_raja',
    maintainer_email='ashutoshr980@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            's570_publisher = s570_ros2_bridge.s570_publisher:main',
            's570_to_ur_bridge = s570_ros2_bridge.s570_to_ur_bridge:main',
        ],
    },
)
