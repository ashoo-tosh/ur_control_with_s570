from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # UR10 visualization (from ur_description)
        Node(
            package='ur_description',
            executable='view_ur.launch.py',
            name='ur10_view',
            output='screen',
            arguments=['ur_type:=ur10'],
        ),

        # S570 joint publisher node
        Node(
            package='s570_ros2_bridge',
            executable='s570_publisher',
            name='s570_publisher',
            output='screen',
        ),

        # Bridge node: right arm → UR10 joints
        Node(
            package='s570_ros2_bridge',
            executable='s570_to_ur_bridge',
            name='s570_to_ur_bridge',
            output='screen',
        ),
    ])
