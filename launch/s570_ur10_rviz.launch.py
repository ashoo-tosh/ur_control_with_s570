from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Robot state publisher for UR10
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open('/home/roshnee_patel/s570_ros2_ws/src/ur_description/urdf/ur.urdf.xacro').read()}]
        ),

        # RViz2 visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/home/roshnee_pat_el/s570_ros2_ws/src/ur_description/rviz/ur10.rviz']  # optional
        ),

        # S570 publisher
        Node(
            package='s570_ros2_bridge',
            executable='s570_publisher',
            name='s570_publisher',
            output='screen',
        ),

        # Bridge node: right arm → UR10
        Node(
            package='s570_ros2_bridge',
            executable='s570_to_ur_bridge',
            name='s570_to_ur_bridge',
            output='screen',
        )
    ])
