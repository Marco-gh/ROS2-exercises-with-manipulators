from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_square')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    rviz_cfg  = os.path.join(pkg_share, 'rviz', 'view.rviz')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # jsp = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     parameters=[{'robot_description': robot_description}],
    #     output='screen'
    # )

    talker = Node(
        package='robot_square',
        executable='talker'
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','1','world','base_link'],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen'
    )

    return LaunchDescription([rsp, static_tf, rviz, talker])
