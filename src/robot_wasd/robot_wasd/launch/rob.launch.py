from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, sys

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_wasd')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot_plan.urdf')
    rviz_cfg  = os.path.join(pkg_share, 'rviz', 'view.rviz')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # Robot_state_publisher + URDF + /joint_states → pubblica l’albero TF su /tf e /tf_static.
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

    if not sys.stdin.isatty():
        main_node = Node(
                package='robot_wasd',
                executable='main_node'
        )
        return LaunchDescription([rsp, static_tf, rviz, main_node])
    else:
        return LaunchDescription([rsp, static_tf, rviz])