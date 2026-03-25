from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')
    example_pkg_path = FindPackageShare('ping_pong_collision_test')
    gz_launch_path = PathJoinSubstitution(
        [ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py']
    )

    return LaunchDescription([
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            PathJoinSubstitution([example_pkg_path, 'models'])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': PathJoinSubstitution(
                    [example_pkg_path, 'worlds', 'ping_pong_world.sdf']
                ),
                'on_exit_shutdown': 'True'
            }.items(),
        ),
        Node(
            package='ping_pong_collision_test',
            executable='ping_pong_pub_fun',
        ),
        Node(
            package='ros_gz_bridge',
            # 4 topic bridges: 3 for the command topics, 1 for the joint state topic
            # tutti dentro un solo nodo per evitare problemi di sincronizzazione
            executable='parameter_bridge',
            arguments=[
                # ROS -> Gazebo
                # /TOPIC@ROS_MSG@GZ_MSG
                '/arm/joint0/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                '/arm/joint1/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                '/arm/joint2/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                '/arm/joint3/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                # Gazebo -> ROS
                '/arm/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            ]

        )
    ])