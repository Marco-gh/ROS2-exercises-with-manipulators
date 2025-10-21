from launch import LaunchDescription
import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    model_name = 'planar_arm' 
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('bridge_test_gz'), 'world', 'test.sdf'
        ]),
        description='World SDF con piano e joint di ancoraggio'
    )

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={'gz_args': LaunchConfiguration('world')}.items(),
    )
    
    return LaunchDescription([
        world_arg, 
        gz_launch,
    ])