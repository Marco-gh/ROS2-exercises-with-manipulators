from launch import LaunchDescription
import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # --- 1. Parametri Iniziali ---
    initial_joint_positions = {
        'joint1': 3.15,
        'joint2': -1.57, # -90 gradi in radianti
        'joint3': 1.57,  # 90 gradi in radianti
        'joint4': 1.5,
        'joint5': 1.57,
        'joint6': -1.54,
    }

    # Il nome del modello nel tuo SDF è 'six_dof_arm'
    model_name = 'six_dof_arm' 
    
    # --- 2. Argomento Mondo ---
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('prova_gz'), 'world', 'test.sdf'
        ]),
        description='World SDF con piano e joint di ancoraggio'
    )

    # --- 3. Avvio di Gazebo Sim ---
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        # Passa il percorso del mondo come argomento per gz_args
        launch_arguments={'gz_args': LaunchConfiguration('world')}.items(),
    )
    # --- 4. Configurazione del ROS-Gazebo Bridge ---
    bridge_mappings = []
    
    for joint_name in initial_joint_positions.keys():
        ros_topic = f'/{joint_name}_pos_cmd'

        bridge_mappings.append(
            f'{ros_topic}@std_msgs/msg/Float64]gz.msgs.Double'
        )
    
    # Nodo per avviare il bridge
    ros_gz_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge'
        ] + bridge_mappings, 
        output='screen'
    )
    
    def create_initial_pose_actions(positions):
        timer_actions = []
        for joint_name, position in positions.items():
            ros_topic = f'/{joint_name}_pos_cmd'
            # Comando: ros2 topic pub -1 <topic> <msg_type> "<data>"
            pub_cmd = [
                'ros2', 'topic', 'pub',
                '-1', # Pubblica solo un messaggio e poi esci
                ros_topic,
                'std_msgs/msg/Float64',
                f"{{data: {position}}}" # Messaggio Float64 in formato YAML/JSON
            ]
            
            timer_actions.append(
                ExecuteProcess(
                    cmd=pub_cmd,
                    output='screen',
                    name=f'pub_{joint_name}_initial_pose'
                )
            )
        
        # Esegui tutte le pubblicazioni dopo un ritardo di 5 secondi
        return launch.actions.TimerAction(
            period=5.0, # Attende 5 secondi per il caricamento completo
            actions=timer_actions
        )

    set_initial_pose = create_initial_pose_actions(initial_joint_positions)
    # =================================================================
    
    return LaunchDescription([
        world_arg, 
        gz_launch, 
        ros_gz_bridge,
        set_initial_pose
    ])