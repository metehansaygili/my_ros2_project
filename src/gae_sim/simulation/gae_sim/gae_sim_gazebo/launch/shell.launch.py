import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Configurations
    verbose = LaunchConfiguration('verbose')
    pub_tf = LaunchConfiguration('pub_tf')
    world_cfg = LaunchConfiguration('world')

    # Package paths
    pkg_share_vehicle = FindPackageShare(package='gae_vehicle_description').find('gae_vehicle_description')
    pkg_share_parkour = get_package_share_directory('parkour_pkg')
    pkg_share_gazebo = get_package_share_directory('gazebo_ros')
    
    urdf_file_vehicle = os.path.join(pkg_share_vehicle, 'urdf', 'gae_vehicle.urdf.xacro')
    
    # Debug prints
    print(f"[DEBUG] Vehicle URDF: {urdf_file_vehicle}")
    print(f"[DEBUG] URDF exists: {os.path.exists(urdf_file_vehicle)}")
    print(f"[DEBUG] Parkour models path: {os.path.join(pkg_share_parkour, 'models')}")

    # GAZEBO_MODEL_PATH setup
    models_path = os.path.join(pkg_share_parkour, 'models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        model_path = models_path + ':' + os.environ['GAZEBO_MODEL_PATH']
    else:
        model_path = models_path
    
    print(f"[DEBUG] Final GAZEBO_MODEL_PATH: {model_path}")

    # World file path
    pathWorldFile = PathJoinSubstitution([
        FindPackageShare('gae_sim_gazebo'), 
        'worlds', 
        world_cfg
    ])
    
    # Launch arguments
    verbose_arg = DeclareLaunchArgument(
        'verbose', 
        default_value='false',
        description='Enable verbose output'
    )
    
    pub_tf_arg = DeclareLaunchArgument(
        'pub_tf', 
        default_value='false',
        description='Publish TF transforms'
    )
    
    world_arg = DeclareLaunchArgument(
        'world', 
        default_value='track.world',
        description='World file name'
    )

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share_gazebo, 'launch', 'gazebo.launch.py')
        ), 
        launch_arguments={
            'world': pathWorldFile, 
            'verbose': verbose
        }.items()
    )

    # Vehicle state publisher
    vehicle_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='vehicle_state_publisher',
        namespace='gae_vehicle_ns',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'publish_frequency': 100.0,
            'robot_description': Command(['xacro ', urdf_file_vehicle, ' pub_tf:=', pub_tf])
        }]
    )

    # Vehicle spawn entity
    vehicle_spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='vehicle_spawn_entity',
        namespace='gae_vehicle_ns',
        arguments=[
            '-entity', 'kucukfurkan',
            '-topic', 'robot_description',
            '-x', '183.757110',
            '-y', '-287.031875',
            '-z', '0.399991',
            '-Y', '0.252795',
            '-timeout', '300'
        ],
        output='screen'
    )

    teleop_node = ExecuteProcess(
        cmd=[
            'python3',
            os.path.join(pkg_share_vehicle, 'src', 'teleop_node.py')
        ],
        output='screen'
    )

    # GPS heading node
    gps_heading_node = Node(
        package='gae_vehicle_description',
        executable='gps_heading_node',
        name='gps_heading_node',
        namespace='gae_vehicle_ns',  # Added namespace for consistency
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        # Environment variables
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        
        # Launch arguments
        verbose_arg,
        pub_tf_arg,
        world_arg,
        
        # Nodes and includes
        gazebo_launch,
        vehicle_state_publisher,
        vehicle_spawn_entity,
        teleop_node
        # gps_heading_node
    ])