import os, time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Configuration for main vehicle
    verbose = LaunchConfiguration('verbose')
    pkg_share_vehicle = FindPackageShare(package='gae_vehicle_description').find('gae_vehicle_description')
    urdf_file_vehicle = os.path.join(pkg_share_vehicle, 'urdf', 'gae_vehicle.urdf.xacro')

    # Configuration for centilevered traffic light
    pkg_traffic_light = FindPackageShare(package='traffic_light_teknofest').find('traffic_light_teknofest')
    urdf_file_traffic_light = os.path.join(pkg_traffic_light, 'urdf', 'standalone_model.urdf.xacro')

    # Set GAZEBO_MODEL_PATH
    if 'GAZEBO_MODEL_PATH' in os.environ:
        model_path = os.environ['GAZEBO_MODEL_PATH'] + ':' + os.path.join(get_package_share_directory('gae_sim_gazebo'), 'models')
    else:
        model_path = os.path.join(get_package_share_directory('gae_sim_gazebo'), 'models')

    # Gazebo simulator launch
    gazebo_simulator = PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'))
    pathWorldFile = PathJoinSubstitution([FindPackageShare('gae_sim_gazebo'), 'worlds', LaunchConfiguration('world')])
    availableWorlds = os.listdir(os.path.join(get_package_share_directory('gae_sim_gazebo'), 'worlds'))

    # Declare launch arguments
    verbose_arg = DeclareLaunchArgument('verbose', default_value='false', description='Enable verbose mode for Gazebo')
    world_arg = DeclareLaunchArgument('world', default_value='teknofest.world', description=f'Choose an available world to launch in Gazebo: {availableWorlds}')

    # Include Gazebo launch
    gazeboLaunch = IncludeLaunchDescription(gazebo_simulator, launch_arguments={'world': pathWorldFile, 'verbose': verbose}.items())

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
            'robot_description': Command([f'xacro {urdf_file_vehicle} pub_tf:=true'])
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
            '-x', '58',
            '-y', '44',
            '-z', '0',
            '-Y', '3.14',
            '-timeout', '300'
        ],
        output='screen'
    )

    # Traffic Light Configuration
    def create_traffic_light_nodes(index, urdf_file, x, y, z, yaw):
        return [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name=f'traffic_light_state_publisher_{index}',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'robot_description': Command([f'xacro {urdf_file}']),
                }]
            ),
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name=f'traffic_light_spawn_entity_{index}',
                arguments=[
                    '-entity', f'traffic_light_{index}',
                    '-topic', 'robot_description',
                    '-x', str(x),
                    '-y', str(y),
                    '-z', str(z),
                    '-Y', str(yaw)
                ],
                output='screen'
            )
        ]

    traffic_light_nodes = []

    teleop_node = ExecuteProcess(
        cmd=[
            'python3',
            os.path.join(pkg_share_vehicle, 'src', 'teleop_node.py')
        ],
        output='screen'
    )

    # Create centilevered traffic lights
    traffic_light_nodes += create_traffic_light_nodes(1, urdf_file_traffic_light, -28.815199, 46.635651, 0, -0.012584)
    traffic_light_nodes += create_traffic_light_nodes(2, urdf_file_traffic_light, 17.155, 14.969, 0.02, 3.129767)
    traffic_light_nodes += create_traffic_light_nodes(3, urdf_file_traffic_light, -15.481564, -42.771363, 0.00, -3.139767)

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        verbose_arg,
        world_arg,
        gazeboLaunch,
        vehicle_state_publisher,
        teleop_node,
        vehicle_spawn_entity,
    ] + traffic_light_nodes)