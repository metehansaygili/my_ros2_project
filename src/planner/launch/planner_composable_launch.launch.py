import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer

def generate_launch_description():
    # Get the directory containing the default.yaml file
    config_dir = os.path.join(get_package_share_directory('planner'), 'config', 'config.yaml')
    # Define the composable nodes
    global_planner = ComposableNode(
        package='planner',
        plugin='itusct::GlobalPlanner',
        name='global_planner_exe',
        parameters=[config_dir],
    )

    mission_planner = ComposableNode(
        package='planner',
        plugin='itusct::MissionPlanner',
        name='mission_planner_exe',
        parameters=[config_dir],
    )

    occupancy_grid = ComposableNode(
        package='planner',
        plugin='itusct::OccupancyGrid',
        name='occupancy_grid_exe',
        parameters=[config_dir],
    )

    trajectory_planner = ComposableNode(
        package='planner',
        plugin='itusct::TrajectoryPlanner',
        name='trajectory_planner_exe',
        parameters=[config_dir],
    )

    controller = ComposableNode(
        package='planner',
        plugin='itusct::Controller',
        name='controller_exe',
        parameters=[config_dir],
    )

    state_machine = ComposableNode(
        package='planner',
        plugin='itusct::StateMachine',
        name='state_machine_exe',
        parameters=[config_dir],
    )

    # Define the container for composable nodes
    container = ComposableNodeContainer(
        name='planner_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            global_planner,
            mission_planner,
            occupancy_grid,
            trajectory_planner,
            state_machine,
            controller,
        ],
        output='screen',
    )

    return LaunchDescription([
        # Increase service timeout to prevent timeout warnings when loading nodes
        SetEnvironmentVariable('RMW_TIMEOUT', '30000'),  # 30 seconds in milliseconds
        container
    ])