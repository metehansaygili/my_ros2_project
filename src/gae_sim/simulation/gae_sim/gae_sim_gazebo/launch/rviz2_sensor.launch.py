import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    pkg_share = get_package_share_directory('gae_sim_gazebo')
    
    # Path to RViz config file
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'sensor_config.rviz')
    
    print(f"[DEBUG] RViz config: {rviz_config_file}")
    print(f"[DEBUG] Config exists: {os.path.exists(rviz_config_file)}")
    
    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        rviz_node
    ])