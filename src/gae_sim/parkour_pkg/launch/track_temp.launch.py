import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('parkour_pkg')
    
    # Modellerin yüklendiği install yolunu bulalım
    models_path = os.path.join(pkg_share, 'models')
    
    # Debug: Modeller dizininin var olup olmadığını kontrol et
    print(f"[DEBUG] Package share directory: {pkg_share}")
    print(f"[DEBUG] Models path: {models_path}")
    print(f"[DEBUG] Models directory exists: {os.path.exists(models_path)}")
    
    if os.path.exists(models_path):
        print(f"[DEBUG] Contents: {os.listdir(models_path)}")
        google_map_path = os.path.join(models_path, 'google_map')
        print(f"[DEBUG] google_map exists: {os.path.exists(google_map_path)}")
    
    # GAZEBO_MODEL_PATH'i güncelle
    if 'GAZEBO_MODEL_PATH' in os.environ:
        new_model_path = models_path + ':' + os.environ['GAZEBO_MODEL_PATH']
    else:
        new_model_path = models_path

    print(f"[DEBUG] Final GAZEBO_MODEL_PATH: {new_model_path}")

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=new_model_path
    )

    # World dosyamızın yolu
    world_file_path = os.path.join(pkg_share, 'worlds', 'track_temp.world')
    print(f"[DEBUG] World file path: {world_file_path}")
    print(f"[DEBUG] World file exists: {os.path.exists(world_file_path)}")
    
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    
    return LaunchDescription([
        set_gazebo_model_path,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world_file_path, 'verbose': 'true'}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')
            )
        ),
    ])