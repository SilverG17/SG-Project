from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    sdf_file = 'line_world.sdf'
    sdf_path = os.path.join(
        get_package_share_directory('python_pkg'),
        'urdf',  # rename 'urdf' to 'models' if more accurate
        sdf_file
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ign', 'gazebo', sdf_path],
            output='screen'
        )
    ])
