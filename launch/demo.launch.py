from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_path = get_package_share_directory('drone_predictor_clean')
    rviz_config = os.path.join(pkg_path, 'rviz', 'demo.rviz')

    return LaunchDescription([

        # Pose Publisher
        Node(
            package='drone_predictor_clean',
            executable='pose_publisher',
            name='pose_publisher',
            output='screen'
        ),

        # ML Predictor
        Node(
            package='drone_predictor_clean',
            executable='predictor_node',
            name='ml_predictor',
            output='screen'
        ),

        # RViz
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config],
            output='screen'
        )
    ])
