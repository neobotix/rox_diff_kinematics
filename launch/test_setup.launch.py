import launch
import launch.actions
import launch.substitutions
import os
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions

def generate_launch_description():
    config = os.path.join(get_package_share_directory('rox_diff_kinematics'),'launch','test_setup.yaml')
    return launch.LaunchDescription([
        launch_ros.actions.Node(package='rox_diff_kinematics', executable='rox_diff_kinematics_node', output='screen',
            name='rox_diff_kinematics', parameters = [config])
    ])
