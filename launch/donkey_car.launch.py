import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import platform

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='car_composite',
            executable='subscriber',
            output='screen',
            arguments = [
                '--ros-args', '--log-level', 'debug'
            ],
        ),
        launch_ros.actions.ComposableNodeContainer(
            name='ros_donkey_car',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            parameters = [
                {'use_mock_pca9685': platform.processor() != 'aarch64'}
            ],
            arguments = [
                '--ros-args', '--log-level', 'debug'
            ],
            composable_node_descriptions=[
                launch_ros.descriptions.ComposableNode(
                    package='servo_mgr',
                    plugin='servo_mgr::ServoManagerNode',
                    name='ServoManager',
                ),
            ],
            output='screen',
        ),
    ])
