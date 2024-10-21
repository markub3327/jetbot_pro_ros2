from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            node_executable='joy_node',
            name='joy_node',
        ),
        Node(
            package='jetbot_pro_ros2',
            node_executable='teleop_joy',
            name='teleop_joy_node',
            parameters=[
                {"x_speed": 0.3},
                {"y_speed": 0.0},
                {"w_speed": 1.0},
            ],
        ),
    ])
