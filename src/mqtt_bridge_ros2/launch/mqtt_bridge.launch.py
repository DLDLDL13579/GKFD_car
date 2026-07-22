from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package="mqtt_bridge_ros2",
            executable="mqtt_bridge_node",
            output="screen",
            parameters=[{
                "mqtt_broker": "192.168.1.67",
                "mqtt_port": 1883,
                "mqtt_client_id": "jetson_robot",
                "topic_prefix": "robot_0"
            }],
        ),
    ])