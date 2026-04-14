from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    fake_obstacle_node = Node(
        package="obstacle_avoidance",
        executable="fake_obstacle_node",
        name="fake_depth_node",
        output="screen"
    )

    avoidance_node = Node(
        package="obstacle_avoidance",
        executable="avoidance_node",
        name="obstacle_avoidance",
        output="screen",
        remappings=[
            # Note: fake_obstacle_node already publishes to /realsense/depth_image
            # as per its source code.
        ]
    )

    return LaunchDescription([
        fake_obstacle_node,
        avoidance_node
    ])
