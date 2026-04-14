from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():

    rviz_config = os.path.expanduser(
        "~/ros2_ws/src/obstacle_avoidance/rviz/avoidance.rviz"
    )

    avoidance_node = Node(
        package="obstacle_avoidance",
        executable="avoidance_node",
        name="obstacle_avoidance",
        output="screen",
        remappings=[
            ("/camera/depth/image_raw", "/realsense/depth_image")
        ]
    )   

    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     arguments=["-d", rviz_config],
    #     output="screen"
    # )

    return LaunchDescription([
        avoidance_node,
        # rviz_node
    ])
