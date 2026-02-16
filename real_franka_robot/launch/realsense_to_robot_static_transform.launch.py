""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-TO-HAND: fr3_link0 -> camera_color_optical_frame """
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="log",
            arguments=[
                "--frame-id",
                "fr3_link0",
                "--child-frame-id",
                "camera_link",
                "--x",
                "0.725901",
                "--y",
                "0.642500",
                "--z",
                "0.304688",
                "--qx",
                "-0.175860",
                "--qy",
                "-0.085133",
                "--qz",
                "0.897853",
                "--qw",
                "-0.394571",
            ],
        ),
    ]
    return LaunchDescription(nodes)
