import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    ld = LaunchDescription()

    date_time = f"{datetime.now()}"[:19]
    date_format = date_time.replace(" ", "-").replace(":", "-")
    ros_bag_path = os.path.join(os.getcwd(), "ros_bags", f"{date_format}-recording")

    ros_bag = ExecuteProcess(
        cmd=["ros2", "bag", "record", "--all", "--storage", "mcap", "--output", ros_bag_path]
    )

    ld.add_action(ros_bag)

    return ld
