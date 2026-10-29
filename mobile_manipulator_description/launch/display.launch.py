"""Show the description in RViz with sliders for every joint.

  ros2 launch mobile_manipulator_description display.launch.py arm:=ur10e
  ros2 launch mobile_manipulator_description display.launch.py arm:=generic_6dof
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
import xacro


def _launch(context):
    pkg = get_package_share_directory('mobile_manipulator_description')
    args = {
        k: context.perform_substitution(v)
        for k, v in (('arm', LaunchConfiguration('arm')), ('lift_stroke', LaunchConfiguration('lift_stroke')))
    }
    doc = xacro.process_file(os.path.join(pkg, 'urdf', 'mobile_manipulator.urdf.xacro'), mappings=args)
    robot_description = doc.toxml()
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', os.path.join(pkg, 'rviz', 'display.rviz')]),
    ]


from launch.substitutions import LaunchConfiguration  # noqa: E402


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arm', default_value='ur10e'),
        DeclareLaunchArgument('lift_stroke', default_value='1.5'),
        OpaqueFunction(function=_launch),
    ])
