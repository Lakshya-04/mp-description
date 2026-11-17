"""Bring up the robot on real hardware (hardware:=real): vendor driver chosen from the profile's arm.

    ros2 launch mobile_manipulator_description hardware_real.launch.py profile:=ur10e_telescopic \
        real_params:="robot_ip=192.168.1.10,script_filename=...,output_recipe_filename=...,input_recipe_filename=..." \
        real_lift_plugin:=my_lift/LiftSystem
    ros2 launch mobile_manipulator_description hardware_real.launch.py profile:=generic_6dof_telescopic \
        real_driver:=mock                    # the same wiring on mock_components, no robot, no driver

What it does: validates the choice against config/hardware_drivers.yaml (arm -> driver, required
parameters, lift plugin, driver package installed) and, on any problem, prints one message that says what
to do and shuts down without starting a node. Otherwise it renders the description with hardware:=real
and starts ros2_control_node, robot_state_publisher and joint_state_broadcaster; with
`controllers_file` and `controller` it also starts that controller (spawned after the broadcaster).

The vendor driver packages are not part of this repository. Nothing here has been run on hardware:
docs/hardware_bringup.md (section 0a) is the procedure, docs/calibration.md the calibrations.
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from mobile_manipulator_description import load_profile
from mobile_manipulator_description.hardware_drivers import HardwareDriverError, plan_real_hardware


def _setup(context):
    def arg(name):
        return context.perform_substitution(LaunchConfiguration(name))

    try:
        mappings = plan_real_hardware(load_profile(arg('profile')), arg('real_driver'), arg('real_params'),
                                      arg('real_lift_plugin'))
    except HardwareDriverError as err:
        message = f'hardware:=real: {err}'
        return [LogInfo(msg=f'[hardware_real] ERROR {message}'), EmitEvent(event=Shutdown(reason=message))]

    xacro_file = os.path.join(get_package_share_directory('mobile_manipulator_description'),
                              'urdf', 'mobile_manipulator.urdf.xacro')
    description = xacro.process_file(xacro_file, mappings=mappings).toxml()
    robot_description = {'robot_description': ParameterValue(description, value_type=str)}
    controllers_file = arg('controllers_file')
    control_params = [robot_description, {'update_rate': int(arg('update_rate'))}]
    if controllers_file:
        control_params.append(controllers_file)

    control_node = Node(package='controller_manager', executable='ros2_control_node',
                        parameters=control_params, output='screen')
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[robot_description], output='screen')
    jsb = Node(package='controller_manager', executable='spawner',
               arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
               output='screen')
    nodes = [control_node, rsp, jsb]
    if arg('controller'):
        controller = Node(package='controller_manager', executable='spawner',
                          arguments=[arg('controller'), '--controller-manager', '/controller_manager'],
                          output='screen')
        nodes.append(RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[controller])))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='ur10e_telescopic',
                              description='profile name or YAML path (config/profiles); its arm picks the driver'),
        DeclareLaunchArgument('real_driver', default_value='auto',
                              description='driver key of config/hardware_drivers.yaml: auto (by arm), '
                                          'ufactory, ur, franka, mock'),
        DeclareLaunchArgument('real_params', default_value='',
                              description='the driver\'s required parameters, "key=value,key=value" '
                                          '(e.g. robot_ip=192.168.1.10)'),
        DeclareLaunchArgument('real_lift_plugin', default_value='',
                              description='ros2_control plugin of the lift (no arm vendor driver drives it)'),
        DeclareLaunchArgument('controllers_file', default_value='',
                              description='ros2_control controller parameters (yaml); empty = none loaded'),
        DeclareLaunchArgument('controller', default_value='',
                              description='controller to spawn from controllers_file, after the broadcaster'),
        DeclareLaunchArgument('update_rate', default_value='100', description='control rate, Hz'),
        OpaqueFunction(function=_setup),
    ])
