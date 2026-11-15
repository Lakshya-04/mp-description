"""`hardware:=<name>` selects exactly one ros2_control plugin, and the right one.

Guards the selector in mobile_manipulator.ros2_control.xacro, where each backend adds its own branch
(mock, jtac_mock, mujoco, gazebo): a mistake there loads two plugins, or the wrong one, for a backend.
"""
import pathlib
import re

import pytest
import xacro

PKG = pathlib.Path(__file__).resolve().parents[1]

EXPECTED = {
    'mock': ['mock_components/GenericSystem'],
    # jtac_mock adds the F/T bridge (a scripted wrench stands in for a sensor)
    'jtac_mock': ['ft_bridge_hardware/FtBridgeHardwareInterface', 'mock_components/GenericSystem'],
    # mujoco adds a second F/T bridge for the conditioned wrench (sensor ft_sensor_tcp)
    'mujoco': ['ft_bridge_hardware/FtBridgeHardwareInterface', 'sim_mujoco/MujocoSystemInterface'],
    # gazebo adds the F/T bridge for the conditioned wrench (sensor `ft_sensor_tcp`), under
    # gz_ros2_control's own plugin base class, the only one its resource manager loads
    'gazebo': ['ft_bridge_hardware/FtBridgeSimSystem', 'gz_ros2_control/GazeboSimSystem'],
}


def _installed(package):
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory(package)
        return True
    except Exception:
        return False


def _render(hardware):
    return xacro.process_file(str(PKG / 'urdf' / 'mobile_manipulator.urdf.xacro'),
                              mappings={'arm': 'ur10e', 'hardware': hardware}).toxml()


@pytest.mark.parametrize('hardware', sorted(EXPECTED))
def test_each_backend_loads_exactly_its_plugin(hardware):
    if hardware == 'gazebo' and not _installed('sim_gazebo'):
        pytest.skip('sim_gazebo is not installed')
    plugins = re.findall(r'<plugin>([^<]+)</plugin>', _render(hardware))
    assert sorted(plugins) == EXPECTED[hardware], plugins


def test_the_description_without_hardware_has_no_ros2_control_block():
    assert '<ros2_control' not in _render('none')


@pytest.mark.parametrize('hardware', ['mujoco', 'gazebo'])
def test_simulated_backends_export_the_raw_force_torque_sensor(hardware):
    if hardware == 'gazebo' and not _installed('sim_gazebo'):
        pytest.skip('sim_gazebo is not installed')
    xml = _render(hardware)
    # the ros2_control sensor is the one without a type attribute (Gazebo adds its own typed sensors)
    assert xml.count('<sensor name="ft_sensor">') == 1
    for interface in ('force.x', 'force.y', 'force.z', 'torque.x', 'torque.y', 'torque.z'):
        assert f'<state_interface name="{interface}"' in xml


def test_mujoco_exports_the_conditioned_sensor_next_to_the_raw_one():
    xml = _render('mujoco')
    assert xml.count('<sensor name="ft_sensor_tcp">') == 1
    assert '<param name="source_topic">/ft_conditioner/wrench_conditioned</param>' in xml
    # the raw name stays unique: the conditioned sensor must not reuse it
    assert xml.count('<sensor name="ft_sensor">') == 1


def test_gazebo_conditioned_sensor_does_not_clash_with_the_raw_one():
    if not _installed('sim_gazebo'):
        pytest.skip('sim_gazebo is not installed')
    xml = _render('gazebo')
    assert xml.count('<sensor name="ft_sensor_tcp">') == 1
    assert '/ft_conditioner/wrench_conditioned' in xml
