"""hardware:=real: the arm -> vendor driver table, argument validation, and the xacro for it (mock path).

No vendor driver is needed: the xacro only names a plugin, and the mock driver renders the same wiring
on mock_components. Nothing here talks to a robot.
"""
import importlib.util
import pathlib
import re

import pytest
import xacro

from mobile_manipulator_description import load_profile
from mobile_manipulator_description.hardware_drivers import (
    HardwareDriverError, check_installed, driver_for_arm, load_table, parse_params, plan_real_hardware,
    resolve)

PKG = pathlib.Path(__file__).resolve().parents[1]
TABLE = load_table()


def _have(package):
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory(package)
        return True
    except Exception:
        return False


def _render(**mappings):
    mappings.setdefault('arm', 'generic_6dof')
    mappings.setdefault('lift', 'none')
    return xacro.process_file(str(PKG / 'urdf' / 'mobile_manipulator.urdf.xacro'),
                              mappings={'hardware': 'real', **mappings}).toxml()


def _plugins(xml):
    return sorted(re.findall(r'<plugin>([^<]+)</plugin>', xml))


# ---- the table -------------------------------------------------------------------------------------

def test_every_driver_has_the_fields_the_xacro_and_launch_read():
    for key, d in TABLE.items():
        for field in ('arms', 'package', 'plugin', 'required_params', 'command_interfaces', 'ft_source'):
            assert field in d, f'{key}: {field}'
        assert '/' in d['plugin'], f'{key}: plugin must be package/Class'
        assert d['ft_source']['kind'] in ('driver_sensor', 'topic', 'injection', 'none')
        if d['ft_source']['kind'] != 'none':
            assert d['ft_source']['sensor']
        if d['ft_source']['kind'] == 'topic':
            assert d['ft_source']['topic'].startswith('/')


@pytest.mark.parametrize('arm,driver,package,plugin', [
    ('xarm6', 'ufactory', 'xarm_controller', 'xarm_control/UFRobotSystemHardware'),
    ('ur10e', 'ur', 'ur_robot_driver', 'ur_robot_driver/URPositionHardwareInterface'),
    ('ur5e', 'ur', 'ur_robot_driver', 'ur_robot_driver/URPositionHardwareInterface'),
    ('panda', 'franka', 'franka_hardware', 'franka_hardware/FrankaHardwareInterface'),
])
def test_arm_to_driver(arm, driver, package, plugin):
    key, entry = driver_for_arm(arm, TABLE)
    assert (key, entry['package'], entry['plugin']) == (driver, package, plugin)


def test_an_arm_is_served_by_one_driver_only():
    seen = {}
    for key, d in TABLE.items():
        for arm in d['arms']:
            assert arm not in seen, f'{arm} in {seen[arm]} and {key}'
            seen[arm] = key


def test_every_ur_arm_of_the_description_has_a_driver():
    for arm in ('ur3', 'ur5', 'ur10', 'ur10e', 'ur16e', 'ur20'):
        assert driver_for_arm(arm, TABLE)[0] == 'ur'


def test_arm_without_a_driver_names_the_known_arms():
    with pytest.raises(HardwareDriverError, match='generic_6dof.*xarm6|xarm6.*real_driver:=mock'):
        driver_for_arm('generic_6dof', TABLE)


def test_mock_is_never_chosen_by_arm():
    assert TABLE['mock']['arms'] == []


# ---- argument validation ---------------------------------------------------------------------------

def test_parse_params():
    assert parse_params('robot_ip=1.2.3.4, hw_ns=xarm') == {'robot_ip': '1.2.3.4', 'hw_ns': 'xarm'}
    assert parse_params('') == {}
    with pytest.raises(HardwareDriverError, match='key=value'):
        parse_params('robot_ip')


def test_missing_required_param_is_named_with_the_fix():
    with pytest.raises(HardwareDriverError, match=r'robot_ip.*real_params'):
        resolve('xarm6', {}, table=TABLE)
    with pytest.raises(HardwareDriverError, match='script_filename'):
        resolve('ur10e', {'robot_ip': '1.2.3.4'}, table=TABLE)


def test_defaults_are_overridden_by_params():
    r = resolve('xarm6', {'robot_ip': '10.0.0.2', 'hw_ns': 'arm1'}, table=TABLE)
    assert r['params']['hw_ns'] == 'arm1' and r['params']['robot_ip'] == '10.0.0.2'
    assert r['params']['velocity_control'] == 'true'


def test_unknown_driver_is_rejected():
    with pytest.raises(HardwareDriverError, match='unknown real_driver'):
        resolve('ur10e', {}, 'kuka', TABLE)


def test_missing_driver_package_fails_with_install_message():
    with pytest.raises(HardwareDriverError, match=r"'ur_robot_driver'.*not installed.*real_driver:=mock"):
        check_installed('ur', TABLE, installed=lambda p: False)
    check_installed('ur', TABLE, installed=lambda p: True)
    check_installed('mock', TABLE, installed=lambda p: False)   # no package needed


def test_plan_fails_before_launching_when_the_driver_is_missing():
    profile = load_profile('generic_6dof_telescopic')
    with pytest.raises(HardwareDriverError, match='lift'):
        plan_real_hardware(profile, 'ufactory', 'robot_ip=1.2.3.4', '', installed=lambda p: True, table=TABLE)
    with pytest.raises(HardwareDriverError, match='xarm_controller'):
        plan_real_hardware(profile, 'ufactory', 'robot_ip=1.2.3.4', 'my_lift/LiftSystem',
                           installed=lambda p: False, table=TABLE)
    m = plan_real_hardware(profile, 'ufactory', 'robot_ip=1.2.3.4', 'my_lift/LiftSystem',
                           installed=lambda p: True, table=TABLE)
    assert m['hardware'] == 'real' and m['real_driver'] == 'ufactory' and m['arm'] == 'generic_6dof'


def test_plan_with_mock_needs_nothing_installed():
    m = plan_real_hardware(load_profile('generic_6dof_telescopic'), 'mock', installed=lambda p: False)
    assert m['real_driver'] == 'mock'


def test_launch_shuts_down_with_a_message_instead_of_a_traceback():
    spec = importlib.util.spec_from_file_location('hardware_real_launch',
                                                  PKG / 'launch' / 'hardware_real.launch.py')
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    from launch import LaunchContext
    from launch.actions import EmitEvent, LogInfo
    ctx = LaunchContext()
    for k, v in dict(profile='generic_6dof_telescopic', real_driver='ufactory', real_params='',
                     real_lift_plugin='', controllers_file='', controller='', update_rate='100').items():
        ctx.launch_configurations[k] = v
    actions = launch._setup(ctx)
    assert [type(a) for a in actions] == [LogInfo, EmitEvent]
    assert 'robot_ip' in actions[1].event.reason


# ---- the xacro (mock path) -------------------------------------------------------------------------

def test_mock_renders_the_same_blocks_as_a_vendor_driver_would():
    xml = _render(real_driver='mock', lift='telescopic')
    assert _plugins(xml) == ['ft_bridge_hardware/FtBridgeHardwareInterface'] * 2 + [
        'mock_components/GenericSystem'] * 2
    for block in ('lift_system', 'arm_system', 'ft_sensor_bridge', 'ft_conditioned_bridge'):
        assert f'<ros2_control name="{block}"' in xml
    assert '<param name="injection_mode">true</param>' in xml
    assert xml.count('<sensor name="ft_sensor">') == 1 and xml.count('<sensor name="ft_sensor_tcp">') == 1
    assert 'mobile_manipulator_system' not in xml   # the simulated block is not emitted for real


def test_arm_joints_come_from_the_profile_with_velocity_command():
    profile = load_profile('generic_6dof_telescopic')
    xml = _render(real_driver='mock')
    for j in profile.arm_joints:
        assert f'<joint name="{j}">' in xml
    assert xml.count('<command_interface name="velocity"/>') == len(profile.arm_joints)


def test_no_lift_block_without_an_actuated_lift():
    assert 'lift_system' not in _render(real_driver='mock', lift='none')


def test_vendor_plugin_params_and_sensor_wiring():
    xml = _render(real_driver='ufactory', real_params='robot_ip=1.2.3.4', real_lift_plugin='my_lift/LiftSystem',
                  lift='telescopic')
    assert 'xarm_control/UFRobotSystemHardware' in xml and 'my_lift/LiftSystem' in xml
    assert 'mock_components' not in xml
    assert '<param name="robot_ip">1.2.3.4</param>' in xml and '<param name="hw_ns">xarm</param>' in xml
    assert '<param name="source_topic">/xarm/uf_ftsensor_ext_states</param>' in xml
    assert '<sensor name="uf_ft_sensor">' in xml and '<param name="injection_mode">' not in xml


def test_franka_has_no_force_sensor_interface():
    xml = _render(arm='panda', real_params='robot_ip=1.2.3.4', lift='none')
    assert 'franka_hardware/FrankaHardwareInterface' in xml
    assert '<sensor name="ft_sensor">' not in xml and '<sensor name="uf_ft_sensor">' not in xml
    assert '<sensor name="ft_sensor_tcp">' in xml    # the conditioned wrench is still exported


@pytest.mark.skipif(not _have('ur_description'), reason='ur_description is not installed')
def test_ur_arm_selects_the_ur_driver_and_its_sensor():
    xml = _render(arm='ur10e', real_params='robot_ip=1.2.3.4,script_filename=a,output_recipe_filename=b,'
                                           'input_recipe_filename=c')
    assert 'ur_robot_driver/URPositionHardwareInterface' in xml
    assert '<sensor name="tcp_fts_sensor">' in xml


@pytest.mark.parametrize('kwargs,message', [
    (dict(arm='generic_6dof'), 'no driver for arm generic_6dof'),
    (dict(arm='panda'), 'needs real_params for driver franka: robot_ip'),
    (dict(real_driver='nonexistent'), 'no driver for arm'),
    (dict(real_driver='ufactory', real_params='robot_ip=1', lift='telescopic'), 'real_lift_plugin'),
])
def test_xacro_fails_with_the_message(kwargs, message):
    with pytest.raises(Exception, match=message):
        _render(**kwargs)


def test_python_and_xacro_agree_on_the_plugin():
    for driver in TABLE:
        params = ','.join(f'{p}=x' for p in TABLE[driver]['required_params'])
        xml = _render(real_driver=driver, real_params=params, real_lift_plugin='my_lift/LiftSystem',
                      lift='telescopic')
        assert resolve('generic_6dof', parse_params(params), driver, TABLE)['plugin'] in xml
