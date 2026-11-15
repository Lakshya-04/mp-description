"""Contract tests for the swappable description.

Renders the top-level xacro for every available arm and checks the contract that the rest of the
stack relies on: required frames exist, there are exactly seven actuated joints (the lift first,
then six arm joints in kinematic order), the lift limits and mimic stages are right, and the
planning chain runs from base_link to tcp_link. Runs from the source tree: it builds a temporary
ament prefix so `$(find mobile_manipulator_description)` resolves without installing anything.
"""
import os
import pathlib
import tempfile

import pytest
import xacro
import yaml
from urdf_parser_py.urdf import URDF

PKG = pathlib.Path(__file__).resolve().parents[1]
NAME = 'mobile_manipulator_description'


@pytest.fixture(scope='module', autouse=True)
def ament_prefix():
    """Make `$(find mobile_manipulator_description)` resolve to the source tree."""
    tmp = tempfile.TemporaryDirectory()
    root = pathlib.Path(tmp.name)
    (root / 'share').mkdir()
    (root / 'share' / NAME).symlink_to(PKG)
    idx = root / 'share' / 'ament_index' / 'resource_index' / 'packages'
    idx.mkdir(parents=True)
    (idx / NAME).touch()
    old = os.environ.get('AMENT_PREFIX_PATH', '')
    os.environ['AMENT_PREFIX_PATH'] = f'{root}:{old}' if old else str(root)
    yield
    os.environ['AMENT_PREFIX_PATH'] = old
    tmp.cleanup()


def _has_ur_description():
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory('ur_description')
        return True
    except Exception:  # noqa: BLE001
        return False


def _render(**mappings):
    mappings = {k: str(v) for k, v in mappings.items()}
    return xacro.process_file(str(PKG / 'urdf' / 'mobile_manipulator.urdf.xacro'),
                              mappings=mappings).toxml()


def _profile(name):
    return yaml.safe_load((PKG / 'config' / 'profiles' / f'{name}.yaml').read_text())


CASES = [('generic_6dof', 'generic_6dof_telescopic')]
if _has_ur_description():
    CASES.append(('ur10e', 'ur10e_telescopic'))


@pytest.mark.parametrize('arm,profile_name', CASES)
def test_contract(arm, profile_name):
    profile = _profile(profile_name)
    robot = URDF.from_xml_string(_render(arm=arm))

    links = {l.name for l in robot.links}
    for key in ('world_base', 'base', 'lift_base', 'lift_output', 'arm_base', 'flange',
                'tool_mount', 'tcp'):
        assert profile['frames'][key] in links, f'missing frame {key}'

    actuated = [j.name for j in robot.joints if j.type in ('revolute', 'prismatic', 'continuous')
                and j.mimic is None]
    expected = [profile['joints']['lift']] + profile['joints']['arm']
    assert sorted(actuated) == sorted(expected)

    chain_joints = robot.get_chain(profile['frames']['base'], profile['frames']['tcp'],
                                   links=False, fixed=False)
    chain_joints = [j for j in chain_joints if robot.joint_map[j].mimic is None]
    assert chain_joints == expected, 'planning chain order must be lift first, then the arm'


@pytest.mark.parametrize('stages', [1, 2, 3])
def test_lift_mimic_stages(stages):
    robot = URDF.from_xml_string(_render(arm='generic_6dof', lift_stages=stages, lift_stroke=1.2))
    lift = robot.joint_map['lift_joint']
    assert lift.type == 'prismatic'
    assert lift.limit.lower == 0.0 and lift.limit.upper == pytest.approx(1.2)
    assert list(lift.axis) == [0, 0, 1]
    mimics = [j for j in robot.joints if j.mimic is not None]
    assert len(mimics) == stages
    for j in mimics:
        assert j.mimic.joint == 'lift_joint'
        assert j.mimic.multiplier == pytest.approx(1.0 / stages)


def test_fixed_lift_gives_an_arm_only_robot():
    """lift_fixed_height pins the lift and its stages: six moving joints, no mimics."""
    robot = URDF.from_xml_string(_render(arm='generic_6dof', lift_stages=3, lift_fixed_height=0.5))
    moving = [j for j in robot.joints if j.type in ('revolute', 'prismatic', 'continuous')]
    assert len(moving) == 6
    assert all(j.mimic is None for j in robot.joints)
    lift = next(j for j in robot.joints if j.name == 'lift_joint')
    default = URDF.from_xml_string(_render(arm='generic_6dof'))
    base = next(j for j in default.joints if j.name == 'lift_joint')
    assert lift.type == 'fixed'
    assert lift.origin.xyz[2] == pytest.approx(base.origin.xyz[2] + 0.5)


def _ros2_control(arm, hardware):
    import xml.etree.ElementTree as ET
    root = ET.fromstring(_render(arm=arm, hardware=hardware))
    return root.findall('ros2_control')


@pytest.mark.parametrize('arm', ['generic_6dof'])
def test_mock_hardware_is_position_commanded_only(arm):
    blocks = _ros2_control(arm, 'mock')
    assert [b.get('type') for b in blocks] == ['system']
    for joint in blocks[0].findall('joint'):
        assert [c.get('name') for c in joint.findall('command_interface')] == ['position']


@pytest.mark.parametrize('arm', ['generic_6dof'])
def test_jtac_mock_adds_velocity_commands_and_an_ft_sensor(arm):
    system, sensor = _ros2_control(arm, 'jtac_mock')
    assert system.get('type') == 'system' and sensor.get('type') == 'sensor'
    joints = system.findall('joint')
    assert len(joints) == 7
    for joint in joints:
        assert [c.get('name') for c in joint.findall('command_interface')] == ['position', 'velocity']
    assert sensor.find('hardware/plugin').text == 'ft_bridge_hardware/FtBridgeHardwareInterface'
    assert sensor.find('sensor').get('name') == 'ft_sensor'
    assert [i.get('name') for i in sensor.find('sensor')] == [
        'force.x', 'force.y', 'force.z', 'torque.x', 'torque.y', 'torque.z']


@pytest.mark.parametrize('hardware', ['gazebo', 'mujoco', 'isaac', 'typo'])
def test_unknown_hardware_fails_loudly(hardware):
    """A backend that is not wired into the ros2_control block must not render a plugin-less one."""
    with pytest.raises(Exception) as err:
        _render(arm='generic_6dof', hardware=hardware)
    assert 'unknown hardware' in str(err.value)


def test_lift_velocity_and_stroke_are_parameters():
    robot = URDF.from_xml_string(_render(arm='generic_6dof', lift_stroke=0.9,
                                         lift_max_velocity=0.3))
    lift = robot.joint_map['lift_joint']
    assert lift.limit.upper == pytest.approx(0.9)
    assert lift.limit.velocity == pytest.approx(0.3)


def test_tcp_offset_is_a_parameter():
    robot = URDF.from_xml_string(_render(arm='generic_6dof', tcp_xyz='0 0 0.3'))
    assert robot.joint_map['tcp_joint'].origin.xyz == pytest.approx([0, 0, 0.3])


def test_unknown_arm_fails_loudly():
    """An arm that is not wired into arm.xacro must not silently render a robot without an arm."""
    with pytest.raises(Exception) as err:
        _render(arm='not_an_arm')
    assert 'unknown arm_type' in str(err.value)


def _generated_acm(arm):
    import importlib.util
    spec = importlib.util.spec_from_file_location('generate_acm', PKG / 'scripts' / 'generate_acm.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    robot = URDF.from_xml_string(_render(arm=arm))
    adjacent = module.adjacent_pairs(robot)
    lift = module.lift_pairs(robot) - adjacent
    return adjacent | lift


def _committed_acm(name):
    import re
    text = (PKG / 'srdf' / name).read_text()
    return {tuple(sorted(m)) for m in re.findall(r'link1="([^"]+)" link2="([^"]+)"', text)}


@pytest.mark.parametrize('arm,file', [('generic_6dof', 'acm_generic_6dof.xacro')] +
                         ([('ur10e', 'acm_ur.xacro')] if _has_ur_description() else []))
def test_committed_acm_is_up_to_date(arm, file):
    assert _generated_acm(arm) == _committed_acm(file), (
        f'srdf/{file} is stale: run scripts/generate_acm.py --arm {arm} --out srdf/{file}')


def test_srdf_includes_the_generated_matrix():
    srdf = xacro.process_file(str(PKG / 'srdf' / 'mobile_manipulator.srdf.xacro'),
                              mappings={'arm': 'generic_6dof'}).toxml()
    assert 'lift_stage_2_link' in srdf and 'Adjacent' in srdf
    assert 'group name="lift_arm"' in srdf
