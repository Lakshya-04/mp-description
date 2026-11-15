"""Arm swap test: for each profile, render the URDF and SRDF and check that joint names, DOF,
group names, limits and the home pose agree with what the profile says (N = 6 and N = 7)."""
import importlib.util
import os
import pathlib
import re
import sys
import tempfile
import xml.etree.ElementTree as ET

import pytest
import xacro
from urdf_parser_py.urdf import URDF

PKG = pathlib.Path(__file__).resolve().parents[1]
NAME = 'mobile_manipulator_description'
sys.path.insert(0, str(PKG))

from mobile_manipulator_description import load_profile  # noqa: E402


@pytest.fixture(scope='module', autouse=True)
def ament_prefix():
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


PROFILES = ['generic_6dof_telescopic', 'panda_telescopic']
if _has_ur_description():
    PROFILES.append('ur10e_telescopic')


def _urdf(profile, **extra):
    mappings = dict(profile.xacro_mappings(), profile=profile.name, **extra)
    return xacro.process_file(str(PKG / 'urdf' / 'mobile_manipulator.urdf.xacro'),
                              mappings=mappings).toxml()


def _srdf(profile):
    return xacro.process_file(str(PKG / 'srdf' / 'mobile_manipulator.srdf.xacro'),
                              mappings={'arm': profile.arm, 'profile': profile.name}).toxml()


@pytest.mark.parametrize('name', PROFILES)
def test_urdf_matches_profile(name):
    p = load_profile(name)
    robot = URDF.from_xml_string(_urdf(p))
    chain = robot.get_chain(p.frames['base'], p.tcp_link, links=False, fixed=False)
    chain = [j for j in chain if robot.joint_map[j].mimic is None]
    assert chain == p.joints
    arm_joints = [robot.joint_map[j] for j in p.arm_joints]
    assert len(arm_joints) == p.arm_dof
    assert all(j.type == 'revolute' for j in arm_joints)
    assert robot.joint_map[p.lift_joint].type == 'prismatic'
    for j in arm_joints:
        assert j.limit.lower < j.limit.upper and j.limit.velocity > 0


@pytest.mark.parametrize('name', PROFILES)
def test_srdf_matches_profile(name):
    p = load_profile(name)
    root = ET.fromstring(_srdf(p))
    groups = {g.get('name') for g in root.findall('group')}
    assert groups == {p.arm_group, p.whole_body_group, p.tool_group}
    state = root.find('group_state')
    assert state.get('group') == p.whole_body_group
    values = {j.get('name'): float(j.get('value')) for j in state.findall('joint')}
    assert list(values) == p.joints
    assert values == pytest.approx(p.home)


@pytest.mark.parametrize('name', PROFILES)
@pytest.mark.parametrize('hardware', ['mock', 'jtac_mock'])
def test_ros2_control_has_one_interface_set_per_joint(name, hardware):
    p = load_profile(name)
    root = ET.fromstring(_urdf(p, hardware=hardware))
    system = root.findall('ros2_control')[0]
    joints = system.findall('joint')
    assert [j.get('name') for j in joints] == p.joints
    expected = ['position', 'velocity'] if hardware == 'jtac_mock' else ['position']
    for j in joints:
        assert [c.get('name') for c in j.findall('command_interface')] == expected


@pytest.mark.parametrize('name', PROFILES)
def test_home_is_within_limits(name):
    p = load_profile(name)
    robot = URDF.from_xml_string(_urdf(p))
    for joint, value in zip(p.joints, p.home_positions):
        limit = robot.joint_map[joint].limit
        assert limit.lower <= value <= limit.upper, joint


def test_srdf_without_profile_follows_the_arm():
    srdf = xacro.process_file(str(PKG / 'srdf' / 'mobile_manipulator.srdf.xacro'),
                              mappings={'arm': 'panda'}).toxml()
    assert srdf.count('<joint name="arm_joint') == 7


@pytest.mark.parametrize('lift,suffix', [('telescopic', ''), ('scissor', '_scissor'), ('none', '_none')])
def test_panda_acm_is_up_to_date(lift, suffix):
    spec = importlib.util.spec_from_file_location('generate_acm', PKG / 'scripts' / 'generate_acm.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    p = load_profile('panda_telescopic')
    robot = URDF.from_xml_string(_urdf(p, lift=lift))
    adjacent = module.adjacent_pairs(robot)
    generated = adjacent | (module.lift_pairs(robot, mounted=lift != 'scissor') - adjacent)
    text = (PKG / 'srdf' / f'acm_panda{suffix}.xacro').read_text()
    committed = {tuple(sorted(m)) for m in re.findall(r'link1="([^"]+)" link2="([^"]+)"', text)}
    assert generated == committed, (
        f'run scripts/generate_acm.py --arm panda --lift {lift} --out srdf/acm_panda{suffix}.xacro')


# Arm swap combined with other lifts and bases: the arm joints and the whole-body group follow
# the profile whatever the lift / base is.
COMBOS = [(prof, lift, base) for prof in PROFILES
          for lift, base in (('scissor', 'ranger_mini'), ('none', 'husky'), ('telescopic', 'mir100'))]


@pytest.mark.parametrize('name,lift,base', COMBOS)
def test_arm_with_other_lifts_and_bases(name, lift, base):
    p = load_profile(name)
    robot = URDF.from_xml_string(_urdf(p, lift=lift, base=base))
    chain = robot.get_chain(p.frames['base'], p.tcp_link, links=False, fixed=False)
    moving = [j for j in chain if robot.joint_map[j].mimic is None]
    expected = p.arm_joints if lift == 'none' else p.joints
    assert moving == expected
    srdf = ET.fromstring(xacro.process_file(
        str(PKG / 'srdf' / 'mobile_manipulator.srdf.xacro'),
        mappings={'arm': p.arm, 'profile': p.name, 'lift': lift}).toxml())
    state = srdf.find('group_state')
    assert [j.get('name') for j in state.findall('joint')] == expected
    assert state.get('group') == p.whole_body_group
    control = ET.fromstring(_urdf(p, lift=lift, base=base, hardware='mock')).find('ros2_control')
    assert [j.get('name') for j in control.findall('joint')] == expected


def _rpy(r, p, y):
    import numpy as np
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _link_poses_at_zero(root):
    """World (R, p) of every link with all joints at 0."""
    import numpy as np
    joints = {j.find('child').get('link'): j for j in root.findall('joint')}
    poses = {}

    def pose(link):
        if link not in poses:
            if link not in joints:
                poses[link] = (np.eye(3), np.zeros(3))
            else:
                j = joints[link]
                pr, pp = pose(j.find('parent').get('link'))
                o = j.find('origin')
                xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
                rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
                poses[link] = (pr @ _rpy(*rpy), pp + pr @ np.array(xyz))
        return poses[link]

    for link in list(joints):
        pose(link)
    return poses


def test_panda_collision_cylinders_lie_on_the_kinematic_chain():
    """Every joint-to-joint segment of the Panda chain is covered by a collision cylinder, and no
    cylinder sticks out of the chain (the arm is built from primitives, so a wrong axis or offset
    leaves a segment without geometry and adds a phantom bar elsewhere)."""
    import numpy as np
    root = ET.fromstring(_urdf(load_profile('panda_telescopic')))
    poses = _link_poses_at_zero(root)
    links = [f'arm_link{i}' for i in range(1, 8)]
    cylinders = []   # (end a, end b, radius) in world
    for name in links:
        rot, pos = poses[name]
        for c in root.find(f"link[@name='{name}']").findall('collision'):
            cyl = c.find('geometry/cylinder')
            o = c.find('origin')
            xyz = np.array([float(v) for v in o.get('xyz', '0 0 0').split()])
            r_c = _rpy(*[float(v) for v in o.get('rpy', '0 0 0').split()])
            half = float(cyl.get('length')) / 2 * (rot @ r_c @ np.array([0, 0, 1.0]))
            centre = pos + rot @ xyz
            cylinders.append((centre - half, centre + half, float(cyl.get('radius'))))

    def dist_to_segment(p, a, b):
        t = np.clip(np.dot(p - a, b - a) / max(np.dot(b - a, b - a), 1e-12), 0.0, 1.0)
        return np.linalg.norm(p - (a + t * (b - a)))

    chain = ['arm_base_link', *links, 'arm_flange_link']
    segments = [(poses[a][1], poses[b][1]) for a, b in zip(chain, chain[1:])]
    for a, b in segments:
        if np.linalg.norm(b - a) < 1e-6:
            continue
        for s in np.linspace(0.0, 1.0, 11):
            p = a + s * (b - a)
            assert any(dist_to_segment(p, ea, eb) <= r + 1e-6 for ea, eb, r in cylinders), \
                f'segment {a} -> {b} is not covered at {p}'
    for ea, eb, _ in cylinders:
        for end in (ea, eb):
            assert min(dist_to_segment(end, a, b) for a, b in segments) < 0.1, \
                f'cylinder end {end} is off the kinematic chain'
