"""Tests for the swappable bases (`base:=<name>`).

Every base macro must render, provide the three contract links, form a valid kinematic tree with
no actuated joint in the base, and agree with the `base:` section of its profile (footprint
contains the body, mount height, wheel radius and track). The whole robot must render with each
base on the UR10e + telescopic lift. Runs from the source tree with a temporary ament prefix.
"""
import importlib.util
import os
import pathlib
import tempfile

import pytest
import xacro
import yaml
from urdf_parser_py.urdf import URDF

PKG = pathlib.Path(__file__).resolve().parents[1]
NAME = 'mobile_manipulator_description'
PROFILES = PKG / 'config' / 'profiles'
BASES = ['generic_base', 'ranger_mini', 'scout_mini', 'scout', 'husky', 'mpo_500', 'mir100',
         'turtlebot3_burger', 'turtlebot3_waffle']
DRIVE_TYPES = {'omni', 'diff', 'skid', '4ws'}
TOL = 1e-6


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


needs_ur = pytest.mark.skipif(not _has_ur_description(), reason='ur_description not installed')


def _render(**mappings):
    mappings = {k: str(v) for k, v in mappings.items()}
    return xacro.process_file(str(PKG / 'urdf' / 'mobile_manipulator.urdf.xacro'),
                              mappings=mappings).toxml()


def _profile_files():
    return sorted(p for p in PROFILES.glob('*.yaml') if 'base' in yaml.safe_load(p.read_text()))


def _profile_args(profile):
    # robot.xacro_args plus the lift variant from the profile's `lift:` section (scripts/profile_xacro_args.py)
    spec = importlib.util.spec_from_file_location('profile_xacro_args', PKG / 'scripts' / 'profile_xacro_args.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.xacro_args(profile)


def _load(path):
    return yaml.safe_load(path.read_text())


def _positions(robot):
    """Translation of every link in base_footprint (the base joints carry no rotation)."""
    pos = {'base_footprint': (0.0, 0.0, 0.0)}
    children = {}
    for j in robot.joints:
        children.setdefault(j.parent, []).append(j)
    stack = ['base_footprint']
    while stack:
        parent = stack.pop()
        for j in children.get(parent, []):
            rpy = j.origin.rpy if j.origin else [0, 0, 0]
            assert all(abs(a) < TOL for a in rpy), f'{j.name}: base joints must not rotate'
            xyz = j.origin.xyz if j.origin else [0, 0, 0]
            pos[j.child] = tuple(p + o for p, o in zip(pos[parent], xyz))
            if j.child != 'lift_base_link':  # the lift and arm are not part of the base
                stack.append(j.child)
    return pos


def _base_links(robot):
    """Names of the links that belong to the base: everything above lift_base_link's subtree."""
    lift_sub = set()
    stack = ['lift_base_link']
    kids = {}
    for j in robot.joints:
        kids.setdefault(j.parent, []).append(j.child)
    while stack:
        n = stack.pop()
        lift_sub.add(n)
        stack.extend(kids.get(n, []))
    return {l.name for l in robot.links} - lift_sub


def _xy_extent(geom, origin, centre):
    """(xmin, xmax, ymin, ymax) of a box or y-axis cylinder at `centre` + origin offset."""
    ox, oy = (origin.xyz[0], origin.xyz[1]) if origin else (0.0, 0.0)
    cx, cy = centre[0] + ox, centre[1] + oy
    if hasattr(geom, 'size'):
        hx, hy = geom.size[0] / 2, geom.size[1] / 2
    elif hasattr(geom, 'length'):  # cylinder rolled onto its side: axis along y
        hx, hy = geom.radius, geom.length / 2
    else:  # sphere
        hx = hy = geom.radius
    return cx - hx, cx + hx, cy - hy, cy + hy


def _render_base(base, arm='generic_6dof'):
    return URDF.from_xml_string(_render(arm=arm, base=base))


def test_profiles_exist_for_every_base():
    names = {_load(p)['base']['name'] for p in _profile_files()}
    assert set(BASES) <= names
    assert 'ranger_mini' in names and (PROFILES / 'ur10e_ranger_mini_telescopic.yaml').exists()


@pytest.mark.parametrize('base', BASES)
def test_base_contract_and_tree(base):
    robot = _render_base(base)
    links = {l.name for l in robot.links}
    for needed in ('base_footprint', 'base_link', 'lift_mount_link'):
        assert needed in links, f'{base}: missing {needed}'

    parents = {}
    for j in robot.joints:
        assert j.child not in parents, f'{base}: link {j.child} has two parents'
        parents[j.child] = j.parent
    roots = links - set(parents)
    assert roots == {'base_footprint'}, f'{base}: roots are {roots}'
    for link in links:  # every link reaches the root without a cycle
        seen, node = set(), link
        while node in parents:
            assert node not in seen, f'{base}: cycle at {node}'
            seen.add(node)
            node = parents[node]

    for name in _base_links(robot):
        joint = next((j for j in robot.joints if j.child == name), None)
        if joint is not None:
            assert joint.type == 'fixed', f'{base}: {joint.name} must be fixed'

    pos = _positions(robot)
    assert pos['base_footprint'][2] == 0.0
    assert pos['lift_mount_link'][2] > pos['base_link'][2] > 0.0


@pytest.mark.parametrize('path', _profile_files(), ids=lambda p: p.stem)
def test_footprint_contains_the_base_body(path):
    profile = _load(path)['base']
    base = profile['name']
    poly = profile['footprint']
    assert len(poly) >= 3
    xs, ys = [p[0] for p in poly], [p[1] for p in poly]
    # The footprint is a rectangle; check it is axis-aligned so a bounding-box test is exact.
    assert sorted(set(xs)) == [min(xs), max(xs)] and sorted(set(ys)) == [min(ys), max(ys)]

    robot = _render_base(base)
    pos = _positions(robot)
    checked = 0
    lo_x = lo_y = float('inf')
    hi_x = hi_y = float('-inf')
    for link in robot.links:
        if link.name not in _base_links(robot):
            continue
        for shape in (link.visual, link.collision):
            if shape is None:
                continue
            x0, x1, y0, y1 = _xy_extent(shape.geometry, shape.origin, pos[link.name])
            assert min(xs) - 1e-3 <= x0 and x1 <= max(xs) + 1e-3, f'{base}/{link.name} x outside'
            assert min(ys) - 1e-3 <= y0 and y1 <= max(ys) + 1e-3, f'{base}/{link.name} y outside'
            lo_x, hi_x, lo_y, hi_y = min(lo_x, x0), max(hi_x, x1), min(lo_y, y0), max(hi_y, y1)
            checked += 1
    assert checked >= 2
    # Not absurdly loose: within 5 cm of the tightest box around body, wheels and casters.
    assert (max(xs) - min(xs)) <= (hi_x - lo_x) + 0.05 and (max(ys) - min(ys)) <= (hi_y - lo_y) + 0.05


@pytest.mark.parametrize('path', _profile_files(), ids=lambda p: p.stem)
def test_profile_agrees_with_the_model(path):
    profile = _load(path)
    b = profile['base']
    base = b['name']
    assert profile['robot']['xacro_args']['base'] == base
    assert b['drive_type'] in DRIVE_TYPES
    assert isinstance(b['can_strafe'], bool)
    for key in ('max_linear_velocity_mps', 'max_angular_velocity_radps',
                'max_linear_acceleration_mps2', 'max_angular_acceleration_radps2', 'mass_kg'):
        assert b[key] > 0
    if b['drive_type'] in ('omni', '4ws'):
        assert b['can_strafe']
    if b['drive_type'] in ('diff', 'skid'):
        assert not b['can_strafe']

    robot = _render_base(base)
    pos = _positions(robot)
    assert pos['lift_mount_link'][2] == pytest.approx(b['lift_mount_height_m'], abs=1e-5)
    body = next(l for l in robot.links if l.name == 'base_link')
    bottom = pos['base_link'][2] - body.collision.geometry.size[2] / 2
    assert bottom == pytest.approx(b['ground_clearance_m'], abs=1e-5)
    assert body.inertial.mass == pytest.approx(b['mass_kg'])

    wheels = [l for l in robot.links if l.name.endswith('_wheel_link')]
    if b['wheel_radius_m'] is None:
        assert not wheels
        return
    assert len(wheels) in (2, 4)
    for w in wheels:
        assert w.visual.geometry.radius == pytest.approx(b['wheel_radius_m'])
        assert pos[w.name][2] == pytest.approx(b['wheel_radius_m'], abs=1e-5)  # rests on ground
    ys = [pos[w.name][1] for w in wheels]
    assert max(ys) - min(ys) == pytest.approx(b['track_m'], abs=1e-5)
    if b['wheelbase_m'] is not None:
        wx = [pos[w.name][0] for w in wheels]
        assert max(wx) - min(wx) == pytest.approx(b['wheelbase_m'], abs=1e-5)


@pytest.mark.parametrize('base', BASES)
def test_footprint_sits_on_the_ground_plane_below_the_body(base):
    robot = _render_base(base)
    pos = _positions(robot)
    for link in robot.links:
        if link.name in _base_links(robot) and link.collision is not None:
            z = pos[link.name][2]
            geom = link.collision.geometry
            half = geom.size[2] / 2 if hasattr(geom, 'size') else geom.radius
            assert z - half >= -1e-6, f'{base}/{link.name} goes below the ground'


@needs_ur
@pytest.mark.parametrize('base', BASES)
def test_whole_robot_renders_with_ur10e_and_telescopic_lift(base):
    robot = URDF.from_xml_string(_render(arm='ur10e', base=base, lift_stroke=1.5))
    links = {l.name for l in robot.links}
    for needed in ('base_footprint', 'base_link', 'lift_mount_link', 'lift_base_link',
                   'lift_output_link', 'arm_base_link', 'arm_flange_link', 'tcp_link'):
        assert needed in links
    actuated = [j.name for j in robot.joints
                if j.type in ('revolute', 'prismatic', 'continuous') and j.mimic is None]
    assert len(actuated) == 7, f'{base}: wheels must not add actuated joints, got {actuated}'
    assert robot.joint_map['lift_joint'].limit.upper == pytest.approx(1.5)
    chain = robot.get_chain('base_link', 'tcp_link', links=False, fixed=False)
    assert [j for j in chain if robot.joint_map[j].mimic is None][0] == 'lift_joint'
    # The lift attaches at the mount, whatever the base is.
    lift = robot.joint_map['lift_base_joint'] if 'lift_base_joint' in robot.joint_map else None
    if lift is not None:
        assert lift.parent == 'lift_mount_link'


@needs_ur
@pytest.mark.parametrize('path', _profile_files(), ids=lambda p: p.stem)
def test_every_profile_renders_with_its_base(path):
    profile = _load(path)
    robot = URDF.from_xml_string(_render(**_profile_args(profile)))
    links = {l.name for l in robot.links}
    assert set(profile['frames'].values()) <= links
    actuated = [j.name for j in robot.joints
                if j.type in ('revolute', 'prismatic', 'continuous') and j.mimic is None]
    lift = profile['joints']['lift']          # null for lift:=none / fixed, which has no actuated lift joint
    assert sorted(actuated) == sorted(([lift] if lift else []) + profile['joints']['arm'])


def test_default_base_is_generic_base():
    default = _positions(URDF.from_xml_string(_render(arm='generic_6dof')))
    generic = _positions(_render_base('generic_base'))
    assert default == generic
    assert not [l for l in URDF.from_xml_string(_render(arm='generic_6dof')).links
                if l.name.endswith('_wheel_link')]


def test_base_parameters_can_be_overridden(tmp_path):
    """The macros stay parameterised: a longer ranger body and a prefix."""
    path = PKG / 'urdf' / 'base' / 'ranger_mini.xacro'
    wrapper = tmp_path / 'wrapper.urdf.xacro'
    wrapper.write_text('<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="t">'
                       f'<xacro:include filename="{path}"/>'
                       '<xacro:ranger_mini_base prefix="x_" length="1.0" track="0.5"/></robot>')
    robot = URDF.from_xml_string(xacro.process_file(str(wrapper)).toxml())
    assert {'x_base_footprint', 'x_base_link', 'x_lift_mount_link'} <= {l.name for l in robot.links}
    assert robot.link_map['x_base_link'].collision.geometry.size[0] == pytest.approx(1.0)


def test_unknown_base_fails_loudly():
    with pytest.raises(Exception) as err:
        _render(arm='generic_6dof', base='not_a_base')
    assert 'unknown base_type' in str(err.value)


def _load_module(name, rel):
    import importlib.util
    spec = importlib.util.spec_from_file_location(name, PKG / rel)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_display_launch_forwards_base():
    launch = pytest.importorskip('launch')
    display = _load_module('display_launch', 'launch/display.launch.py')
    context = launch.LaunchContext()
    context.launch_configurations.update({'arm': 'generic_6dof', 'base': 'husky', 'lift_stroke': '1.5'})
    assert display._mappings(context)['base'] == 'husky'
    declared = [a.name for a in display.generate_launch_description().entities
                if isinstance(a, launch.actions.DeclareLaunchArgument)]
    assert 'base' in declared


def test_generate_acm_takes_a_base(tmp_path):
    """With a wheeled base the matrix lists the wheel/body pairs."""
    import subprocess
    import sys
    out = tmp_path / 'acm.xacro'
    subprocess.run([sys.executable, str(PKG / 'scripts' / 'generate_acm.py'), '--arm', 'generic_6dof',
                    '--base', 'scout_mini', '--out', str(out)], check=True, capture_output=True)
    text = out.read_text()
    assert 'base_link' in text and '_wheel_link' in text


@pytest.mark.parametrize('path', _profile_files(), ids=lambda p: p.stem)
def test_profile_arm_mount_matches_the_arm_base_position(path):
    """Stationing places the footprint by `arm_mount_xy`; it must be where the URDF puts the arm."""
    profile = _load(path)
    robot = _render_base(profile['base']['name'])
    x = y = 0.0
    for name in robot.get_chain('base_footprint', 'arm_base_link', links=False):
        origin = robot.joint_map[name].origin
        if origin is not None:
            assert all(abs(a) < TOL for a in origin.rpy), f'{name} rotates'
            x, y = x + origin.xyz[0], y + origin.xyz[1]
    assert profile['base']['arm_mount_xy'] == pytest.approx([x, y], abs=1e-6)
