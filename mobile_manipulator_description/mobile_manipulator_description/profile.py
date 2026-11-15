"""Robot profile loader: the Python reader of config/profiles/*.yaml.

A profile names everything that differs between arms: planning-group names, the lift joint, the
arm joints in kinematic order (so the arm DOF), the frames, and the home pose. Nodes and launch
files read it instead of repeating names. The C++ twin is mobile_manipulator_profile.

    p = load_profile('ur10e_telescopic')      # or a path to a .yaml file
    p.whole_body_group, p.arm_dof, p.joints   # 'lift_arm', 6, ['lift_joint', ...]
"""
import dataclasses
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import yaml

DEFAULT_PROFILE = 'ur10e_telescopic'
_DEFAULT_GROUPS = {'arm': 'arm', 'whole_body': 'lift_arm', 'tool': 'tool'}
MAX_ARM_JOINTS = 12   # the ros2_control and SRDF xacro files unroll one entry per joint up to this
_REQUIRED_FRAMES = ('world_base', 'base', 'arm_base', 'flange', 'tool_mount', 'tcp')


class ProfileError(ValueError):
    """The profile is missing a field or is internally inconsistent."""


def profiles_dir():
    """Directory holding the profiles: the installed share dir, else the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('mobile_manipulator_description')
        path = os.path.join(share, 'config', 'profiles')
        if os.path.isdir(path):
            return path
    except Exception:  # noqa: BLE001  (not installed: running from the source tree)
        pass
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'profiles')


def profile_for_arm(arm):
    """Profile that goes with an `arm` xacro argument (used when only the arm is given)."""
    if arm.startswith('ur'):
        return 'ur10e_telescopic'
    return f'{arm}_telescopic'


@dataclass(frozen=True)
class Profile:
    name: str
    robot_name: str
    xacro_args: Dict[str, str]
    frames: Dict[str, str]
    lift_joint: Optional[str]  # None: no actuated lift (lift:=none / fixed)
    arm_joints: List[str]
    mimic_joints: List[str]
    home: Dict[str, float]
    groups: Dict[str, str]
    controller_family: str
    raw: dict = field(repr=False, compare=False, default_factory=dict)
    path: str = field(default='', compare=False)  # YAML file it was read from (empty for parse_profile)

    @property
    def arm(self):
        return str(self.xacro_args['arm'])

    @property
    def arm_dof(self):
        return len(self.arm_joints)

    @property
    def dof(self):
        """Whole-body DOF: the lift (if actuated) plus the arm."""
        return len(self.joints)

    @property
    def joints(self):
        """Actuated joints of the whole-body group, kinematic order (lift first)."""
        return ([self.lift_joint] if self.lift_joint else []) + list(self.arm_joints)

    @property
    def arm_group(self):
        return self.groups['arm']

    @property
    def whole_body_group(self):
        return self.groups['whole_body']

    @property
    def tool_group(self):
        return self.groups['tool']

    @property
    def tcp_link(self):
        return self.frames['tcp']

    @property
    def home_positions(self):
        """Home pose as a list in `joints` order."""
        return [self.home[j] for j in self.joints]

    def xacro_mappings(self):
        """xacro mappings that render this profile's robot (strings)."""
        return {k: str(v) for k, v in self.xacro_args.items()}


def _resolve(name_or_path):
    if name_or_path.endswith(('.yaml', '.yml')) or os.sep in name_or_path:
        return name_or_path
    return os.path.join(profiles_dir(), f'{name_or_path}.yaml')


def parse_profile(data, name='profile'):
    """Validate a parsed profile dict and wrap it."""
    try:
        joints = data['joints']
        frames = data['frames']
        xacro_args = data['robot']['xacro_args']
        lift_joint = joints.get('lift') or None  # `lift: null` = no actuated lift
        arm_joints = list(joints['arm'])
        if 'arm' not in xacro_args:
            raise KeyError('robot.xacro_args.arm')
    except (KeyError, TypeError) as err:
        raise ProfileError(f'{name}: missing field {err}') from err
    missing = [k for k in _REQUIRED_FRAMES if k not in frames]
    if missing:
        raise ProfileError(f'{name}: frames missing {missing}')
    if not arm_joints:
        raise ProfileError(f'{name}: joints.arm is empty')
    if len(set(arm_joints + ([lift_joint] if lift_joint else []))) != len(
            arm_joints) + (1 if lift_joint else 0):
        raise ProfileError(f'{name}: duplicate joint names')
    if len(arm_joints) > MAX_ARM_JOINTS:
        raise ProfileError(f'{name}: {len(arm_joints)} arm joints, at most {MAX_ARM_JOINTS} are supported')
    mimic = list(joints.get('mimic') or [])
    if any(j in arm_joints or j == lift_joint for j in mimic) or len(set(mimic)) != len(mimic):
        raise ProfileError(f'{name}: joints.mimic must be unique and not an arm or lift joint '
                           f'(got {mimic})')
    arm_section = data.get('arm') or {}
    try:
        dof = int(arm_section['dof']) if 'dof' in arm_section else len(arm_joints)
        home = {k: float(v) for k, v in (data.get('home') or {}).items()}
    except (TypeError, ValueError) as err:
        raise ProfileError(f'{name}: arm.dof or a home value is not a number ({err})') from err
    if dof != len(arm_joints):
        raise ProfileError(f'{name}: arm.dof={dof} but joints.arm lists {len(arm_joints)}')
    missing_home = [j for j in ([lift_joint] if lift_joint else []) + arm_joints if j not in home]
    if missing_home:
        raise ProfileError(f'{name}: home pose misses joints {missing_home}')
    groups = dict(_DEFAULT_GROUPS)
    groups.update(data.get('groups') or {})
    return Profile(
        name=name, robot_name=data['robot'].get('name', 'mobile_manipulator'),
        xacro_args={k: str(v) for k, v in xacro_args.items()}, frames=dict(frames),
        lift_joint=lift_joint, arm_joints=arm_joints,
        mimic_joints=mimic, home=home, groups=groups,
        controller_family=str(arm_section.get(
            'controller_family', 'ur' if str(xacro_args['arm']).startswith('ur') else xacro_args['arm'])),
        raw=data)


def load_profile(name_or_path=DEFAULT_PROFILE):
    """Load a profile by name ('ur10e_telescopic') or by path to a .yaml file."""
    path = _resolve(name_or_path)
    if not os.path.isfile(path):
        raise ProfileError(f'profile not found: {path}')
    with open(path) as f:
        data = yaml.safe_load(f)
    profile = parse_profile(data, os.path.splitext(os.path.basename(path))[0])
    return dataclasses.replace(profile, path=os.path.abspath(path))
