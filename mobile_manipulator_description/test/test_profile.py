"""Profile loader tests: every shipped profile loads, is consistent, and exposes names and DOF."""
import pathlib
import sys

import pytest
import yaml

PKG = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

from mobile_manipulator_description import (  # noqa: E402
    ProfileError, load_profile, parse_profile, profile_for_arm)

PROFILES = sorted(p.stem for p in (PKG / 'config' / 'profiles').glob('*.yaml'))


def test_expected_profiles_exist():
    assert {'ur10e_telescopic', 'generic_6dof_telescopic', 'panda_telescopic'} <= set(PROFILES)


@pytest.mark.parametrize('name', PROFILES)
def test_profile_is_consistent(name):
    p = load_profile(name)
    assert p.name == name
    assert p.dof == len(p.joints) == p.arm_dof + (1 if p.lift_joint else 0)
    assert p.joints[0] == (p.lift_joint or p.arm_joints[0])
    assert len(p.home_positions) == p.dof
    assert p.whole_body_group != p.arm_group
    assert p.tcp_link == 'tcp_link'


def test_dof_per_profile():
    assert load_profile('ur10e_telescopic').arm_dof == 6
    assert load_profile('generic_6dof_telescopic').arm_dof == 6
    assert load_profile('panda_telescopic').arm_dof == 7


def test_no_lift_profile_has_only_arm_joints():
    p = load_profile('ur10e_nolift')
    assert p.lift_joint is None and p.joints == p.arm_joints and p.dof == 6
    assert p.controller_family == 'ur'


def test_default_group_names_match_the_srdf():
    p = load_profile('ur10e_telescopic')
    assert (p.arm_group, p.whole_body_group, p.tool_group) == ('arm', 'lift_arm', 'tool')


def test_load_by_path():
    path = PKG / 'config' / 'profiles' / 'panda_telescopic.yaml'
    assert load_profile(str(path)).arm == 'panda'


def test_profile_for_arm():
    assert profile_for_arm('ur5e') == 'ur10e_telescopic'
    assert profile_for_arm('panda') == 'panda_telescopic'


def test_missing_profile_raises():
    with pytest.raises(ProfileError):
        load_profile('nope')


def test_dof_mismatch_is_rejected():
    data = yaml.safe_load((PKG / 'config' / 'profiles' / 'panda_telescopic.yaml').read_text())
    data['arm']['dof'] = 6
    with pytest.raises(ProfileError, match='arm.dof'):
        parse_profile(data)


def test_home_must_cover_every_joint():
    data = yaml.safe_load((PKG / 'config' / 'profiles' / 'panda_telescopic.yaml').read_text())
    del data['home']['arm_joint7']
    with pytest.raises(ProfileError, match='home'):
        parse_profile(data)


def _panda():
    return yaml.safe_load((PKG / 'config' / 'profiles' / 'panda_telescopic.yaml').read_text())


def test_more_arm_joints_than_the_xacro_unrolls_are_rejected():
    data = _panda()
    names = [f'arm_joint{i}' for i in range(1, 14)]
    data['joints']['arm'] = names
    data['arm']['dof'] = 13
    data['home'] = {n: 0.0 for n in names + [data['joints']['lift']]}
    with pytest.raises(ProfileError, match='at most 12'):
        parse_profile(data)


def test_mimic_joint_must_not_repeat_or_overlap():
    data = _panda()
    data['joints']['mimic'] = ['arm_joint1']
    with pytest.raises(ProfileError, match='mimic'):
        parse_profile(data)


def test_non_numeric_home_value_names_the_profile():
    data = _panda()
    data['home']['arm_joint1'] = 'up'
    with pytest.raises(ProfileError, match='home value'):
        parse_profile(data, 'broken')
