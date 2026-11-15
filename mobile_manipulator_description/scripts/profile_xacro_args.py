#!/usr/bin/env python3
"""Turn a profile YAML into the xacro arguments that render its robot.

`robot.xacro_args` is passed as is; the profile's `lift:` section then fills the lift arguments
(lift, lift_stroke, lift_stages, lift_max_velocity, lift_max_acceleration, lift_mount_height) and wins
over a lift_* entry in xacro_args, so a profile states each lift value once.

    profile_xacro_args.py profile.yaml          # prints  key:=value  pairs, one per line
"""
import sys

import yaml

LIFT_KEYS = {
    'type': 'lift',
    'stroke_m': 'lift_stroke',
    'stages': 'lift_stages',
    'max_velocity_mps': 'lift_max_velocity',
    'max_acceleration_mps2': 'lift_max_acceleration',
    'mount_height_m': 'lift_mount_height',
}


def xacro_args(profile):
    args = {k: str(v) for k, v in (profile.get('robot', {}).get('xacro_args') or {}).items()}
    lift = profile.get('lift') or {}
    for key, arg in LIFT_KEYS.items():
        if key in lift:
            args[arg] = str(lift[key])
    return args


def load(path):
    with open(path) as f:
        return yaml.safe_load(f)


if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    for k, v in xacro_args(load(sys.argv[1])).items():
        print(f'{k}:={v}')
