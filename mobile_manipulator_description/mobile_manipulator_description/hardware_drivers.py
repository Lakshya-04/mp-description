"""Arm -> vendor ros2_control driver table for hardware:=real (config/hardware_drivers.yaml).

    driver_for_arm('ur10e')            # ('ur', {...table entry...})
    resolve('ur10e', {'robot_ip': '192.168.1.5', ...})    # plugin and params, or HardwareDriverError
    check_installed('ur')              # HardwareDriverError naming the missing package

The xacro (mobile_manipulator.ros2_control.xacro) reads the same yaml; the tests check that the two agree.
"""
import os

import yaml

from .profile import profiles_dir


class HardwareDriverError(RuntimeError):
    """A hardware:=real choice cannot be satisfied (unknown arm, missing parameter, driver not installed)."""


def table_path():
    return os.path.join(os.path.dirname(profiles_dir()), 'hardware_drivers.yaml')


def load_table(path=None):
    with open(path or table_path()) as f:
        return yaml.safe_load(f)['drivers']


def parse_params(text):
    """'a=1,b=2' -> {'a': '1', 'b': '2'} (the real_params xacro argument)."""
    out = {}
    for item in filter(None, (s.strip() for s in (text or '').split(','))):
        if '=' not in item:
            raise HardwareDriverError(f"real_params entry '{item}' is not key=value")
        key, value = item.split('=', 1)
        out[key.strip()] = value.strip()
    return out


def driver_for_arm(arm, table=None):
    table = table or load_table()
    for key, entry in table.items():
        if arm in entry.get('arms', []):
            return key, entry
    known = sorted(a for e in table.values() for a in e.get('arms', []))
    raise HardwareDriverError(
        f"no vendor driver is listed for arm '{arm}' in hardware_drivers.yaml (known arms: "
        f"{', '.join(known)}); add an entry, or use real_driver:=mock to test the wiring")


def select(arm, driver='auto', table=None):
    table = table or load_table()
    if driver in ('', 'auto'):
        return driver_for_arm(arm, table)
    if driver not in table:
        raise HardwareDriverError(f"unknown real_driver '{driver}' (choose from: {', '.join(table)})")
    return driver, table[driver]


def resolve(arm, params=None, driver='auto', table=None):
    """Plugin and parameters for `arm`: defaults overridden by `params`; missing required ones fail."""
    key, entry = select(arm, driver, table)
    merged = {k: str(v) for k, v in (entry.get('params') or {}).items()}
    merged.update(params or {})
    missing = [p for p in entry.get('required_params', []) if p not in merged]
    if missing:
        raise HardwareDriverError(
            f"hardware:=real for arm '{arm}' (driver '{key}') needs: {', '.join(missing)}. "
            f"Pass real_params:=\"{','.join(m + '=<value>' for m in missing)}\"")
    return {'driver': key, 'plugin': entry['plugin'], 'params': merged, 'entry': entry}


def package_installed(package):
    if not package:
        return True
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory(package)
        return True
    except Exception:  # noqa: BLE001  (PackageNotFoundError and friends)
        return False


def check_installed(key, table=None, installed=package_installed):
    entry = (table or load_table())[key]
    package = entry.get('package', '')
    if package and not installed(package):
        raise HardwareDriverError(
            f"driver package '{package}' for '{key}' is not installed. This repository does not ship vendor "
            f"drivers: install it (see docs/hardware_bringup.md, section 'Vendor drivers'), source its "
            f"workspace, and launch again. To test the wiring without a robot use real_driver:=mock.")


def plan_real_hardware(profile, real_driver='auto', real_params='', real_lift_plugin='',
                       installed=package_installed, table=None):
    """Validate a hardware:=real launch and return the xacro mappings that render it.

    `profile` is a Profile (profile.load_profile). Raises HardwareDriverError, with a message that says
    what to do, for: an arm with no driver, a missing required parameter, a moving lift without a lift
    plugin, a driver package that is not installed.
    """
    table = table or load_table()
    resolved = resolve(profile.arm, parse_params(real_params), real_driver, table)
    if profile.lift_joint and not (real_lift_plugin or resolved['entry'].get('lift_plugin')):
        raise HardwareDriverError(
            f"profile '{profile.name}' has a moving lift ({profile.lift_joint}) and no arm vendor driver "
            f"drives it: pass real_lift_plugin:=PLUGIN (the ros2_control plugin of your lift), or use a "
            f"profile without a lift")
    check_installed(resolved['driver'], table, installed)
    mappings = profile.xacro_mappings()
    mappings.update(hardware='real', real_driver=resolved['driver'], real_params=real_params or '',
                    real_lift_plugin=real_lift_plugin or '')
    return mappings
