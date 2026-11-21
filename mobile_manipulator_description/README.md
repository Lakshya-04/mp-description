# mobile_manipulator_description

Generic mobile-manipulator description: a base, a swappable lift (telescopic with mimic stages, scissor, fixed or none), a swappable arm (a primitives-only generic 6-DOF reference arm, several 7-DOF arms, and the UR family through `ur_description`) and a tool. A profile YAML is the single source of group names, joint names, frames, limits and home pose.

## Tier

Message-free description package: xacro/URDF/SRDF, profile YAML, small Python helpers and launch files. ROS is needed to render and launch it.

## Inputs and outputs

- Top-level xacro `urdf/mobile_manipulator.urdf.xacro` (arguments include `arm`, `base`, `lift_stroke`, `lift_stages`, `tcp_xyz`); `urdf/mobile_manipulator.ros2_control.xacro` for the `hardware:=` choice; `srdf/mobile_manipulator.srdf.xacro` with a generated allowed-collision matrix per arm.
- Profiles: `config/profiles/*.yaml`, one per arm/base/lift combination (for example `generic_6dof_telescopic`, `panda_telescopic`, `ur10e_telescopic`). Sections: `robot.xacro_args`, `groups`, `arm` (DOF), `frames`, `joints`, `lift`, `home`, `base`.
- Python module `mobile_manipulator_description` (`profile.py`: `load_profile`, `parse_profile`, `profile_for_arm`; `hardware_drivers.py`: the driver table behind `hardware:=real`). Table: `config/hardware_drivers.yaml`.
- Launch: `display.launch.py` (arguments `arm`, `base`, `lift_stroke`), `hardware_real.launch.py` (arguments `profile`, `real_driver`, `real_params`, `real_lift_plugin`, `controllers_file`, `controller`, `update_rate`).
- Scripts: `scripts/generate_acm.py`, `scripts/profile_xacro_args.py`.

## Dependencies

- In this repo: none.
- External: `xacro`, `robot_state_publisher`, `joint_state_publisher_gui`, `rviz2`, `controller_manager`, `urdfdom_py`, `python3-yaml`, `ur_description` (only for the UR arms, not bundled).
- Needs ROS 2 to render and launch; the profile loader is plain Python. No GPU. No simulator needed.

## Use it on its own

```bash
ros2 launch mobile_manipulator_description display.launch.py arm:=generic_6dof
xacro urdf/mobile_manipulator.urdf.xacro arm:=generic_6dof -o /tmp/robot.urdf   # run in the package directory
```

To add your own arm, copy a profile and an arm xacro and follow the contract in the robot description doc.

## Tests

86 test cases in 7 Python files under `test/` (counted from the sources): profile parsing, arm and lift swaps, bases, hardware selector, the real-hardware wiring and the description itself.

```bash
colcon test --packages-select mobile_manipulator_description
```

## Status and limits

- Models use published kinematics and limits; the 7-DOF arms and the generic arm are primitives only (no vendor meshes).
- Base numbers are nominal; check them against your unit.
- `hardware:=real` reads a driver table written from public driver documentation; it was not run on hardware or checked against an installed driver. Nothing here has run on hardware.

## Related docs

- [robot_description.md](../../../docs/robot_description.md)
- [robot_assets.md](../../../docs/robot_assets.md)
- [hardware_bringup.md](../../../docs/hardware_bringup.md)
- [simulation.md](../../../docs/simulation.md)
