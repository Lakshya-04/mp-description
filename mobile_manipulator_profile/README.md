# mobile_manipulator_profile

Header-only C++ reader of the robot profile YAML that lives in `mobile_manipulator_description`: planning group names, the lift and arm joint lists (so the arm DOF), frames and the home pose.

## Tier

ROS-free core library (header-only `INTERFACE` library).

## Inputs and outputs

- Header `include/mobile_manipulator_profile/profile.hpp`, namespace of the same name.
- `struct Profile`: `name`, `robot_name`, `arm`, `lift_joint`, `arm_joints`, `mimic_joints`, `home`, `frames`, group names (`arm_group`, `whole_body_group`, `tool_group`), `controller_family`; helpers `armDof()`, `dof()`, `joints()`, `homePositions()`, `frameOr(key, fallback)`.
- Functions: `loadProfile(name_or_path)` (default `ur10e_telescopic`), `loadProfileOrDefaults(name_or_path, warning)`, `parseProfile(node, name)`, `profilesDir()`.
- Input: a profile YAML, by name (looked up in the description package) or by file path.

## Dependencies

- In this repo: `mobile_manipulator_description` (for the shipped profiles, found through `ament_index_cpp`).
- External: `yaml-cpp`, `ament_index_cpp`.
- Does not use `rclcpp`; building it with `ament_cmake` still needs the ament packages. No GPU. No simulator.

## Use it on its own

```cmake
find_package(mobile_manipulator_profile REQUIRED)
target_link_libraries(my_target mobile_manipulator_profile::mobile_manipulator_profile)
```

```cpp
auto p = mobile_manipulator_profile::loadProfile("/path/my_robot.yaml");
size_t dof = p.dof();
```

## Tests

9 test cases in `test/test_profile.cpp` (counted from the source).

```bash
colcon test --packages-select mobile_manipulator_profile
```

## Status and limits

- Reads only; it does not validate the xacro against the profile (the Python tests in the description package do).
- Planning code that takes joint and frame names from here works for any profile with the same sections; nothing here has run on hardware.

## Related docs

- [robot_description.md](../../../docs/robot_description.md)
- [package_layout.md](../../../docs/package_layout.md)
