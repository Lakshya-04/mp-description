# mp-description

Robot model and profiles for the Motion-Planner stack: URDF/xacro description and the robot/tool profile package.

This repository is a submodule of [Lakshya-04/Motion-Planner](https://github.com/Lakshya-04/Motion-Planner), which holds the core packages, the Docker setup, the documentation and the contributing guide. It is checked out at `src/mp-description` and built together with the rest of the workspace (colcon); the packages depend on packages in the main repository and in the other `mp-*` repositories, so it does not build on its own.

Packages: mobile_manipulator_description, mobile_manipulator_profile.

Issues, pull requests and the contributing rules (Conventional Commits, one theme per PR, squash merge) are in the main repository: open them there and say which package is concerned.

Licence: Apache-2.0 (see `LICENSE`).
