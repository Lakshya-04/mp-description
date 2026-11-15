cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python profile loader: `from mobile_manipulator_description import load_profile`.
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY urdf srdf config launch rviz DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest QUIET)
  if(ament_cmake_pytest_FOUND)
    ament_add_pytest_test(test_description test/test_description.py)
    ament_add_pytest_test(test_profile test/test_profile.py)
    ament_add_pytest_test(test_arm_swap test/test_arm_swap.py)
    ament_add_pytest_test(test_hardware_selector test/test_hardware_selector.py)
  endif()
endif()

ament_package()
