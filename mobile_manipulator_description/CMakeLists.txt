cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf srdf config launch rviz DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest QUIET)
  if(ament_cmake_pytest_FOUND)
    ament_add_pytest_test(test_description test/test_description.py)
  endif()
endif()

ament_package()
