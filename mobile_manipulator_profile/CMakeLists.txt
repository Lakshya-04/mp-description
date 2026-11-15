cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_profile)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# Header-only reader of the robot profile YAML (see mobile_manipulator_description).
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)
target_link_libraries(${PROJECT_NAME} INTERFACE ament_index_cpp::ament_index_cpp yaml-cpp)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets)
ament_export_targets(${PROJECT_NAME}Targets)
ament_export_dependencies(ament_index_cpp yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_profile test/test_profile.cpp)
  target_link_libraries(test_profile ${PROJECT_NAME})
  # Read the profiles straight from the source tree.
  target_compile_definitions(test_profile PRIVATE
    PROFILE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../mobile_manipulator_description/config/profiles")
endif()

ament_package()
