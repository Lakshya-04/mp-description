// Header-only reader of the robot profile (mobile_manipulator_description/config/profiles/*.yaml),
// the C++ twin of mobile_manipulator_description.profile. Nodes take group names, joint lists,
// DOF and frame names from here instead of repeating them.
//
//   auto p = mobile_manipulator_profile::loadProfile("ur10e_telescopic");
//   p.whole_body_group;   // "lift_arm"
//   p.dof();              // 7 (lift + 6 arm joints); 8 for the 7-DOF arm
//   p.joints();           // lift joint first, then the arm joints in kinematic order
#pragma once

#include <cstdlib>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"

namespace mobile_manipulator_profile
{

inline constexpr char kDefaultProfile[] = "ur10e_telescopic";

struct Profile
{
  std::string name;
  std::string robot_name;
  std::string arm;  ///< xacro `arm` argument, e.g. "ur10e", "generic_6dof", "panda".
  std::string lift_joint;  ///< Empty: no actuated lift (lift:=none / fixed).
  std::vector<std::string> arm_joints;
  std::vector<std::string> mimic_joints;
  std::map<std::string, double> home;
  std::map<std::string, std::string> frames;
  std::string arm_group = "arm";
  std::string whole_body_group = "lift_arm";
  std::string tool_group = "tool";
  std::string controller_family;

  size_t armDof() const {return arm_joints.size();}
  /// Whole-body DOF: the lift (if actuated) plus the arm.
  size_t dof() const {return joints().size();}
  /// Actuated joints of the whole-body group, lift first.
  std::vector<std::string> joints() const
  {
    std::vector<std::string> all;
    if (!lift_joint.empty()) {all.push_back(lift_joint);}
    all.insert(all.end(), arm_joints.begin(), arm_joints.end());
    return all;
  }
  /// Home pose in joints() order.
  std::vector<double> homePositions() const
  {
    std::vector<double> q;
    for (const auto & j : joints()) {q.push_back(home.at(j));}
    return q;
  }
  const std::string & tcpLink() const {return frames.at("tcp");}
  const std::string & frame(const std::string & key) const {return frames.at(key);}
  std::string frameOr(const std::string & key, const std::string & fallback) const
  {
    const auto it = frames.find(key);
    return it == frames.end() ? fallback : it->second;
  }
};

/// Directory with the profile files: $MOBILE_MANIPULATOR_PROFILE_DIR if set, else the installed
/// share directory of mobile_manipulator_description.
inline std::string profilesDir()
{
  if (const char * env = std::getenv("MOBILE_MANIPULATOR_PROFILE_DIR")) {
    if (*env != '\0') {return env;}
  }
  return ament_index_cpp::get_package_share_directory("mobile_manipulator_description") +
         "/config/profiles";
}

/// Validate and read a parsed profile. Throws std::runtime_error on a missing or inconsistent field.
inline Profile parseProfile(const YAML::Node & root, const std::string & name)
{
  auto fail = [&name](const std::string & what) {
      throw std::runtime_error("profile " + name + ": " + what);
    };
  Profile p;
  p.name = name;
  if (!root["robot"] || !root["robot"]["xacro_args"] || !root["robot"]["xacro_args"]["arm"]) {
    fail("missing robot.xacro_args.arm");
  }
  p.robot_name = root["robot"]["name"] ? root["robot"]["name"].as<std::string>() :
    "mobile_manipulator";
  p.arm = root["robot"]["xacro_args"]["arm"].as<std::string>();
  if (!root["joints"] || !root["joints"]["arm"]) {fail("missing joints.arm");}
  if (root["joints"]["lift"] && !root["joints"]["lift"].IsNull()) {
    p.lift_joint = root["joints"]["lift"].as<std::string>();
  }
  p.arm_joints = root["joints"]["arm"].as<std::vector<std::string>>();
  if (root["joints"]["mimic"] && !root["joints"]["mimic"].IsNull()) {
    p.mimic_joints = root["joints"]["mimic"].as<std::vector<std::string>>();
  }
  if (p.arm_joints.empty()) {fail("joints.arm is empty");}
  std::set<std::string> unique(p.arm_joints.begin(), p.arm_joints.end());
  if (!p.lift_joint.empty()) {unique.insert(p.lift_joint);}
  if (unique.size() != p.arm_joints.size() + (p.lift_joint.empty() ? 0 : 1)) {
    fail("duplicate joint names");
  }
  if (!root["frames"]) {fail("missing frames");}
  for (const auto & kv : root["frames"]) {
    p.frames[kv.first.as<std::string>()] = kv.second.as<std::string>();
  }
  for (const char * key : {"world_base", "base", "arm_base", "flange", "tool_mount", "tcp"}) {
    if (!p.frames.count(key)) {fail(std::string("frames.") + key + " missing");}
  }
  if (root["groups"]) {
    const auto g = root["groups"];
    if (g["arm"]) {p.arm_group = g["arm"].as<std::string>();}
    if (g["whole_body"]) {p.whole_body_group = g["whole_body"].as<std::string>();}
    if (g["tool"]) {p.tool_group = g["tool"].as<std::string>();}
  }
  p.controller_family = p.arm.rfind("ur", 0) == 0 ? "ur" : p.arm;
  if (root["arm"]) {
    if (root["arm"]["controller_family"]) {
      p.controller_family = root["arm"]["controller_family"].as<std::string>();
    }
    if (root["arm"]["dof"] && root["arm"]["dof"].as<size_t>() != p.arm_joints.size()) {
      fail("arm.dof does not match joints.arm");
    }
  }
  if (root["home"]) {
    for (const auto & kv : root["home"]) {
      p.home[kv.first.as<std::string>()] = kv.second.as<double>();
    }
  }
  for (const auto & j : p.joints()) {
    if (!p.home.count(j)) {fail("home pose misses joint " + j);}
  }
  return p;
}

/// Load by profile name ("ur10e_telescopic") or by path to a .yaml file.
inline Profile loadProfile(const std::string & name_or_path = kDefaultProfile)
{
  const bool is_path = name_or_path.find('/') != std::string::npos ||
    (name_or_path.size() > 5 && name_or_path.substr(name_or_path.size() - 5) == ".yaml");
  const std::string path = is_path ? name_or_path : profilesDir() + "/" + name_or_path + ".yaml";
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("cannot read profile " + path + ": " + e.what());
  }
  std::string stem = path.substr(path.find_last_of('/') == std::string::npos ? 0 :
      path.find_last_of('/') + 1);
  if (stem.size() > 5 && stem.substr(stem.size() - 5) == ".yaml") {stem.resize(stem.size() - 5);}
  return parseProfile(root, stem);
}

/// For nodes that only need names: the profile if it loads, else a Profile holding just the
/// built-in default group names (no joints, no frames; use frameOr) and `warning` set. A node
/// then keeps working where the description package is not installed.
inline Profile loadProfileOrDefaults(const std::string & name_or_path, std::string & warning)
{
  warning.clear();
  try {
    return loadProfile(name_or_path.empty() ? std::string(kDefaultProfile) : name_or_path);
  } catch (const std::exception & e) {
    warning = std::string("using built-in default group names: ") + e.what();
    Profile p;
    p.name = name_or_path;
    return p;
  }
}

}  // namespace mobile_manipulator_profile
