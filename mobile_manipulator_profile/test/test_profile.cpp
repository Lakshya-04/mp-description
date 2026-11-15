#include <cstdlib>
#include <filesystem>

#include "gtest/gtest.h"
#include "mobile_manipulator_profile/profile.hpp"

using mobile_manipulator_profile::loadProfile;

class ProfileTest : public ::testing::Test
{
protected:
  void SetUp() override {setenv("MOBILE_MANIPULATOR_PROFILE_DIR", PROFILE_DIR, 1);}
};

TEST_F(ProfileTest, Ur10eHasSixArmJoints)
{
  const auto p = loadProfile("ur10e_telescopic");
  EXPECT_EQ(p.arm, "ur10e");
  EXPECT_EQ(p.armDof(), 6u);
  EXPECT_EQ(p.dof(), 7u);
  EXPECT_EQ(p.joints().front(), "lift_joint");
  EXPECT_EQ(p.joints()[1], "arm_shoulder_pan_joint");
  EXPECT_EQ(p.whole_body_group, "lift_arm");
  EXPECT_EQ(p.arm_group, "arm");
  EXPECT_EQ(p.tcpLink(), "tcp_link");
  EXPECT_EQ(p.homePositions().size(), 7u);
}

TEST_F(ProfileTest, PandaHasSevenArmJoints)
{
  const auto p = loadProfile("panda_telescopic");
  EXPECT_EQ(p.armDof(), 7u);
  EXPECT_EQ(p.dof(), 8u);
  EXPECT_EQ(p.joints().back(), "arm_joint7");
  EXPECT_EQ(p.controller_family, "panda");
}

TEST_F(ProfileTest, GenericArmLoadsByPath)
{
  const auto p = loadProfile(std::string(PROFILE_DIR) + "/generic_6dof_telescopic.yaml");
  EXPECT_EQ(p.name, "generic_6dof_telescopic");
  EXPECT_EQ(p.armDof(), 6u);
}

TEST_F(ProfileTest, MissingProfileThrows)
{
  EXPECT_THROW(loadProfile("no_such_profile"), std::runtime_error);
}

TEST_F(ProfileTest, DofMismatchThrows)
{
  auto root = YAML::LoadFile(std::string(PROFILE_DIR) + "/panda_telescopic.yaml");
  root["arm"]["dof"] = 6;
  EXPECT_THROW(mobile_manipulator_profile::parseProfile(root, "bad"), std::runtime_error);
}

TEST_F(ProfileTest, HomeMustCoverEveryJoint)
{
  auto root = YAML::LoadFile(std::string(PROFILE_DIR) + "/panda_telescopic.yaml");
  root["home"].remove("arm_joint7");
  EXPECT_THROW(mobile_manipulator_profile::parseProfile(root, "bad"), std::runtime_error);
}

TEST_F(ProfileTest, NoLiftProfileHasNoLiftJoint)
{
  const auto p = loadProfile("ur10e_nolift");
  EXPECT_TRUE(p.lift_joint.empty());
  EXPECT_EQ(p.dof(), p.armDof());
  EXPECT_EQ(p.homePositions().size(), p.dof());
}

TEST_F(ProfileTest, NullMimicIsEmpty)
{
  auto root = YAML::LoadFile(std::string(PROFILE_DIR) + "/panda_telescopic.yaml");
  root["joints"]["mimic"] = YAML::Node(YAML::NodeType::Null);
  EXPECT_TRUE(mobile_manipulator_profile::parseProfile(root, "p").mimic_joints.empty());
}

TEST_F(ProfileTest, EveryShippedProfileLoads)
{
  for (const auto & e : std::filesystem::directory_iterator(PROFILE_DIR)) {
    if (e.path().extension() != ".yaml") {continue;}
    EXPECT_NO_THROW(loadProfile(e.path().string())) << e.path();
  }
}
