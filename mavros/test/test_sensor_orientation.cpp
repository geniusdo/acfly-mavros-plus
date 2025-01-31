// mavros
// Copyright 2021 Vladimir Ermakov, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//

/**
 * Test libmavros sensor orientation utilities
 *
 * We look up for some rotations, not fill list.
 */

#include <gtest/gtest.h>

#include "mavconn/mavlink_dialect.hpp"
#include "mavros/frame_tf.hpp"
#include "mavros/utils.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace mavros; // NOLINT
using mavros::utils::enum_value;
using SO = mavlink::common::MAV_SENSOR_ORIENTATION;

static const double epsilon = 1e-9;

#define EXPECT_QUATERNION(exp, res)                                            \
  EXPECT_NEAR(exp.w(), res.w(), epsilon);                                      \
  EXPECT_NEAR(exp.x(), res.x(), epsilon);                                      \
  EXPECT_NEAR(exp.y(), res.y(), epsilon);                                      \
  EXPECT_NEAR(exp.z(), res.z(), epsilon)

TEST(UTILS, sensor_orientation_matching__none) {
  auto expected = ftf::quaternion_from_rpy(0.0, 0.0, 0.0);
  auto out = utils::sensor_orientation_matching(SO::ROTATION_NONE);

  EXPECT_QUATERNION(expected, out);
}

TEST(UTILS, sensor_orientation_matching__roll_180) {
  auto expected = ftf::quaternion_from_rpy(M_PI, 0.0, 0.0);
  auto out = utils::sensor_orientation_matching(SO::ROTATION_ROLL_180);

  EXPECT_QUATERNION(expected, out);
}

TEST(UTILS, sensor_orientation_matching__roll_180_yaw_90) {
  auto expected = ftf::quaternion_from_rpy(M_PI, 0.0, M_PI / 2);
  auto out = utils::sensor_orientation_matching(SO::ROTATION_ROLL_180_YAW_90);

  EXPECT_QUATERNION(expected, out);
}

TEST(UTILS, sensor_orientation_matching__custom) {
  auto expected = ftf::quaternion_from_rpy(0.0, 0.0, 0.0);
  auto out = utils::sensor_orientation_matching(SO::ROTATION_CUSTOM);

  EXPECT_QUATERNION(expected, out);
}

TEST(UTILS, to_string__none) {
  EXPECT_EQ("NONE", utils::to_string(SO::ROTATION_NONE));
}

TEST(UTILS, to_string__roll_180) {
  EXPECT_EQ("ROLL_180", utils::to_string(SO::ROTATION_ROLL_180));
}

TEST(UTILS, to_string__roll_180_yaw_90) {
  EXPECT_EQ("ROLL_180_YAW_90", utils::to_string(SO::ROTATION_ROLL_180_YAW_90));
}

TEST(UTILS, to_string__custom) {
  EXPECT_EQ("CUSTOM", utils::to_string(SO::ROTATION_CUSTOM));
}

TEST(UTILS, sensor_orientation_from_str__none) {
  EXPECT_EQ(enum_value(SO::ROTATION_NONE),
            utils::sensor_orientation_from_str("NONE"));
}

TEST(UTILS, sensor_orientation_from_str__unknown) {
  EXPECT_LT(utils::sensor_orientation_from_str("completely wrong identifier"),
            0);
}

TEST(UTILS, sensor_orientation_from_str__number) {
  EXPECT_EQ(enum_value(SO::ROTATION_ROLL_270),
            utils::sensor_orientation_from_str("20"));
}

TEST(UTILS, sensor_orientation_from_str__wrong_number) {
  // 123 >> 38 (max)
  EXPECT_LT(utils::sensor_orientation_from_str("123"), 0);
}

TEST(UTILS, sensor_orientation_from_str__roll_180) {
  EXPECT_EQ(enum_value(SO::ROTATION_ROLL_180),
            utils::sensor_orientation_from_str("ROLL_180"));
}

TEST(UTILS, sensor_orientation_from_str__roll_180_yaw_90) {
  EXPECT_EQ(enum_value(SO::ROTATION_ROLL_180_YAW_90),
            utils::sensor_orientation_from_str("ROLL_180_YAW_90"));
}

TEST(UTILS, sensor_orientation_from_str__last_element_roll_90_yaw_270) {
  EXPECT_EQ(enum_value(SO::ROTATION_ROLL_90_YAW_270),
            utils::sensor_orientation_from_str("ROLL_90_YAW_270"));
}

TEST(UTILS, sensor_orientation_from_str__custom) {
  EXPECT_EQ(enum_value(SO::ROTATION_CUSTOM),
            utils::sensor_orientation_from_str("CUSTOM"));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
