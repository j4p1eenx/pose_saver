#include <gtest/gtest.h>
#include <limits>
#include "pose_saver_pkg/cloud_db_client.hpp"
#include "pose_saver_pkg/pose_logic.hpp"

TEST(Record, JsonRoundTrip) {
  PoseRecord a;
  a.robot_id = "r1"; a.map_id = "m"; a.x = 1.5; a.y = -2.0; a.qz = 0.0; a.qw = 1.0; a.stamp_unix = 1e9;
  PoseRecord b;
  ASSERT_TRUE(CloudDBClient::fromJson(CloudDBClient::toJson(a), b));
  EXPECT_DOUBLE_EQ(b.x, 1.5);
  EXPECT_EQ(b.map_id, "m");
  EXPECT_DOUBLE_EQ(b.stamp_unix, 1e9);
}

TEST(Record, MissingRequiredFieldIsRejected) {
  PoseRecord r;
  EXPECT_FALSE(CloudDBClient::fromJson(nlohmann::json{{"y", 1.0}, {"qw", 1.0}}, r));
  EXPECT_FALSE(CloudDBClient::fromJson(nlohmann::json("error string"), r));
}

TEST(Record, Validity) {
  PoseRecord r; r.stamp_unix = 1.0;
  EXPECT_TRUE(isValidPose(r));
  r.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(isValidPose(r));
  r.x = 0; r.qw = 0;  // zero quaternion
  EXPECT_FALSE(isValidPose(r));
}

// A 100x100 map (0.05 m, origin -2.5,-2.5) with a vertical wall at x = 1.0 m.
static nav_msgs::msg::OccupancyGrid wallMap() {
  nav_msgs::msg::OccupancyGrid m;
  m.info.resolution = 0.05f; m.info.width = 100; m.info.height = 100;
  m.info.origin.position.x = -2.5; m.info.origin.position.y = -2.5;
  m.data.assign(100 * 100, 0);
  for (int y = 0; y < 100; ++y) m.data[y * 100 + 70] = 100;
  return m;
}
static sensor_msgs::msg::LaserScan scanFacingWall() {  // robot at (0,0) facing +x
  sensor_msgs::msg::LaserScan s;
  s.angle_min = -0.5f; s.angle_increment = 0.01f; s.range_min = 0.1f; s.range_max = 3.5f;
  for (int i = 0; i < 101; ++i)
    s.ranges.push_back(static_cast<float>(1.0 / std::cos(-0.5 + 0.01 * i)));
  return s;
}

TEST(ScanMatch, CorrectPoseScoresHigh) {
  EXPECT_GT(scanMatchScore(wallMap(), scanFacingWall(), 0, 0, 0, 0, 0, 0, 1, 2, 20), 0.9);
}
TEST(ScanMatch, MovedPoseScoresLow) {
  EXPECT_LT(scanMatchScore(wallMap(), scanFacingWall(), 0.6, 0, 0, 0, 0, 0, 1, 2, 20), 0.2);
  EXPECT_LT(scanMatchScore(wallMap(), scanFacingWall(), 0, 0, 1.57, 0, 0, 0, 1, 2, 20), 0.2);
}
TEST(ScanMatch, TooFewBeamsIsUnknown) {
  sensor_msgs::msg::LaserScan s = scanFacingWall();
  s.ranges.resize(5);
  EXPECT_LT(scanMatchScore(wallMap(), s, 0, 0, 0, 0, 0, 0, 1, 2, 20), 0.0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}