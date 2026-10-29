#include <gtest/gtest.h>
#include <limits>
#include <plan_manage/reference_path_utils.h>

TEST(ReferencePath, RemovesZeroLengthSegments)
{
  std::vector<Eigen::Vector3d> result;
  const Eigen::Vector3d start(0, 0, 0.37);
  const std::vector<Eigen::Vector3d> points{
    start, start, Eigen::Vector3d(1, 0, 0.37), Eigen::Vector3d(1, 0, 0.37),
    Eigen::Vector3d(2, 0, 0.37)};
  ASSERT_TRUE(scan_planner::distinctReferencePoints(start, points, result));
  ASSERT_EQ(result.size(), 3u);
  EXPECT_DOUBLE_EQ(result.back().x(), 2.0);
}

TEST(ReferencePath, RejectsInvalidOrStationaryPath)
{
  std::vector<Eigen::Vector3d> result;
  const Eigen::Vector3d start(0, 0, 0);
  EXPECT_FALSE(scan_planner::distinctReferencePoints(start, {start}, result));
  EXPECT_FALSE(scan_planner::distinctReferencePoints(start,
    {Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0, 0)}, result));
}
