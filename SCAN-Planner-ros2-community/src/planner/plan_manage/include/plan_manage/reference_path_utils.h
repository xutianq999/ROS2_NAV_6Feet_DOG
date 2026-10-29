#pragma once
#include <Eigen/Core>
#include <vector>

namespace scan_planner
{
inline bool distinctReferencePoints(const Eigen::Vector3d & start,
  const std::vector<Eigen::Vector3d> & waypoints, std::vector<Eigen::Vector3d> & points)
{
  points.clear();
  if (!start.allFinite()) return false;
  points.push_back(start);
  for (const auto & point : waypoints) {
    if (!point.allFinite()) return false;
    // Zero-length polynomial segments lead to singular matrices and NaNs.
    if ((point - points.back()).norm() >= 0.01) points.push_back(point);
  }
  return points.size() >= 2;
}
}
