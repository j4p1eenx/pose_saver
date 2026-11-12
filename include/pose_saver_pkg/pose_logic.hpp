#pragma once
#include <algorithm>
#include <cmath>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "pose_saver_pkg/cloud_db_client.hpp"

inline double yawFromQuat(double x, double y, double z, double w) {
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}
inline double normalizeAngle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

inline bool isValidPose(const PoseRecord& r) {
  const double v[] = {r.x, r.y, r.z, r.qx, r.qy, r.qz, r.qw, r.stamp_unix};
  for (double d : v) if (!std::isfinite(d)) return false;
  const double n = std::sqrt(r.qx * r.qx + r.qy * r.qy + r.qz * r.qz + r.qw * r.qw);
  return n > 0.9 && n < 1.1;
}

/**
 * Fraction of valid lidar endpoints that fall on (or within `nbr` cells of) an
 * occupied map cell, if the base frame is at (bx,by,byaw) in the map frame.
 * (sx,sy,syaw) is the scan frame expressed in the base frame.
 * Returns -1 if there are fewer than `min_beams` valid beams (cannot judge).
 * Assumes the map origin has zero yaw (true for map_server maps from SLAM).
 */
inline double scanMatchScore(const nav_msgs::msg::OccupancyGrid& map,
                             const sensor_msgs::msg::LaserScan& scan,
                             double bx, double by, double byaw,
                             double sx, double sy, double syaw,
                             int stride, int nbr, int min_beams) {
  const double res = map.info.resolution;
  const int W = static_cast<int>(map.info.width), H = static_cast<int>(map.info.height);
  if (res <= 0.0 || W == 0 || H == 0 || map.data.size() < static_cast<size_t>(W) * H) return -1.0;
  const double ox = map.info.origin.position.x, oy = map.info.origin.position.y;
  const double cb = std::cos(byaw), sb = std::sin(byaw);
  const double cs = std::cos(syaw), ss = std::sin(syaw);
  const size_t step = static_cast<size_t>(std::max(1, stride));

  int valid = 0, hits = 0;
  for (size_t i = 0; i < scan.ranges.size(); i += step) {
    const double r = scan.ranges[i];
    if (!std::isfinite(r) || r < scan.range_min || r >= 0.98 * scan.range_max) continue;
    const double a = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    const double px = r * std::cos(a), py = r * std::sin(a);            // scan frame
    const double qx = sx + cs * px - ss * py, qy = sy + ss * px + cs * py;  // base frame
    const double mx = bx + cb * qx - sb * qy, my = by + sb * qx + cb * qy;  // map frame
    const int cx = static_cast<int>(std::floor((mx - ox) / res));
    const int cy = static_cast<int>(std::floor((my - oy) / res));
    ++valid;
    bool hit = false;
    for (int dy = -nbr; dy <= nbr && !hit; ++dy)
      for (int dx = -nbr; dx <= nbr; ++dx) {
        const int x = cx + dx, y = cy + dy;
        if (x < 0 || y < 0 || x >= W || y >= H) continue;
        if (map.data[static_cast<size_t>(y) * W + x] >= 65) { hit = true; break; }
      }
    if (hit) ++hits;
  }
  return valid < min_beams ? -1.0 : static_cast<double>(hits) / valid;
}