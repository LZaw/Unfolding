
#pragma once

// C++ Headers
#include <vector>

// Project Headers

// Eigen Headers
#include <Eigen/Dense>

namespace PolygonUtil{
  bool lineLineIntersection(const Eigen::Vector2d& l10, const Eigen::Vector2d& l11, const Eigen::Vector2d& l20, const Eigen::Vector2d& l21);
}
