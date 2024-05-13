
#pragma once

// C++ Headers
#include <limits>
#include <tuple>
#include <vector>

// Eigen Headers
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace DDGUtil{
  std::vector<double> calculateCotHalfs(const std::vector<Eigen::Vector3d>& edges);
  std::vector<Eigen::Vector3d> calculateEdges(const Eigen::MatrixXd& vertices, const std::vector<unsigned int>& currentFace);
  std::pair<Eigen::VectorXd, double> calculateLargestEigenpair(const Eigen::SparseMatrix<double>& M, double precision = std::numeric_limits<float>::epsilon(), size_t maxIterations = std::numeric_limits<size_t>::max());
}
