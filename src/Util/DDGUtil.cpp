
#include "DDGUtil.hpp"

// C++ Headers
#include <cmath>
#include <iostream>

std::vector<double> DDGUtil::calculateCotHalfs(const std::vector<Eigen::Vector3d>& edges){
  std::vector<double> cotHalfs(3);
  for(size_t i = 0; i < edges.size(); ++i){
    int i0 = (i + 0) % 3;
    int i2 = (i + 2) % 3;
    const Eigen::Vector3d& e0 = edges[i0];
    const Eigen::Vector3d& e1 = -edges[i2];
    cotHalfs[i] = e0.dot(e1) / (2. * e0.cross(e1).norm());
  }
  return cotHalfs;
}

std::vector<Eigen::Vector3d> DDGUtil::calculateEdges(const Eigen::MatrixXd& vertices, const std::vector<unsigned int>& currentFace){
  std::vector<Eigen::Vector3d> edges(currentFace.size());
  for(size_t i = 0; i < currentFace.size(); ++i){
    edges[i] = vertices.row(currentFace[(i + 1) % currentFace.size()]) - vertices.row(currentFace[i]);
  }
  return edges;
}

std::pair<Eigen::VectorXd, double> DDGUtil::calculateLargestEigenpair(const Eigen::SparseMatrix<double>& M,
                                                                      double precision,
                                                                      size_t maxIterations){
  Eigen::VectorXd tmp(M.cols());
  Eigen::VectorXd b(M.cols());
  tmp.setZero();
  b.setRandom().normalize();
  // Power Method
  double absDotProduct = 0.;
  size_t iterations = 0;
  while(1. - absDotProduct > precision && iterations < maxIterations){
    tmp = M * b;
    tmp.normalize();
    absDotProduct = std::fabs(tmp.dot(b));
    b = tmp;
    ++iterations;
  }
  return {b, b.dot(M * b) / b.dot(b)};
}
