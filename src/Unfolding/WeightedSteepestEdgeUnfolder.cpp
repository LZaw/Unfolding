
#include "WeightedSteepestEdgeUnfolder.hpp"

// C++ Headers
#include <cmath>
#include <limits>

// Protected

int WeightedSteepestEdgeUnfolder::calculateRootIndex(const UnfoldMesh* mesh, const Eigen::VectorXd& weights){
  if(weights.sum() < std::numeric_limits<float>::epsilon()){
    return SteepestEdgeUnfolder::calculateRootIndex(mesh, weights);
  }
  int rootIndex;
  weights.minCoeff(&rootIndex);
  return rootIndex;
}

Eigen::Vector3d WeightedSteepestEdgeUnfolder::generateSteepestDirection(const UnfoldMesh* mesh, const Eigen::VectorXd& weights){
  if(std::fabs(weights.sum()) < std::numeric_limits<float>::epsilon()){
    return SteepestEdgeUnfolder::generateSteepestDirection(mesh, weights);
  }
  const Eigen::MatrixXd& faceNormals = mesh->getFaceNormals();
  // Weighted mean of FaceNormals
  Eigen::Vector3d direction((faceNormals.array().colwise() * weights.array()).colwise().sum());
  return -direction.normalized();
}
