
#pragma once

// C++ Headers
#include <vector>

// Project Headers
#include <Datastructures/Mesh/Mesh.hpp>
#include <Datastructures/Unfolding/UnfoldTree.hpp>
#include <Datastructures/Unfolding/UnfoldTreeList.hpp>

// Eigen Headers
#include <Eigen/Dense>

class SteepestEdgeUnfolder{
public:
  SteepestEdgeUnfolder() = default;

  virtual UnfoldTree* createNewUnfoldTree(const UnfoldMesh* mesh, UnfoldTreeList& unfoldTreeList, const Eigen::VectorXd& weights);

protected:
  virtual std::vector<std::vector<unsigned int>> calculateCutEdges(const UnfoldMesh* mesh, const Eigen::Vector3d& steepestDirection);
  virtual int calculateRootIndex(const UnfoldMesh* mesh, const Eigen::VectorXd& weights);

  virtual Eigen::Vector3d generateSteepestDirection(const UnfoldMesh* mesh, const Eigen::VectorXd& weights);

  virtual void setUpTree(const UnfoldMesh* mesh, UnfoldTreeList& unfoldTreeList, const std::vector<std::vector<unsigned int>>& cutEdges);
};
