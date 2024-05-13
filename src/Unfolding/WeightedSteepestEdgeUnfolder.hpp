
#pragma once

// Project Headers
#include <Unfolding/SteepestEdgeUnfolder.hpp>

class WeightedSteepestEdgeUnfolder : public SteepestEdgeUnfolder{
public:
  using SteepestEdgeUnfolder::SteepestEdgeUnfolder;

protected:
  int calculateRootIndex(const UnfoldMesh* mesh, const Eigen::VectorXd& weights) override;

  Eigen::Vector3d generateSteepestDirection(const UnfoldMesh* mesh, const Eigen::VectorXd& weights) override;
};
