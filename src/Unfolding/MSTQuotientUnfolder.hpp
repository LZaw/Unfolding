
#pragma once

// C++ Headers
#include <functional>
#include <queue>
#include <tuple>
#include <vector>

// Project Headers
#include <Unfolding/MSTUnfolder.hpp>

// Eigen Headers
#include <Eigen/Dense>

class MSTQuotientUnfolder: public MSTUnfolder{
public:
  using MSTUnfolder::MSTUnfolder;

protected:
  virtual void updateQueue(std::priority_queue<std::tuple<double, unsigned int, unsigned int>, std::vector<std::tuple<double, unsigned int, unsigned int>>, std::less<std::tuple<double, unsigned int, unsigned int>>>& priorityQueue, unsigned int nodeIndex, const UnfoldMesh* mesh, const Eigen::VectorXd& weights) override;
};
