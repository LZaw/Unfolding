
#pragma once

// C++ Headers
#include <functional>
#include <queue>
#include <tuple>
#include <vector>

// Project Headers
#include <Datastructures/Mesh/UnfoldMesh.hpp>
#include <Datastructures/Unfolding/UnfoldTree.hpp>
#include <Datastructures/Unfolding/UnfoldTreeList.hpp>

// Eigen Headers
#include <Eigen/Dense>

class MSTUnfolder{
public:
  virtual UnfoldTree* createNewUnfoldTree(const UnfoldMesh* mesh, UnfoldTreeList& unfoldTreeList, const Eigen::VectorXd& weights);

protected:
  virtual int calculateRootIndex(const Eigen::VectorXd& weights);

  virtual void setUpTree(const UnfoldMesh* mesh, UnfoldTreeList& unfoldTreeList, const Eigen::VectorXd& weights);

  virtual void updateQueue(std::priority_queue<std::tuple<double, unsigned int, unsigned int>, std::vector<std::tuple<double, unsigned int, unsigned int>>, std::less<std::tuple<double, unsigned int, unsigned int>>>& priorityQueue, unsigned int nodeIndex, const UnfoldMesh* mesh, const Eigen::VectorXd& weights) = 0;
};
