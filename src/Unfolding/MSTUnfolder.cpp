
#include "MSTUnfolder.hpp"

// C++ Headers
#include <iostream>
#include <queue>

// Project Headers
#include <Datastructures/Mesh/DualGraph.hpp>
#include <Datastructures/Mesh/DualGraphNode.hpp>
#include <Unfolding/SteepestEdgeUnfolder.hpp>

// Public

UnfoldTree* MSTUnfolder::createNewUnfoldTree(const UnfoldMesh* mesh, UnfoldTreeList& unfoldTreeList, const Eigen::VectorXd& weights){
  if(static_cast<size_t>(weights.rows()) != mesh->getFaces().size()){
    std::cerr << "Weights have the wrong size." << std::endl;
    return nullptr;
  }

  // Create Tree via MSP Calculation
  setUpTree(mesh, unfoldTreeList, weights);

  return unfoldTreeList.getRoot();
}

// Protected

int MSTUnfolder::calculateRootIndex(const Eigen::VectorXd& weights){
  int rootIndex;
  weights.minCoeff(&rootIndex);
  return rootIndex;
}

void MSTUnfolder::setUpTree(const UnfoldMesh* mesh, UnfoldTreeList& unfoldTreeList, const Eigen::VectorXd& weights){
  std::vector<bool> inserted(mesh->getFaces().size(), false);

  // Set Root Node
  int rootIndex = calculateRootIndex(weights);
  unfoldTreeList.setRootIndex(rootIndex);
  unfoldTreeList.getRoot()->createNode(rootIndex, nullptr, UnfoldTransformation(rootIndex, -1));
  inserted[unfoldTreeList.getRootIndex()] = true;

  // Weight, Parent, Child
  // Priority queue returning the smallest entry first
  std::priority_queue<std::tuple<double, unsigned int, unsigned int>,
                      std::vector<std::tuple<double, unsigned int, unsigned int>>,
                      std::less<std::tuple<double, unsigned int, unsigned int>>> edgeWeightQueue;
  updateQueue(edgeWeightQueue, unfoldTreeList.getRootIndex(), mesh, weights);

  // Prim
  while(!edgeWeightQueue.empty()){
    const std::tuple<double, unsigned int, unsigned int> edgeWeight = edgeWeightQueue.top();
    edgeWeightQueue.pop();
    if(!inserted[std::get<2>(edgeWeight)]){
      unsigned int parentIndex = std::get<1>(edgeWeight);
      unsigned int childIndex = std::get<2>(edgeWeight);
      UnfoldTree* newChild = unfoldTreeList[childIndex].get();
      UnfoldTree* parent = unfoldTreeList[parentIndex].get();
      parent->m_children.push_back(newChild);
      newChild->createNode(childIndex, parent, mesh->getDualGraph().findUnfoldTransformation(childIndex, parentIndex));
      updateQueue(edgeWeightQueue, childIndex, mesh, weights);
      inserted[childIndex] = true;
    }
  }
}
