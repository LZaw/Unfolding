
#include "MSTDiffUnfolder.hpp"

// Protected

// Calculates w_i - w_j for edge i,j
void MSTDiffUnfolder::updateQueue(std::priority_queue<std::tuple<double, unsigned int, unsigned int>, std::vector<std::tuple<double, unsigned int, unsigned int>>, std::less<std::tuple<double, unsigned int, unsigned int>>>& priorityQueue, unsigned int nodeIndex, const UnfoldMesh* mesh, const Eigen::VectorXd& weights){
  const std::vector<DualGraphNode>& neighbors = mesh->getDualGraph()[nodeIndex];
  for(std::vector<DualGraphNode>::const_iterator nodeIterator = neighbors.cbegin(); nodeIterator != neighbors.cend(); ++nodeIterator){
    // So schnell es geht schlechter werden: weights[nodeIndex] - weights[neighbors[j].getRef()];
    double weight = weights[nodeIndex] - weights[nodeIterator->getRef()];
    priorityQueue.push({weight, nodeIndex, nodeIterator->getRef()});
  }
}
