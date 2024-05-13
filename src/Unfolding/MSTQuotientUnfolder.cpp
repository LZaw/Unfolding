
#include "MSTQuotientUnfolder.hpp"

// Protected

// Calculates w_j / w_i for edge i,j
void MSTQuotientUnfolder::updateQueue(std::priority_queue<std::tuple<double, unsigned int, unsigned int>, std::vector<std::tuple<double, unsigned int, unsigned int>>, std::less<std::tuple<double, unsigned int, unsigned int>>>& priorityQueue, unsigned int nodeIndex, const UnfoldMesh* mesh, const Eigen::VectorXd& weights){
  const std::vector<DualGraphNode>& neighbors = mesh->getDualGraph()[nodeIndex];
  for(std::vector<DualGraphNode>::const_iterator nodeIterator = neighbors.cbegin(); nodeIterator != neighbors.cend(); ++nodeIterator){
    // Den Knoten mit der kleinsten relativen Änderung: weights[neighbors[j].getRef()] / weights[nodeIndex];
    double weight = weights[nodeIterator->getRef()] / weights[nodeIndex];
    priorityQueue.push({weight, nodeIndex, nodeIterator->getRef()});
  }
}
