
#pragma once

// C++ Headers
#include <vector>

// Project Headers
#include <Collisions/Solver/2D/CollisionSolver2DBase.hpp>

class RandomParentStepperBase : virtual public CollisionSolver2DBase{
public:
  RandomParentStepperBase():
    CollisionSolver2DBase(nullptr, nullptr, nullptr){
  }

  virtual ~RandomParentStepperBase() = default;

protected:
  bool takeStep(UnfoldTree*& currentTreeNode) override{
    std::vector<unsigned int> possibleNeighbors = currentTreeNode->calculatePossibleNeighbors();
    if(possibleNeighbors.empty() ||
       m_interruptSolving){
      return false;
    }
    int newParentIndex = possibleNeighbors[std::rand() % possibleNeighbors.size()];
    UnfoldTree* newParent = m_unfoldTreeList->findTreeNodeByIndex(newParentIndex);
    currentTreeNode->attachToNewParent(newParent);
    m_polygonList->generateSubTreePolygons(currentTreeNode);
    return true;
  }
};
