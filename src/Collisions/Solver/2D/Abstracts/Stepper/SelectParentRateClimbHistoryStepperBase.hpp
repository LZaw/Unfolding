
#pragma once

// C++ Headers
#include <utility>
#include <vector>

// Project Headers
#include <Collisions/Solver/2D/CollisionSolver2DBase.hpp>

class SelectParentRateClimbHistoryStepperBase : virtual public CollisionSolver2DBase{
public:
  SelectParentRateClimbHistoryStepperBase():
    CollisionSolver2DBase(nullptr, nullptr, nullptr){
  }

  virtual ~SelectParentRateClimbHistoryStepperBase() = default;

protected:
  std::vector<std::pair<int, int>> m_lastMoves;

  virtual bool acceptStep(UnfoldTree* currentTreeNode, UnfoldTree* newParent) = 0;

  virtual UnfoldTree* climb(UnfoldTree* currentTreeNode) = 0;

  virtual UnfoldTree* selectBestParent(UnfoldTree*& currentTreeNode, const std::vector<unsigned int>& possibleNeighbors) = 0;
  virtual std::vector<unsigned int> setupNeighborList(UnfoldTree* currentTreeNode) = 0;

  bool takeStep(UnfoldTree*& currentTreeNode) override{
    while(currentTreeNode->m_parent != nullptr){
      std::vector<unsigned int> possibleNeighbors = setupNeighborList(currentTreeNode);
      UnfoldTree* newParent = selectBestParent(currentTreeNode, possibleNeighbors);
      if(newParent == nullptr){
        currentTreeNode = climb(currentTreeNode);
        continue;
      }
      if(acceptStep(currentTreeNode, newParent)){
        updateHistory(currentTreeNode, newParent);
        currentTreeNode->attachToNewParent(newParent);
        m_polygonList->generateSubTreePolygons(currentTreeNode);
        return true;
      }
      currentTreeNode = climb(currentTreeNode);
    }
    return false;
  }

  virtual void updateHistory(const UnfoldTree* currentTreeNode, const UnfoldTree* newParent) = 0;
};
