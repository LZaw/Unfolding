
#pragma once

// C++ Headers
#include <cstdlib>
#include <vector>

// Project Headers
#include <Collisions/Solver/2D/CollisionSolver2DBase.hpp>

class RandomMovableCollidingFaceSelectorBase : virtual public CollisionSolver2DBase{
public:
  RandomMovableCollidingFaceSelectorBase():
    CollisionSolver2DBase(nullptr, nullptr, nullptr){
  }

  virtual ~RandomMovableCollidingFaceSelectorBase() = default;

protected:
  virtual UnfoldTree* selectFace() const override{
    std::vector<UnfoldTree*> movableTreeNodes = m_unfoldTreeList->calculateMovableNodes();
    const std::vector<std::vector<unsigned int>>& collisions = m_collisionDetector->getCollisions();
    std::vector<UnfoldTree*> collidingMovableTreeNodes;
    collidingMovableTreeNodes.reserve(movableTreeNodes.size());
    for(UnfoldTree* movableTreeNode: movableTreeNodes){
      if(!collisions[movableTreeNode->m_ref].empty()){
        collidingMovableTreeNodes.push_back(movableTreeNode);
      }
    }
    if(collidingMovableTreeNodes.empty()){
      return movableTreeNodes[std::rand() % movableTreeNodes.size()];
    }
    return collidingMovableTreeNodes[std::rand() % collidingMovableTreeNodes.size()];
  }
};
