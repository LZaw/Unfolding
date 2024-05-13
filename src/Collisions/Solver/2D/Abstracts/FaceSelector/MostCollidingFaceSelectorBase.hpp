
#pragma once

// C++ Headers
#include <iterator>
#include <vector>

// Project Headers
#include <Collisions/Solver/2D/CollisionSolver2DBase.hpp>

class MostCollidingFaceSelectorBase : virtual public CollisionSolver2DBase{
public:
  MostCollidingFaceSelectorBase():
    CollisionSolver2DBase(nullptr, nullptr, nullptr){
  }

  virtual ~MostCollidingFaceSelectorBase() = default;

protected:
  virtual UnfoldTree* selectFace() const override{
    const std::vector<std::vector<unsigned int>>& collisions = m_collisionDetector->getCollisions();
    int collidingFaceIndex;
    unsigned int numCollisions = 0;
    for(std::vector<std::vector<unsigned int>>::const_iterator it = collisions.cbegin(); it != collisions.cend() && !m_interruptSolving; ++it){
      if((*it).size() > numCollisions){
        collidingFaceIndex = std::distance(collisions.cbegin(), it);
        numCollisions = (*it).size();
      }
    }
    if(numCollisions == 0){
      return nullptr;
    }
    return m_unfoldTreeList->findTreeNodeByIndex(collidingFaceIndex);
  }
};
