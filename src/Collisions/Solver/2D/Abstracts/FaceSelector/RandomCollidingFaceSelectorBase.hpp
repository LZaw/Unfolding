
#pragma once

// C++ Headers
#include <cstdlib>
#include <iterator>
#include <vector>

// Project Headers
#include <Collisions/Solver/2D/CollisionSolver2DBase.hpp>

class RandomCollidingFaceSelectorBase : virtual public CollisionSolver2DBase{
public:
  RandomCollidingFaceSelectorBase():
    CollisionSolver2DBase(nullptr, nullptr, nullptr){
  }

  virtual ~RandomCollidingFaceSelectorBase() = default;

protected:
  virtual UnfoldTree* selectFace() const override{
    const std::vector<std::vector<unsigned int>>& collisions = m_collisionDetector->getCollisions();
    std::vector<unsigned int> collidingFaces;
    collidingFaces.reserve(collisions.size());
    for(std::vector<std::vector<unsigned int>>::const_iterator collision = collisions.cbegin(); collision != collisions.cend(); ++collision){
      if(!(*collision).empty()){
        collidingFaces.push_back(std::distance(collisions.cbegin(), collision));
      }
    }
    if(collidingFaces.empty()){
      return nullptr;
    }
    int collidingFaceIndex = collidingFaces[std::rand() % collidingFaces.size()];
    return m_unfoldTreeList->findTreeNodeByIndex(collidingFaceIndex);
  }
};
