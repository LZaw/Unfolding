
#pragma once

// C++ Headers
#include <cstdlib>

// Project Headers
#include <Collisions/Solver/2D/CollisionSolver2DBase.hpp>

class RandomFaceSelectorBase : virtual public CollisionSolver2DBase{
public:
  RandomFaceSelectorBase():
    CollisionSolver2DBase(nullptr, nullptr, nullptr){
  }

  virtual ~RandomFaceSelectorBase() = default;

protected:
  virtual UnfoldTree* selectFace() const override{
    return m_unfoldTreeList->findTreeNodeByIndex(std::rand() % m_polygonList->size());
  }
};
