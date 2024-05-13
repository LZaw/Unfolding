
#pragma once

// Project Headers
#include <Collisions/Solver/2D/CollisionSolver2DBase.hpp>

class NoConstraintsBase : virtual public CollisionSolver2DBase{
public:
  NoConstraintsBase():
    CollisionSolver2DBase(nullptr, nullptr, nullptr){
  }

  virtual ~NoConstraintsBase() = default;

protected:
  virtual UnfoldTree* applyConstraints(UnfoldTree* currentTreeNode) override{
    return currentTreeNode;
  }
};
