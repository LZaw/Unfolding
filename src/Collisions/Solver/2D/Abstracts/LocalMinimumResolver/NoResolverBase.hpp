
#pragma once

// Project Headers
#include <Collisions/Solver/2D/CollisionSolver2DBase.hpp>

class NoResolverBase : virtual public CollisionSolver2DBase{
public:
  NoResolverBase():
    CollisionSolver2DBase(nullptr, nullptr, nullptr){
  }

  virtual ~NoResolverBase() = default;

protected:
  bool determineAndResolvePossibleLocalMinima() override{
    return false;
  }
};
