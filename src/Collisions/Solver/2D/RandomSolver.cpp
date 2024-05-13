
#include "RandomSolver.hpp"

// Project Headers
#include <Collisions/Solver/2D/CollisionSolver2DBase.hpp>

RandomSolver::RandomSolver(UnfoldTreeList* unfoldTreeList, PolygonList* polygonList, CollisionDetector* collisionDetector):
  CollisionSolver2DBase(unfoldTreeList, polygonList, collisionDetector){
}
