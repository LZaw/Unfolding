
#include "RandomCollisionSolver.hpp"

// Project Headers
#include <Collisions/Solver/2D/CollisionSolver2DBase.hpp>

RandomCollisionSolver::RandomCollisionSolver(UnfoldTreeList* unfoldTreeList, PolygonList* polygonList, CollisionDetector* collisionDetector):
  CollisionSolver2DBase(unfoldTreeList, polygonList, collisionDetector){
}
