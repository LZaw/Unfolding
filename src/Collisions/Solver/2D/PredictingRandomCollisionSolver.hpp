
#pragma once

// Project Headers
#include <Collisions/Detector/CollisionDetector.hpp>
#include <Collisions/Solver/2D/RandomCollisionSolver.hpp>
#include <Datastructures/Polygon/PolygonList.hpp>
#include <Datastructures/Unfolding/UnfoldTree.hpp>
#include <Datastructures/Unfolding/UnfoldTreeList.hpp>

class PredictingRandomCollisionSolver: public RandomCollisionSolver{
public:
  PredictingRandomCollisionSolver(UnfoldTreeList* unfoldTreeList, PolygonList* polygonList, CollisionDetector* collisionDetector);

protected:
  int rateMove(UnfoldTree* collidingTreeNode, UnfoldTree* newParentNode);

  int selectNewParent(UnfoldTree*& collidingTreeNode);

  bool takeStep(UnfoldTree*& currentTreeNode) override;
};
