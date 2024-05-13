
#pragma once

// Project Headers
#include <Collisions/Solver/CollisionSolverBase.hpp>
#include <Datastructures/Polygon/PolygonList.hpp>
#include <Datastructures/Unfolding/UnfoldTree.hpp>
#include <Datastructures/Unfolding/UnfoldTreeList.hpp>

class CollisionSolver2DBase : public CollisionSolverBase{
public:
  CollisionSolver2DBase(UnfoldTreeList* unfoldTreeList, PolygonList* polygonList, CollisionDetector* collisionDetector);
  virtual ~CollisionSolver2DBase() = default;

  virtual void resetLocalVariables();

  int solve(const unsigned long seconds = 0) override;

protected:
  unsigned int m_iterationsDone = 0;
  unsigned int m_rejectedSteps = 0;

  virtual UnfoldTree* applyConstraints(UnfoldTree* currentTreeNode) = 0;

  virtual bool determineAndResolvePossibleLocalMinima() = 0;

  bool evaluate(UnfoldTree* currentTreeNode);

  virtual UnfoldTree* selectFace() const = 0;

  virtual bool takeStep(UnfoldTree*& currentTreeNode) = 0;
};
