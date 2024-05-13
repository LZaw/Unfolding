
#pragma once

// C++ Headers
#include <ctime>

// Project Headers
#include <Collisions/Detector/CollisionDetector.hpp>
#include <Datastructures/Polygon/PolygonList.hpp>
#include <Datastructures/Unfolding/UnfoldTreeList.hpp>

class CollisionSolverBase{
public:
  CollisionSolverBase(UnfoldTreeList* unfoldTreeList, PolygonList* polygonList, CollisionDetector* collisionDetector);

  void interruptSolve();

  void setUnfoldTreeList(UnfoldTreeList* unfoldTreeList);
  virtual int solve(const unsigned long seconds = 0) = 0;

protected:
  CollisionDetector*  m_collisionDetector;
  bool                m_interruptSolving;
  PolygonList*        m_polygonList;
  std::clock_t        m_startTime;
  UnfoldTreeList*     m_unfoldTreeList;

  bool evaluate();

  unsigned long getRuntime() const;
};
