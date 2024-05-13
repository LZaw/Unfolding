
#include "CollisionSolverBase.hpp"

// C++ Headers

CollisionSolverBase::CollisionSolverBase(UnfoldTreeList* unfoldTreeList, PolygonList* polygonList, CollisionDetector* collisionDetector):
  m_collisionDetector(collisionDetector),
  m_interruptSolving(false),
  m_polygonList(polygonList),
  m_unfoldTreeList(unfoldTreeList){
}

// Public

void CollisionSolverBase::interruptSolve(){
  m_interruptSolving = true;
}

void CollisionSolverBase::setUnfoldTreeList(UnfoldTreeList* unfoldTreeList){
  m_unfoldTreeList = unfoldTreeList;
}

// Protected

bool CollisionSolverBase::evaluate(){
  m_collisionDetector->detectAllCollisions();
  if(m_collisionDetector->getNumberOfCollisions() == 0){
    return true;
  }
  return false;
}

unsigned long CollisionSolverBase::getRuntime() const{
  return (std::clock() - m_startTime) / (CLOCKS_PER_SEC);
}
