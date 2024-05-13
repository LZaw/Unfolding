
#include "CollisionSolver2DBase.hpp"

// C++ Headers
#include <ctime>
#include <iostream>

CollisionSolver2DBase::CollisionSolver2DBase(UnfoldTreeList* unfoldTreeList, PolygonList* polygonList, CollisionDetector* collisionDetector):
  CollisionSolverBase(unfoldTreeList, polygonList, collisionDetector){
}

// Public

void CollisionSolver2DBase::resetLocalVariables(){
}

int CollisionSolver2DBase::solve(const unsigned long seconds){
  std::cout << "Number of collisions: " << m_collisionDetector->getNumberOfCollisions() << std::endl;
  m_interruptSolving = false;
  m_startTime = std::clock();
  m_iterationsDone = 0;
  m_rejectedSteps = 0;

  while(getRuntime() < seconds &&
        !m_interruptSolving &&
        m_collisionDetector->getNumberOfCollisions() != 0){
    if(!determineAndResolvePossibleLocalMinima()){
      std::cout << "Iterations done: " << m_iterationsDone << std::endl;
      return -1;
    }
    // Select Face
    UnfoldTree* currentTreeNode = selectFace();
    if(currentTreeNode == nullptr){
      ++m_rejectedSteps;
      ++m_iterationsDone;
      continue;
    }
    // Root can't be moved. Doesn't count as failure.
    if(currentTreeNode->m_parent == nullptr){
      continue;
    }

    // Apply Constraints
    currentTreeNode = applyConstraints(currentTreeNode);

    // Root can't be moved. Doesn't count as failure.
    if(currentTreeNode->m_parent == nullptr){
      continue;
    }

    // Take Step
    bool success = takeStep(currentTreeNode);
    if(success){
      m_rejectedSteps = 0;
      if(evaluate(currentTreeNode)){
        std::cout << "Iterations done: " << m_iterationsDone << std::endl;
        return 0;
      }
    }else{
      ++m_rejectedSteps;
    }
    ++m_iterationsDone;
  }
  std::cout << "Iterations done: " << m_iterationsDone << std::endl;
  if(m_collisionDetector->getNumberOfCollisions() == 0){
    return 0;
  }
  return -1;
}

// Protected

bool CollisionSolver2DBase::evaluate(UnfoldTree* currentTreeNode){
  m_collisionDetector->detectCollisionsOnSubTree(currentTreeNode);
  if(m_collisionDetector->getNumberOfCollisions() == 0){
    return true;
  }
  return false;
}
