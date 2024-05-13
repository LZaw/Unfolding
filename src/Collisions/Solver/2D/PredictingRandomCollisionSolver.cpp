
#include "PredictingRandomCollisionSolver.hpp"

// C++ Headers
#include <algorithm>
#include <cstdlib>
#include <vector>

PredictingRandomCollisionSolver::PredictingRandomCollisionSolver(UnfoldTreeList* unfoldTreeList, PolygonList* polygonList, CollisionDetector* collisionDetector):
  CollisionSolver2DBase(unfoldTreeList, polygonList, collisionDetector),
  RandomCollisionSolver(unfoldTreeList, polygonList, collisionDetector){
}

// Protected

int PredictingRandomCollisionSolver::rateMove(UnfoldTree* collidingTreeNode, UnfoldTree* newParentNode){
  UnfoldTree* oldParent = collidingTreeNode->m_parent;
  collidingTreeNode->attachToNewParent(newParentNode);
  m_polygonList->generateSubTreePolygons(collidingTreeNode);
  m_collisionDetector->detectAllCollisions();
  int numCollisions = m_collisionDetector->getNumberOfCollisions();
  collidingTreeNode->attachToNewParent(oldParent);
  m_polygonList->generateSubTreePolygons(collidingTreeNode);
  return numCollisions;
}

int PredictingRandomCollisionSolver::selectNewParent(UnfoldTree*& collidingTreeNode){
  std::vector<unsigned int> possibleNeighbors = collidingTreeNode->calculatePossibleNeighbors();
  int newParentIndex = -1;
  int currentRating = m_collisionDetector->getNumberOfCollisions();
  while(newParentIndex == -1 &&
        collidingTreeNode->m_parent != nullptr &&
        !m_interruptSolving){
    // Chance: [3,90]%
    int stepUpChance = std::min(1.f, static_cast<float>(m_rejectedSteps) / static_cast<float>(m_polygonList->size())) * 87 + 3;
    int randomStepUp = std::rand() % 100;

    // Nach oben hangeln, bis ein bewegbarer Knoten gefunden wurde.
    while(collidingTreeNode->m_parent != nullptr &&
          (randomStepUp < stepUpChance ||
           possibleNeighbors.empty()) &&
          !m_interruptSolving){
      collidingTreeNode = collidingTreeNode->m_parent;
      possibleNeighbors = collidingTreeNode->calculatePossibleNeighbors();
      stepUpChance--;
    }
    // Root kann nicht bewegt werden.
    if(collidingTreeNode->m_parent == nullptr){
      break;
    }

    for(const int possibleNeighborIndex: possibleNeighbors){
      if(m_interruptSolving){
        return -1;
      }
      int newRating = rateMove(collidingTreeNode, m_unfoldTreeList->findTreeNodeByIndex(possibleNeighborIndex));
      // Chance Verschlechterung zu akzeptieren sinkt hyperbolisch mit der Zeit (Anzahl Schritte vs Anzahl Polygone). Chance: [3,90[
      float acceptWorseningChance = static_cast<float>(m_polygonList->size()) / static_cast<float>(m_polygonList->size() + m_iterationsDone) * 87.f + 3.f;
      int randomAcceptWorsening = std::rand() % 100;
      if(newRating < currentRating ||
         randomAcceptWorsening < acceptWorseningChance){
        newParentIndex = possibleNeighborIndex;
        currentRating = newRating;
      }
    }
  }
  return newParentIndex;
}

bool PredictingRandomCollisionSolver::takeStep(UnfoldTree*& currentTreeNode){
  int newParentIndex = selectNewParent(currentTreeNode);
  if(newParentIndex == -1 ||
     m_interruptSolving){
    return false;
  }

  UnfoldTree* newParent = m_unfoldTreeList->findTreeNodeByIndex(newParentIndex);
  currentTreeNode->attachToNewParent(newParent);
  m_polygonList->generateSubTreePolygons(currentTreeNode);
  return true;
}
