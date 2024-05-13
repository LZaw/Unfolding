
#include "TabuSolver.hpp"

// C++ Headers
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <limits>
#include <utility>

TabuSolver::TabuSolver(Model2D* model2D, Unfolder* unfolder):
  CollisionSolver2DBase(&unfolder->getUnfoldTreeList(), &model2D->getPolygonListRef(), &model2D->getCollisionDetector()),
  m_forbiddenMovesVector(m_polygonList->size()),
  m_memoryResetCondition(std::max(m_polygonList->size() / 100, static_cast<size_t>(10))),
  m_model2D(model2D),
  m_reinitCondition(0.5 * std::sqrt(m_polygonList->size()) * m_polygonList->size()){
  // Durchschnittliche Valenz ausrechnen
  const PolygonList& polygonList = model2D->getPolygonList();
  double val = 0.;
  for(std::vector<Polygon2D>::const_iterator polygonListIterator = polygonList.cbegin(); polygonListIterator != polygonList.cend(); ++polygonListIterator){
    val += polygonListIterator->m_vertexList.size();
  }
  val /= polygonList.size();

  m_maxMemoryMoves = val * std::log(m_polygonList->size()) / std::log(val);
}

// Protected

bool TabuSolver::determineAndResolvePossibleLocalMinima(){
  // Reset Memory, if no steps can be taken
  if(m_rejectedSteps > m_memoryResetCondition){
    resetLocalVariables();
  }

  // Random Reroot
  m_unfoldTreeList->reroot(std::rand() % m_polygonList->size());
  m_model2D->updatePolygonList();

  // Reroot, if collisions are blocking
  if(m_rejectedSteps > 0){
    const std::vector<std::vector<unsigned int>>& collisions = m_collisionDetector->getCollisions();
    std::vector<unsigned int> collidingFaces;
    collidingFaces.reserve(m_polygonList->size());
    bool reroot = true;
    for(unsigned int i = 0; i < collisions.size(); ++i){
      if(!collisions[i].empty()){
        collidingFaces.push_back(i);
        UnfoldTree* treeNode = m_unfoldTreeList->findTreeNodeByIndex(i);
        if(!setUpPossibleNeighbors(treeNode).empty()){
          reroot = false;
          break;
        }
      }
    }
    if(reroot){
      if(collidingFaces.empty()){
        return true;
      }
      int collidingFace = collidingFaces[std::rand() % collidingFaces.size()];
      UnfoldTree* collidingFaceNode = m_unfoldTreeList->findTreeNodeByIndex(collidingFace);
      std::vector<unsigned int> subTreeIndices = collidingFaceNode->calculateSubTreeIndices();
      if(subTreeIndices.size() <= 1){
        m_unfoldTreeList->reroot(std::rand() % m_polygonList->size());
        m_model2D->updatePolygonList();
      }else{
        m_unfoldTreeList->reroot(subTreeIndices[std::rand() % subTreeIndices.size()]);
        m_model2D->updatePolygonList();
      }
    }
  }
  return true;
}

void TabuSolver::resetLocalVariables(){
  m_forbiddenMovesDeque.clear();
  m_forbiddenMovesVector.clear();
  m_forbiddenMovesVector.resize(m_polygonList->size());
}

bool TabuSolver::takeStep(UnfoldTree*& currentTreeNode){
  if(currentTreeNode == nullptr){
    std::cout << "Nullptr on Input" << std::endl;
    return false;
  }

  // Initialize History variables
  int bestNewNumberOfCollisions = std::numeric_limits<int>::max();
  UnfoldTree* bestNewParent = nullptr;
  UnfoldTree* bestCurrentTreeNode = nullptr;

  // Main loop
  while(currentTreeNode != nullptr &&
        currentTreeNode->m_parent != nullptr){
    // Set up filtered reachable Neighbors
    std::vector<unsigned int> possibleNeighbors;
    while(currentTreeNode->m_parent != nullptr &&
          possibleNeighbors.empty()){
      possibleNeighbors = setUpPossibleNeighbors(currentTreeNode);
      if(possibleNeighbors.empty()){
        climb(currentTreeNode);
      }
    }

    // Select best Neighbor
    int newNumberOfCollisions;
    UnfoldTree* currentBestNeighbor;
    std::tie(currentBestNeighbor, newNumberOfCollisions) = selectBestNeighbor(currentTreeNode, possibleNeighbors);

    // No best parent found
    if(currentBestNeighbor == nullptr){
      climb(currentTreeNode);
      continue;
    }

    // Possibly accept move and return
    if(acceptMove(newNumberOfCollisions)){
      performMove(currentTreeNode, currentBestNeighbor);
      return true;
    }

    // Remember best Move
    if(newNumberOfCollisions < bestNewNumberOfCollisions){
      bestNewParent = currentBestNeighbor;
      bestCurrentTreeNode = currentTreeNode;
      bestNewNumberOfCollisions = newNumberOfCollisions;
    }

    // Climb
    climb(currentTreeNode);
  }
  // If no move fullfilled absolute criteria, perform relative best move.
  if(bestNewParent != nullptr){
    performMove(bestCurrentTreeNode, bestNewParent);
    currentTreeNode = bestCurrentTreeNode;
    return true;
  }
  return false;
}

// Private

bool TabuSolver::acceptMove(const unsigned int newNumberOfCollisions) const{
  if(newNumberOfCollisions < m_collisionDetector->getNumberOfCollisions()){
    return true;
  }
  return false;
}

void TabuSolver::climb(UnfoldTree*& currentTreeNode){
  currentTreeNode = currentTreeNode->m_parent;
}

void TabuSolver::performMove(UnfoldTree* currentTreeNode, UnfoldTree* newParent){
  m_forbiddenMovesDeque.push_back({currentTreeNode->m_ref, currentTreeNode->m_parent->m_ref});
  m_forbiddenMovesVector[currentTreeNode->m_ref].push_back(currentTreeNode->m_parent->m_ref);
  while(m_forbiddenMovesDeque.size() > m_maxMemoryMoves){
    const std::pair<unsigned int, unsigned int>& front = m_forbiddenMovesDeque.front();
    removeForbiddenEntry(front.first, front.second);
    m_forbiddenMovesDeque.pop_front();
  }
  currentTreeNode->attachToNewParent(newParent);
  m_polygonList->generateSubTreePolygons(currentTreeNode);
}

void TabuSolver::removeForbiddenEntry(const unsigned int nodeIndex, const unsigned int forbiddenIndex){
  std::vector<unsigned int>::iterator it =
    std::find_if(m_forbiddenMovesVector[nodeIndex].begin(),
                 m_forbiddenMovesVector[nodeIndex].end(),
                 [forbiddenIndex](unsigned int index){
                   return index == forbiddenIndex;
                 }
                );
  if(it != m_forbiddenMovesVector[nodeIndex].end()){
    m_forbiddenMovesVector[nodeIndex].erase(it);
  }
}

std::pair<UnfoldTree*, unsigned int> TabuSolver::selectBestNeighbor(UnfoldTree* currentTreeNode, const std::vector<unsigned int>& possibleNeighbors){
  UnfoldTree* oldParent = currentTreeNode->m_parent;
  UnfoldTree* bestNeighbor = nullptr;
  unsigned int newNumberOfCollisions = std::numeric_limits<unsigned int>::max();

  for(unsigned int possibleNeighbor: possibleNeighbors){
    // Perform Move
    UnfoldTree* newParent = m_unfoldTreeList->findTreeNodeByIndex(possibleNeighbor);
    currentTreeNode->attachToNewParent(newParent);

    // Calculate Measures
    m_polygonList->generateSubTreePolygons(currentTreeNode);
    unsigned int currentNumberOfCollisions = m_collisionDetector->predictNumberofCollisionsOnSubTree(currentTreeNode);

    if(currentNumberOfCollisions < newNumberOfCollisions){
      newNumberOfCollisions = currentNumberOfCollisions;
      bestNeighbor = newParent;
    }

    // Undo Move
    currentTreeNode->attachToNewParent(oldParent);
  }
  // Recover polygonlist to current state
  m_polygonList->generateSubTreePolygons(currentTreeNode);

  return {bestNeighbor, newNumberOfCollisions};
}

std::vector<unsigned int> TabuSolver::setUpPossibleNeighbors(UnfoldTree* currentTreeNode){
  // Calculate reachable Neighbors
  std::vector<unsigned int> possibleNeighbors;
  possibleNeighbors = currentTreeNode->calculatePossibleNeighbors();

  // Filter out previous moves
  std::vector<unsigned int> filteredPossibleNeighbors;
  filteredPossibleNeighbors.reserve(possibleNeighbors.size());
  std::copy_if(
    possibleNeighbors.cbegin(),
    possibleNeighbors.cend(),
    std::back_inserter(filteredPossibleNeighbors),
    [this, currentTreeNode](unsigned int possibleNeighbor){
      const std::vector<unsigned int>::const_iterator it =
        std::find(
          m_forbiddenMovesVector[currentTreeNode->m_ref].cbegin(),
          m_forbiddenMovesVector[currentTreeNode->m_ref].cend(),
          possibleNeighbor
        );
      return it == m_forbiddenMovesVector[currentTreeNode->m_ref].cend();
    }
  );
  return filteredPossibleNeighbors;
}
