
#include "CollisionDetector.hpp"

// C++ Headers
#include <algorithm>
#include <cmath>
#include <iostream>
#include <unordered_set>

CollisionDetector::CollisionDetector(const PolygonList* polygonList):
  m_polygonList(polygonList){
}

// Public

const std::vector<std::vector<unsigned int>>& CollisionDetector::detectAllCollisions(){
  if(m_collisions.size() != m_polygonList->size()){
    initialize(m_polygonList->size());
  }

  // Kollisionen löschen
  for(std::vector<std::vector<unsigned int>>::iterator collisionIt = m_collisions.begin(); collisionIt != m_collisions.cend(); ++collisionIt){
    collisionIt->clear();
  }

  // Zähler auf Null stellen
  m_numberOfCollisions = 0;

  // Entlang längerer Kante sortieren
  const int largerDimension = calculateLargerBoundingBoxSize();
  sortPolygons(m_sortedPolygons, largerDimension);

  // Alle Polygone durchgehen
  for(std::vector<Polygon2D>::const_iterator polygonIterator = m_sortedPolygons.cbegin(); polygonIterator != m_sortedPolygons.cend(); ++polygonIterator){
    const Polygon2D& comparePolygon = *polygonIterator;
    // Nur die Polygone durchgehen, die nach dem aktuellen kommen
    for(std::vector<Polygon2D>::const_iterator currentIterator = std::next(polygonIterator); currentIterator != m_sortedPolygons.cend(); ++currentIterator){
      const Polygon2D& currentPolygon = *currentIterator;
      // Sweepline check
      if(currentPolygon.m_bBox.min()[largerDimension] > comparePolygon.m_bBox.max()[largerDimension]){
        break;
      }

      // BoundingBox check
      if(!currentPolygon.m_bBox.intersects(comparePolygon.m_bBox)){
        continue;
      }

      if(comparePolygon.intersects(currentPolygon)){
        m_collisions[comparePolygon.m_faceIndex].push_back(currentPolygon.m_faceIndex);
        m_collisions[currentPolygon.m_faceIndex].push_back(comparePolygon.m_faceIndex);
        ++m_numberOfCollisions;
      }
    }
  }

  return m_collisions;
}

const std::vector<std::vector<unsigned int>>& CollisionDetector::detectCollisionsOnSubTree(const UnfoldTree* subTreeRootNode){
  // Nicht korrekt initialisiert -> Es muss eh alles neu berechnet werden
  if(m_collisions.size() != m_polygonList->size()){
    initialize(m_polygonList->size());
    return detectAllCollisions();
  }

  // Teilbaum zu groß -> Sweepline ist schneller
  unsigned int treeSize = subTreeRootNode->calculateTreeSize();
  if(treeSize > std::log(m_polygonList->size()) * 8){
    return detectAllCollisions();
  }

  // Collect treePolygonIndices and store them in a set
  const std::vector<unsigned int> treePolygonIndices = subTreeRootNode->calculateSubTreeIndices();
  const std::unordered_set<unsigned int> treePolygonIndicesSet(treePolygonIndices.cbegin(), treePolygonIndices.cend());

  // Remove old collisions and subtract collisioncount
  for(std::vector<unsigned int>::const_iterator treePolygonIndexIterator = treePolygonIndices.cbegin(); treePolygonIndexIterator != treePolygonIndices.cend(); ++treePolygonIndexIterator){
    for(std::vector<unsigned int>::const_iterator collisionNeighborIterator = m_collisions[*treePolygonIndexIterator].cbegin(); collisionNeighborIterator != m_collisions[*treePolygonIndexIterator].cend(); ++collisionNeighborIterator){
      std::vector<unsigned int>::const_iterator it = std::find(m_collisions[*collisionNeighborIterator].cbegin(), m_collisions[*collisionNeighborIterator].cend(), *treePolygonIndexIterator);
      if(it == m_collisions[*collisionNeighborIterator].cend()){
        continue;
      }
      m_collisions[*collisionNeighborIterator].erase(it);
    }
    // Tree-internal overlaps only get removed once
    m_numberOfCollisions -= m_collisions[*treePolygonIndexIterator].size();
    m_collisions[*treePolygonIndexIterator].clear();
  }

  // Detect new Collisions
  for(std::vector<unsigned int>::const_iterator treePolygonIt = treePolygonIndices.cbegin(); treePolygonIt != treePolygonIndices.cend(); ++treePolygonIt){
    const Polygon2D& comparePolygon = (*m_polygonList)[*treePolygonIt];
    for(PolygonList::const_iterator currentPolygonIt = m_polygonList->cbegin(); currentPolygonIt != m_polygonList->cend(); ++currentPolygonIt){
      const Polygon2D& currentPolygon = *currentPolygonIt;
      // Identity check
      if(currentPolygon.m_faceIndex == comparePolygon.m_faceIndex){
        continue;
      }
      // BoundingBox check
      if(!currentPolygon.m_bBox.intersects(comparePolygon.m_bBox)){
        continue;
      }
      // Count tree-internal overlaps only once
      if(treePolygonIndicesSet.find(currentPolygon.m_faceIndex) != treePolygonIndicesSet.cend()){
        if(currentPolygon.m_faceIndex < comparePolygon.m_faceIndex){
          continue;
        }
      }
      if(comparePolygon.intersects(currentPolygon)){
        m_collisions[comparePolygon.m_faceIndex].push_back(currentPolygon.m_faceIndex);
        m_collisions[currentPolygon.m_faceIndex].push_back(comparePolygon.m_faceIndex);
        ++m_numberOfCollisions;
      }
    }
  }

  return m_collisions;
}

const std::vector<std::vector<unsigned int>>& CollisionDetector::getCollisions() const{
  return m_collisions;
}

size_t CollisionDetector::getNumberOfCollisions() const{
  return m_numberOfCollisions;
}

void CollisionDetector::initialize(const size_t polygonListSize){
  m_collisions.clear();
  m_collisions.resize(polygonListSize);
  for(std::vector<std::vector<unsigned int>>::iterator collisionIt = m_collisions.begin(); collisionIt != m_collisions.end(); ++collisionIt){
    collisionIt->reserve(polygonListSize);
  }
}

unsigned int CollisionDetector::predictNumberofAllCollisions() const{
  // Nicht korrekt initialisiert -> Keine Vorhersage möglich
  if(m_collisions.size() != m_polygonList->size()){
    std::cerr << "Can't predict uninitialized setup." << std::endl;
    return -1;
  }

  int numberOfCollisions = 0;
  // Entlang längerer Kante sortieren
  const int largerDimension = calculateLargerBoundingBoxSize();
  std::vector<Polygon2D> sortedPolygons;
  sortPolygons(sortedPolygons, largerDimension);

  // Alle Polygone durchgehen
  for(std::vector<Polygon2D>::const_iterator compareIt = sortedPolygons.cbegin(); compareIt != sortedPolygons.cend(); ++compareIt){
    const Polygon2D& comparePolygon = *compareIt;
    // Nur die Polygone durchgehen, die nach dem aktuellen kommen
    for(std::vector<Polygon2D>::const_iterator currentIt = std::next(compareIt); currentIt != sortedPolygons.cend(); ++currentIt){
      const Polygon2D& currentPolygon = *currentIt;
      // Sweepline check
      if(currentPolygon.m_bBox.min()[largerDimension] > comparePolygon.m_bBox.max()[largerDimension]){
        break;
      }

      // BoundingBox check
      if(!currentPolygon.m_bBox.intersects(comparePolygon.m_bBox)){
        continue;
      }

      if(comparePolygon.intersects(currentPolygon)){
        ++numberOfCollisions;
      }
    }
  }

  return numberOfCollisions;
}

unsigned int CollisionDetector::predictNumberofCollisionsOnSubTree(const UnfoldTree* subTreeRootNode) const{
  // Nicht korrekt initialisiert -> Keine Vorhersage möglich
  if(m_collisions.size() != m_polygonList->size()){
    std::cerr << "Can't predict uninitialized setup." << std::endl;
    return -1;
  }

  // Teilbaum zu groß -> Sweepline ist schneller
  unsigned int treeSize = subTreeRootNode->calculateTreeSize();
  if(treeSize > std::log(m_polygonList->size()) * 8){
    return predictNumberofAllCollisions();
  }

  // Collect treePolygonIndices
  const std::vector<unsigned int> treePolygonIndices = subTreeRootNode->calculateSubTreeIndices();

  // Remove old collisions and calculate number of collisions to recheck
  // This falsely will subtract tree-internal collisions twice, but they will be added later again.
  int numberOfCollisions = m_numberOfCollisions;
  for(std::vector<unsigned int>::const_iterator treePolygonIndexIterator = treePolygonIndices.cbegin(); treePolygonIndexIterator != treePolygonIndices.cend(); ++treePolygonIndexIterator){
    numberOfCollisions -= m_collisions[*treePolygonIndexIterator].size();
  }

  // Detect new Collisions
  for(std::vector<unsigned int>::const_iterator treeIt = treePolygonIndices.cbegin(); treeIt != treePolygonIndices.cend(); ++treeIt){
    const Polygon2D& comparePolygon = (*m_polygonList)[*treeIt];
    for(PolygonList::const_iterator currentIt = m_polygonList->cbegin(); currentIt != m_polygonList->cend(); ++currentIt){
      const Polygon2D& currentPolygon = *currentIt;
      // Identity check
      if(currentPolygon.m_faceIndex == comparePolygon.m_faceIndex){
        continue;
      }
      // BoundingBox check
      if(!currentPolygon.m_bBox.intersects(comparePolygon.m_bBox)){
        continue;
      }
      // This falsely will add tree-internal collisions twice, but they got subtracted before.
      if(comparePolygon.intersects(currentPolygon)){
        ++numberOfCollisions;
      }
    }
  }

  return numberOfCollisions;
}

void CollisionDetector::reset(){
  m_collisions.clear();
  m_numberOfCollisions = 0;
}

void CollisionDetector::setPolygonList(const PolygonList* polygonList){
  m_polygonList = polygonList;
}

// Private

int CollisionDetector::calculateLargerBoundingBoxSize() const{
  const Eigen::Vector2d& boundingBoxDiagonal = m_polygonList->getBoundingBox().diagonal();
  if(boundingBoxDiagonal[0] > boundingBoxDiagonal[1]){
    return 0;
  }else{
    return 1;
  }
}

void CollisionDetector::prepareFullList(std::vector<Polygon2D>& polygonsToSort) const{
  polygonsToSort.resize(m_polygonList->size());
  std::copy(m_polygonList->cbegin(), m_polygonList->cend(), polygonsToSort.begin());
}

void CollisionDetector::sortPolygons(std::vector<Polygon2D>& polygonsToSort, int largerDimension) const{
  prepareFullList(polygonsToSort);
  PolygonSorter pSorter(largerDimension);
  std::sort(polygonsToSort.begin(), polygonsToSort.end(), pSorter);
}
