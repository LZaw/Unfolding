
#pragma once

// C++ Headers
#include <vector>

// Project Headers
#include <Datastructures/Polygon/PolygonList.hpp>
#include <Datastructures/Unfolding/UnfoldTree.hpp>

// Eigen Headers
#include <Eigen/Dense>

class CollisionDetector{
public:
  CollisionDetector(const PolygonList* polygonList);

  virtual const std::vector<std::vector<unsigned int>>& detectAllCollisions();
  virtual const std::vector<std::vector<unsigned int>>& detectCollisionsOnSubTree(const UnfoldTree* subTreeRootNode);

  const std::vector<std::vector<unsigned int>>& getCollisions() const;
  size_t getNumberOfCollisions() const;

  void initialize(const size_t polygonListSize);

  virtual unsigned int predictNumberofAllCollisions() const;
  virtual unsigned int predictNumberofCollisionsOnSubTree(const UnfoldTree* subTreeRootNode) const;

  void reset();

  void setPolygonList(const PolygonList* polygonList);

protected:
  std::vector<
    std::vector<unsigned int>>  m_collisions;
  unsigned int                  m_numberOfCollisions = 0;
  const PolygonList*            m_polygonList;
  std::vector<Polygon2D>        m_sortedPolygons;

private:
  int calculateLargerBoundingBoxSize() const;

  inline void prepareFullList(std::vector<Polygon2D>& polygonsToSort) const;

  inline void sortPolygons(std::vector<Polygon2D>& polygonsToSort, int largerDimension) const;
};
