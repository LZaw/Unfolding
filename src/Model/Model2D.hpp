
#pragma once

// C++ Headers
#include <memory>
#include <string>
#include <utility>
#include <vector>

// Project Headers
#include <Collisions/Detector/CollisionDetector.hpp>
#include <Collisions/Solver/2D/CollisionSolver2DBase.hpp>
#include <Datastructures/Mesh/UnfoldMesh.hpp>
#include <Datastructures/Polygon/PolygonList.hpp>
#include <SVG/SVGWriter.hpp>
#include <Datastructures/Unfolding/UnfoldTree.hpp>
#include <Datastructures/Unfolding/UnfoldTreeList.hpp>

// Eigen Headers
#include <Eigen/Dense>

class Model2D
{
public:
  explicit Model2D(const UnfoldMesh* mesh);

  void attachPolygonToNewNeighbor(const unsigned int polygonIndex, const unsigned int newParentIndex);

  std::vector<unsigned int> calculateIntersectingPolygonIndicesByCoordinate(const Eigen::Vector2d& coordinate) const;
  unsigned int calculateParentPolygon(const unsigned int polygonIndex) const;
  std::vector<unsigned int> calculatePossibleNeighbors(unsigned int polygonIndex) const;
  std::vector<unsigned int> calculateTailPolygons(unsigned int polygonIndex) const;

  void drop();

  void generatePolygonList();
  const Eigen::Matrix<double, 2, 3>& getBasis() const;
  const Eigen::AlignedBox2d& getBoundingBox() const;
  CollisionDetector& getCollisionDetector();
  const std::vector<std::vector<unsigned int>>& getCollisions() const;
  unsigned int getNumberOfCollisions() const;
  const PolygonList& getPolygonList() const;
  PolygonList& getPolygonListRef();
  UnfoldTree* getUnfoldTree();

  void saveColoredUnfolding(const std::string& path);
  void saveUnfolding(const std::string& path, bool withFoldInstructions = true);
  void saveUnfoldTree(const std::string& path);
  void setUnfoldTreeList(UnfoldTreeList* unfoldTreeList);

  void updatePolygonList();

private:
  Eigen::Matrix<double, 2, 3>   m_basis;
  CollisionDetector             m_collisionDetector;
  const UnfoldMesh*             m_foldedMesh;
  PolygonList                   m_polygonList;
  SVGWriter                     m_svgWriter;
  UnfoldTreeList*               m_unfoldTreeList;

  void calculateBasis();
};
