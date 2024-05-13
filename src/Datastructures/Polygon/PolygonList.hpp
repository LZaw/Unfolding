
#pragma once

// C++ Headers
#include <utility>
#include <vector>

// Project Headers
#include <Datastructures/Mesh/UnfoldMesh.hpp>
#include <Datastructures/Polygon/Polygon2D.hpp>
#include <Datastructures/Unfolding/UnfoldTree.hpp>
#include <Datastructures/Unfolding/UnfoldTreeList.hpp>

// Eigen Headers
#include <Eigen/Dense>
#include <Eigen/Geometry>

class PolygonList : public std::vector<Polygon2D>{
public:
  PolygonList(const UnfoldMesh* foldedMesh);
  PolygonList(size_t size) = delete;

  std::vector<unsigned int> calculateIntersectingPolygonIndicesByCoordinate(const Eigen::Vector2d& coordinate) const;
  void calculateCutEdges(std::vector<std::vector<std::pair<int, int>>>& cutEdges) const;

  void drop();

  void generateNewPolygons();
  void generateSubTreePolygons(const UnfoldTree* subTreeRootNode);
  const Eigen::AlignedBox2d& getBoundingBox() const;
  const std::vector<Polygon2D>& getPolygonList() const;
  const UnfoldTreeList* getUnfoldTreeList() const;

  std::vector<Polygon2D> previewMove(unsigned int polygonIndex, unsigned int newParentIndex) const;

  void setColors(const Eigen::MatrixXf& colors);
  void setUnfoldTreeList(const UnfoldTreeList* unfoldTreeList);

private:
  Eigen::AlignedBox2d                 m_bBox;
  const UnfoldMesh*                   m_foldedMesh;
  const UnfoldTreeList*               m_unfoldTreeList = nullptr;

  inline void adjustBoundingBox();
};
