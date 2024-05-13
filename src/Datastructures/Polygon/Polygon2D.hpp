
#pragma once

// C++ Headers
#include <utility>
#include <vector>

// Project Headers
#include <Datastructures/Helpers/UnfoldTransformation.hpp>
#include <Datastructures/Mesh/UnfoldMesh.hpp>
#include <Util/PolygonUtil.hpp>

// Eigen Headers
#include <Eigen/Dense>
#include <Eigen/Geometry>

class Polygon2D{
public:
  Polygon2D(const int index = -1, const Eigen::Affine3d& transformation = Eigen::Affine3d::Identity());
  Polygon2D(const Polygon2D& other);

  void create(const int index, const Eigen::Affine3d& globalTransformation, const UnfoldTransformation& localTransformation);

  void initialize(const int index, const Eigen::Affine3d& globalTransformation, const UnfoldTransformation& localTransformation);
  bool intersects(const Polygon2D& other) const;
  bool intersectsPoint(const Eigen::Vector2d& point) const;

  static void setBasis(const Eigen::Matrix<double, 2, 3>* const basis);
  static void setMesh(const UnfoldMesh* const mesh);
  static void setOrigin(const int originFaceIndex);

  void update();

  std::pair<int, int>           m_axisIndices;
  Eigen::AlignedBox2d           m_bBox;
  Eigen::Vector2d               m_center;
  Eigen::Vector3f               m_color;
  int                           m_faceIndex;
  Eigen::Affine3d               m_globalTransformation;
  UnfoldTransformation          m_localTransformation;
  int                           m_sign;
  std::vector<Eigen::Vector2d>  m_vertexList;

private:
  static const Eigen::Matrix<double, 2, 3>* s_basis;
  static const UnfoldMesh*                  s_mesh;
  static int                                s_originFaceIndex;
};

struct PolygonSorter{
  int m_dim;
  PolygonSorter(int d){
    m_dim = d;
  }

  bool operator()(const Polygon2D& a, const Polygon2D& b){
    return a.m_bBox.min()[m_dim] < b.m_bBox.min()[m_dim];
  }
};
