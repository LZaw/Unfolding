
#include "Polygon2D.hpp"

// C++ Headers
#include <iostream>

const Eigen::Matrix<double, 2, 3>* Polygon2D::s_basis = nullptr;
const UnfoldMesh* Polygon2D::s_mesh = nullptr;
int Polygon2D::s_originFaceIndex = -1;

Polygon2D::Polygon2D(const int index, const Eigen::Affine3d& transformation):
  m_faceIndex(index),
  m_globalTransformation(transformation),
  m_sign(0){
  m_center.setZero();
  m_color.setZero();
}

Polygon2D::Polygon2D(const Polygon2D& other):
  m_axisIndices(other.m_axisIndices),
  m_bBox(other.m_bBox),
  m_center(other.m_center),
  m_color(other.m_color),
  m_faceIndex(other.m_faceIndex),
  m_globalTransformation(other.m_globalTransformation),
  m_localTransformation(other.m_localTransformation),
  m_sign(other.m_sign),
  m_vertexList(other.m_vertexList){
}

// Public

void Polygon2D::create(const int index, const Eigen::Affine3d& globalTransformation, const UnfoldTransformation& localTransformation){
  initialize(index, globalTransformation, localTransformation);
  update();
}

void Polygon2D::initialize(const int index, const Eigen::Affine3d& globalTransformation, const UnfoldTransformation& localTransformation){
  m_faceIndex = index;
  m_globalTransformation = globalTransformation;
  m_localTransformation = localTransformation;
  m_sign = 0;
}

bool Polygon2D::intersects(const Polygon2D& other) const{
  if(m_faceIndex == other.m_faceIndex){
    return false;
  }
  for(size_t i = 0; i < m_vertexList.size(); ++i){
    const Eigen::Vector2d& polygon0V0 = m_vertexList[i];
    const Eigen::Vector2d& polygon0V1 = m_vertexList[(i + 1) % m_vertexList.size()];
    const Eigen::Vector2d& polygon0Direction = (polygon0V1 - polygon0V0) / 1024.;
    for(size_t j = 0; j < other.m_vertexList.size(); ++j){
      const Eigen::Vector2d& polygon1V0 = other.m_vertexList[j];
      const Eigen::Vector2d& polygon1V1 = other.m_vertexList[(j + 1) % other.m_vertexList.size()];
      // If lines are almost identical, don't check for intersection
      if(((polygon0V0 - polygon1V0).squaredNorm() < std::numeric_limits<float>::epsilon() && (polygon0V1 - polygon1V1).squaredNorm() < std::numeric_limits<float>::epsilon()) ||
         ((polygon0V0 - polygon1V1).squaredNorm() < std::numeric_limits<float>::epsilon() && (polygon0V1 - polygon1V0).squaredNorm() < std::numeric_limits<float>::epsilon())){
        continue;
      }
      const Eigen::Vector2d& polygon1Direction = (polygon1V1 - polygon1V0) / 1024.;
      if(PolygonUtil::lineLineIntersection(polygon0V0 + polygon0Direction, polygon0V1 - polygon0Direction,
                              polygon1V0 + polygon1Direction, polygon1V1 - polygon1Direction)){
        return true;
      }
    }
  }
  return false;
}

// Jordan Algorithm
bool Polygon2D::intersectsPoint(const Eigen::Vector2d& point) const{
  if(!m_bBox.contains(point)){
    return false;
  }
  Eigen::Vector2d sideStartPoint;
  Eigen::Vector2d sideEndPoint;
  Eigen::Vector2d rayStartPoint= m_bBox.min() - m_bBox.diagonal() / 100.;
  Eigen::Vector2d rayEndPoint(point.x(), point.y());
  int intersections = 0;

  for(size_t side = 0; side < m_vertexList.size(); ++side){
    sideStartPoint = m_vertexList[side];
    sideEndPoint = m_vertexList[(side + 1) % m_vertexList.size()];
    if(PolygonUtil::lineLineIntersection(sideStartPoint, sideEndPoint, rayStartPoint, rayEndPoint)){
      ++intersections;
    }
  }
  if(intersections & 1){
    return true;
  }else{
    return false;
  }
}

void Polygon2D::setBasis(const Eigen::Matrix<double, 2, 3>* const basis){
  s_basis = basis;
}

void Polygon2D::setMesh(const UnfoldMesh* const mesh){
  s_mesh = mesh;
}

void Polygon2D::setOrigin(const int originFaceIndex){
  s_originFaceIndex = originFaceIndex;
}

void Polygon2D::update(){
  if(m_faceIndex == -1 || s_basis == nullptr || s_mesh == nullptr || s_originFaceIndex == -1){
    std::cerr << "Polygon not initialized yet." << std::endl;
    return;
  }
  m_bBox.setEmpty();
  const std::vector<unsigned int>& currentFace = s_mesh->getFaces()[m_faceIndex];
  m_center.setZero();
  m_vertexList.resize(currentFace.size());
  unsigned int v0 = currentFace.size();
  unsigned int v1 = currentFace.size();
  const Eigen::Vector3d& origin = s_mesh->getVertices().row(s_mesh->getFaces()[s_originFaceIndex][0]);
  for(size_t i = 0; i < currentFace.size(); ++i){
    if(static_cast<int>(currentFace[i]) == m_localTransformation.m_globalV0){
      v0 = i;
    }
    if(static_cast<int>(currentFace[i]) == m_localTransformation.m_globalV1){
      v1 = i;
    }
    const Eigen::Vector3d tmp(m_globalTransformation * Eigen::Vector3d(s_mesh->getVertices().row(currentFace[i])) - origin);
    const Eigen::Vector2d currentPoint = *s_basis * tmp;
    m_bBox.extend(currentPoint);
    m_vertexList[i] = currentPoint;
    m_center += currentPoint;
  }
  if(m_faceIndex != s_originFaceIndex &&
    (v0 == currentFace.size() || v1 == currentFace.size())){
    std::cout << "Couldn't find vertex within face: " << m_faceIndex << std::endl;
    std::cout << "Transformation: " << m_localTransformation.m_sourceID << " -> " << m_localTransformation.m_targetID << std::endl;
  }
  m_axisIndices = {v0, v1};
  m_center /= m_vertexList.size();
  if(m_faceIndex != s_originFaceIndex){
    const Eigen::Vector3d windingAxis = (s_mesh->getVertices().row(m_localTransformation.m_globalV1) - s_mesh->getVertices().row(m_localTransformation.m_globalV0)).normalized();
    if(windingAxis.dot(m_localTransformation.rotation.vec().normalized()) < 0){
      m_sign = -1;
    }else{
      m_sign = 1;
    }
  }else{
    m_sign = 0;
    m_axisIndices = {-1, -1};
  }
}
