
#include "Model2D.hpp"

// C++ Headers
#include <algorithm>
#include <queue>

// Project Headers
#include <Datastructures/Helpers/UnfoldTransformation.hpp>
#include <Datastructures/Polygon/Polygon2D.hpp>
#include <Util/FileUtil.hpp>
#include <Util/PolygonUtil.hpp>

Model2D::Model2D(const UnfoldMesh* foldedMesh):
  m_collisionDetector(&m_polygonList),
  m_foldedMesh(foldedMesh),
  m_polygonList(m_foldedMesh),
  m_unfoldTreeList(nullptr){
  m_basis.setZero();
  Polygon2D::setMesh(m_foldedMesh);
  Polygon2D::setBasis(&m_basis);
}

// Public

void Model2D::attachPolygonToNewNeighbor(const unsigned int polygonIndex, const unsigned int newParentIndex){
  UnfoldTree* polygonNode = m_unfoldTreeList->findTreeNodeByIndex(polygonIndex);
  UnfoldTree* newParentNode = m_unfoldTreeList->findTreeNodeByIndex(newParentIndex);
  polygonNode->attachToNewParent(newParentNode);
  m_polygonList.generateSubTreePolygons(polygonNode);
  m_collisionDetector.detectCollisionsOnSubTree(polygonNode);
}

std::vector<unsigned int> Model2D::calculateIntersectingPolygonIndicesByCoordinate(const Eigen::Vector2d& coordinate) const{
  return m_polygonList.calculateIntersectingPolygonIndicesByCoordinate(coordinate);
}

unsigned int Model2D::calculateParentPolygon(const unsigned int polygonIndex) const{
  const UnfoldTree* treeNode = m_unfoldTreeList->findTreeNodeByIndex(polygonIndex);
  if(treeNode->m_parent == nullptr){
    return polygonIndex;
  }
  return treeNode->m_parent->m_ref;
}

std::vector<unsigned int> Model2D::calculatePossibleNeighbors(unsigned int polygonIndex) const{
  const UnfoldTree* treeNode = m_unfoldTreeList->findTreeNodeByIndex(polygonIndex);
  return treeNode->calculatePossibleNeighbors();
}

std::vector<unsigned int> Model2D::calculateTailPolygons(unsigned int polygonIndex) const{
  std::vector<unsigned int> tailPolygons;
  tailPolygons.reserve(m_foldedMesh->getFaces().size());
  const UnfoldTree* treeNode = m_unfoldTreeList->findTreeNodeByIndex(polygonIndex);
  std::queue<const UnfoldTree*> nextTreeNodes;
  for(std::vector<UnfoldTree*>::const_iterator it = treeNode->m_children.cbegin(); it != treeNode->m_children.cend(); ++it){
    nextTreeNodes.push(*it);
  }
  while(!nextTreeNodes.empty()){
    const UnfoldTree* currentTreeNode = nextTreeNodes.front();
    nextTreeNodes.pop();
    tailPolygons.push_back(currentTreeNode->m_ref);
    for(std::vector<UnfoldTree*>::const_iterator it = currentTreeNode->m_children.cbegin(); it != currentTreeNode->m_children.cend(); ++it){
      nextTreeNodes.push(*it);
    }
  }
  tailPolygons.shrink_to_fit();
  return tailPolygons;
}

void Model2D::drop(){
  m_basis.setZero();
  m_polygonList.drop();
  m_collisionDetector.reset();
}

void Model2D::generatePolygonList(){
  calculateBasis();
  m_polygonList.generateNewPolygons();
  m_collisionDetector.initialize(m_polygonList.size());
  m_collisionDetector.detectAllCollisions();
}

const Eigen::Matrix<double, 2, 3>& Model2D::getBasis() const{
  return m_basis;
}

const Eigen::AlignedBox2d& Model2D::getBoundingBox() const{
  return m_polygonList.getBoundingBox();
}

CollisionDetector& Model2D::getCollisionDetector(){
  return m_collisionDetector;
}

const std::vector<std::vector<unsigned int>>& Model2D::getCollisions() const{
  return m_collisionDetector.getCollisions();
}

unsigned int Model2D::getNumberOfCollisions() const{
  return m_collisionDetector.getNumberOfCollisions();
}

const PolygonList& Model2D::getPolygonList() const{
  return m_polygonList;
}

PolygonList& Model2D::getPolygonListRef(){
  return m_polygonList;
}

UnfoldTree* Model2D::getUnfoldTree(){
  return m_unfoldTreeList->getRoot();
}

void Model2D::saveColoredUnfolding(const std::string& path){
  m_svgWriter.savePolygonsToSVG(path, m_polygonList);
}

void Model2D::saveUnfolding(const std::string& path, bool withFoldInstructions){
  m_svgWriter.saveUnfoldingToSVG(path, m_polygonList, false);
  if(withFoldInstructions){
    m_svgWriter.saveUnfoldingToSVG(FileUtil::removeFileEnding(path) + "-Folding.svg", m_polygonList, true);
  }
}

void Model2D::saveUnfoldTree(const std::string& path){
  m_svgWriter.saveUnfoldTreeToSVG(path, m_polygonList);
}

void Model2D::setUnfoldTreeList(UnfoldTreeList* unfoldTreeList){
  m_unfoldTreeList = unfoldTreeList;
  m_polygonList.setUnfoldTreeList(m_unfoldTreeList);
}

void Model2D::updatePolygonList(){
  calculateBasis();
  m_polygonList.generateNewPolygons();
}

// Private

void Model2D::calculateBasis(){
  if(m_foldedMesh->getFaces().size() == 0){
    return;
  }
  const std::vector<unsigned int>& currentFace = m_foldedMesh->getFaces()[m_unfoldTreeList->getRootIndex()];
  m_basis.setZero();
  m_basis.row(0) = (m_foldedMesh->getVertices().row(currentFace[1]) - m_foldedMesh->getVertices().row(currentFace[0])).normalized();
  m_basis.row(1) = -(m_foldedMesh->getVertices().row(currentFace[2]) - m_foldedMesh->getVertices().row(currentFace[0])).normalized();
  m_basis.row(1) -= m_basis.row(1).dot(m_basis.row(0)) * m_basis.row(0);
  m_basis.row(1).normalize();
}
