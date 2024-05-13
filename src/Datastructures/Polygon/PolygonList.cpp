
#include "PolygonList.hpp"

// C++ Headers
#include <iterator>
#include <limits>
#include <memory>

// Project Headers
#include <Util/PolygonUtil.hpp>

PolygonList::PolygonList(const UnfoldMesh* foldedMesh):
  m_foldedMesh(foldedMesh){
}

// Public

std::vector<unsigned int> PolygonList::calculateIntersectingPolygonIndicesByCoordinate(const Eigen::Vector2d& coordinate) const{
  std::vector<unsigned int> ret;
  ret.reserve(8);
  for(const_iterator polygonIterator = cbegin(); polygonIterator != cend(); ++polygonIterator){
    if((*polygonIterator).intersectsPoint(coordinate)){
      ret.push_back(std::distance(cbegin(), polygonIterator));
    }
  }
  return ret;
}

void PolygonList::calculateCutEdges(std::vector<std::vector<std::pair<int, int>>>& cutEdges) const{
  std::vector<std::vector<int>> mountainEdges(m_foldedMesh->getVertices().rows());
  std::vector<std::vector<int>> valleyEdges(m_foldedMesh->getVertices().rows());
  for(const_iterator it = cbegin(); it != cend(); ++it){
    const Polygon2D& currentPolygon = (*it);
    if(currentPolygon.m_sign == 0 || currentPolygon.m_faceIndex == -1){
      continue;
    }
    int globalV0 = m_foldedMesh->getFaces()[currentPolygon.m_faceIndex][currentPolygon.m_axisIndices.first];
    int globalV1 = m_foldedMesh->getFaces()[currentPolygon.m_faceIndex][currentPolygon.m_axisIndices.second];
    if(currentPolygon.m_sign == -1){
      mountainEdges[globalV0].push_back(globalV1);
      mountainEdges[globalV1].push_back(globalV0);
    }
    if(currentPolygon.m_sign == 1){
      valleyEdges[globalV0].push_back(globalV1);
      valleyEdges[globalV1].push_back(globalV0);
    }
  }
  for(const_iterator it = cbegin(); it != cend(); ++it){
    const Polygon2D& currentPolygon = (*it);
    if(currentPolygon.m_faceIndex == -1){
      continue;
    }
    cutEdges[currentPolygon.m_faceIndex].reserve(m_foldedMesh->getFaces()[currentPolygon.m_faceIndex].size());
    for(std::vector<Eigen::Vector2d>::const_iterator vertexIterator = currentPolygon.m_vertexList.cbegin(); vertexIterator != currentPolygon.m_vertexList.cend(); ++vertexIterator){
      int index = std::distance(currentPolygon.m_vertexList.cbegin(), vertexIterator);
      int globalV0 = m_foldedMesh->getFaces()[currentPolygon.m_faceIndex][index];
      int globalV1 = m_foldedMesh->getFaces()[currentPolygon.m_faceIndex][(index + 1) % currentPolygon.m_vertexList.size()];
      if(std::find(mountainEdges[globalV0].cbegin(), mountainEdges[globalV0].cend(), globalV1) != mountainEdges[globalV0].cend() ||
         std::find(valleyEdges[globalV0].cbegin(), valleyEdges[globalV0].cend(), globalV1) != valleyEdges[globalV0].cend()){
        continue;
      }
      cutEdges[currentPolygon.m_faceIndex].push_back({index, (index + 1) % currentPolygon.m_vertexList.size()});
    }
  }
}

void PolygonList::drop(){
  clear();
  m_bBox.setEmpty();
}

void PolygonList::generateNewPolygons(){
  Polygon2D::setOrigin(m_unfoldTreeList->getRootIndex());
  resize(m_foldedMesh->getFaces().size());
  m_unfoldTreeList->traverseTreeToPolygonList(*this);
  adjustBoundingBox();
}

void PolygonList::generateSubTreePolygons(const UnfoldTree* subTreeRootNode){
  if(subTreeRootNode->m_parent == nullptr){
    generateNewPolygons();
    return;
  }
  Eigen::Affine3d parentTransformation = (*this)[subTreeRootNode->m_parent->m_ref].m_globalTransformation;
  subTreeRootNode->traverseTreeToPolygonList(*this, parentTransformation);
  adjustBoundingBox();
}

const Eigen::AlignedBox2d& PolygonList::getBoundingBox() const{
  return m_bBox;
}

const std::vector<Polygon2D>& PolygonList::getPolygonList() const{
  return *this;
}

const UnfoldTreeList* PolygonList::getUnfoldTreeList() const{
  return m_unfoldTreeList;
}

std::vector<Polygon2D> PolygonList::previewMove(unsigned int polygonIndex, unsigned int newParentIndex) const{
  std::unique_ptr<UnfoldTree> newSubTree(new UnfoldTree(m_unfoldTreeList->findConstTreeNodeByIndex(polygonIndex)));
  std::vector<Polygon2D> previewPolygons(newSubTree->calculateTreeSize());
  newSubTree->m_transformation = m_foldedMesh->getDualGraph().findUnfoldTransformation(polygonIndex, newParentIndex);
  Eigen::Affine3d parentTransformation = (*this)[newParentIndex].m_globalTransformation;
  newSubTree->traverseTreeToPolygonList(previewPolygons, parentTransformation, false);
  return previewPolygons;
}

void PolygonList::setColors(const Eigen::MatrixXf& colors){
  for(Eigen::Index i = 0; i < colors.rows(); i++){
    (*this)[i].m_color = colors.row(i);
  }
}

void PolygonList::setUnfoldTreeList(const UnfoldTreeList* unfoldTreeList){
  m_unfoldTreeList = unfoldTreeList;
}

// Private

void PolygonList::adjustBoundingBox(){
  m_bBox.setEmpty();
  for(const Polygon2D& currentPolygon: *this){
    m_bBox.extend(currentPolygon.m_bBox);
  }
}
