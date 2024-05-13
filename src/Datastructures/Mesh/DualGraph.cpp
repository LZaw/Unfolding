
#include "DualGraph.hpp"

// C++ Headers
#include <algorithm>
#include <iostream>
#include <utility>

// Project Headers
#include <Datastructures/Mesh/FaceList.hpp>
#include <Datastructures/Mesh/UnfoldMesh.hpp>

DualGraph::DualGraph(const UnfoldMesh* const mesh):
  std::vector<std::vector<DualGraphNode>>(),
  m_mesh(mesh){
}

// Public

bool DualGraph::areNeighbors(const unsigned int sourceFaceID, const unsigned int targetFaceID) const{
  return std::find_if(
       (*this)[sourceFaceID].cbegin(),
       (*this)[sourceFaceID].cend(),
       [targetFaceID](const DualGraphNode& neighborNode){
         return static_cast<int>(targetFaceID) == neighborNode.getRef();
       }
     ) != (*this)[sourceFaceID].cend();
}

void DualGraph::create(bool withTransformations){
  clear();
  const FaceList& faces = m_mesh->getFaces();
  resize(faces.size());
  for(iterator dualGraphIterator = begin(); dualGraphIterator != end(); ++dualGraphIterator){
    dualGraphIterator->reserve(faces[std::distance(begin(), dualGraphIterator)].size());
  }
  std::unordered_map<unsigned int, std::unordered_map<unsigned int, unsigned int>> edgeMap = createEdgeMap();
  for(size_t i = 0; i < faces.size(); ++i){
    addEntry(i, edgeMap, withTransformations);
  }
}

DualGraphNode& DualGraph::findDualGraphNode(const unsigned int sourceFaceID, const unsigned int targetFaceID){
  return const_cast<DualGraphNode&>(const_cast<const DualGraph*>(this)->findDualGraphNode(sourceFaceID, targetFaceID));
}

const DualGraphNode& DualGraph::findDualGraphNode(const unsigned int sourceFaceID, const unsigned int targetFaceID) const{
  for(std::vector<DualGraphNode>::const_iterator it = (*this)[sourceFaceID].cbegin(); it != (*this)[sourceFaceID].cend(); ++it){
    if(it->getRef() == static_cast<int>(targetFaceID)){
      return *it;
    }
  }
  return DualGraphNode::invalidNode();
}

UnfoldTransformation DualGraph::findInverseUnfoldTransformation(const UnfoldTransformation& transformation) const{
  std::vector<DualGraphNode>::const_iterator foundIterator =
    std::find_if(
      (*this)[transformation.m_targetID].cbegin(),
      (*this)[transformation.m_targetID].cend(),
      [transformation](const DualGraphNode& node){
        return node.getUnfoldTransformation().m_targetID == transformation.m_sourceID;
      }
    );
  if(foundIterator == (*this)[transformation.m_targetID].cend()){
    return UnfoldTransformation();
  }
  return foundIterator->getUnfoldTransformation();
}

UnfoldTransformation DualGraph::findUnfoldTransformation(const int sourceID, const int targetID) const{
  std::vector<DualGraphNode>::const_iterator foundIterator =
    std::find_if(
      (*this)[sourceID].cbegin(), (*this)[sourceID].cend(),
      [targetID](const DualGraphNode& node){
        return node.getUnfoldTransformation().m_targetID == targetID;
      }
    );
  if(foundIterator == (*this)[sourceID].cend()){
    return UnfoldTransformation();
  }
  return foundIterator->getUnfoldTransformation();
}

void DualGraph::updateAllTransformations(){
  const Eigen::MatrixXd& vertices = m_mesh->getVertices();
  const Eigen::MatrixXd& faceNormals = m_mesh->getFaceNormals();
  for(iterator dualGraphIterator = begin(); dualGraphIterator != end(); ++dualGraphIterator){
    for(DualGraphNode& node: *dualGraphIterator){
      UnfoldTransformation& ut = node.getUnfoldTransformation();
      ut.rotation.setFromTwoVectors(faceNormals.row(ut.m_sourceID), faceNormals.row(ut.m_targetID));
      ut.translation = Eigen::Translation3d(vertices.row(ut.m_globalV0));
    }
  }
}

void DualGraph::updateTransformations(const unsigned int faceID){
  const Eigen::MatrixXd& vertices = m_mesh->getVertices();
  const Eigen::MatrixXd& faceNormals = m_mesh->getFaceNormals();
  for(DualGraphNode& currentNode: (*this)[faceID]){
    UnfoldTransformation& ut = currentNode.getUnfoldTransformation();
    ut.rotation.setFromTwoVectors(faceNormals.row(ut.m_sourceID), faceNormals.row(ut.m_targetID));
    ut.translation = Eigen::Translation3d(vertices.row(ut.m_globalV0));
    for(DualGraphNode& neighborNode: (*this)[ut.m_targetID]){
      UnfoldTransformation& ot = neighborNode.getUnfoldTransformation();
      if(ot.m_targetID == ut.m_sourceID){
        ot.rotation.setFromTwoVectors(faceNormals.row(ot.m_sourceID), faceNormals.row(ot.m_targetID));
        ot.translation = Eigen::Translation3d(vertices.row(ot.m_globalV0));
        break;
      }
    }
  }
}

// Private

void DualGraph::addEntry(unsigned int faceID, const std::unordered_map<unsigned int, std::unordered_map<unsigned int, unsigned int>>& edgeMap, bool withTransformations){
  const Eigen::MatrixXd& vertices = m_mesh->getVertices();
  const std::vector<unsigned int>& currentFace = m_mesh->getFaces()[faceID];
  const Eigen::MatrixXd& faceNormals = m_mesh->getFaceNormals();
  for(size_t i = 0; i < currentFace.size(); ++i){
    int localV0 = i;
    int localV1 = (i + 1) % currentFace.size();
    unsigned int globalV0 = currentFace[localV0];
    unsigned int globalV1 = currentFace[localV1];
    const unsigned int neighborID = edgeMap.at(globalV1).at(globalV0);
    if(isAlreadyEnlisted(faceID, neighborID)){
      continue;
    }
    DualGraphNode n0;
    n0.setRef(neighborID);
    if(withTransformations){
      UnfoldTransformation t0 = UnfoldTransformation(faceID, neighborID, globalV0, globalV1);
      t0.rotation.setFromTwoVectors(faceNormals.row(faceID), faceNormals.row(neighborID));
      t0.translation = Eigen::Translation3d(vertices.row(globalV0));
      n0.updateUnfoldTransformation(std::move(t0));
    }

    DualGraphNode n1;
    n1.setRef(faceID);
    if(withTransformations){
      UnfoldTransformation t1 = UnfoldTransformation(neighborID, faceID, globalV1, globalV0);
      t1.rotation.setFromTwoVectors(faceNormals.row(neighborID), faceNormals.row(faceID));
      t1.translation = Eigen::Translation3d(vertices.row(globalV1));
      n1.updateUnfoldTransformation(std::move(t1));
    }

    (*this)[faceID].push_back(n0);
    (*this)[neighborID].push_back(n1);
  }
}

std::unordered_map<unsigned int, std::unordered_map<unsigned int, unsigned int>> DualGraph::createEdgeMap(){
  // StartVertex, EndVertex, FaceIndex
  std::unordered_map<unsigned int, std::unordered_map<unsigned int, unsigned int>> edgeMap(3 * m_mesh->getFaces().size());
  for(size_t i = 0; i < m_mesh->getFaces().size(); ++i){
    const std::vector<unsigned int>& currentFace = m_mesh->getFaces()[i];
    for(size_t j = 0; j < currentFace.size(); ++j){
      edgeMap[currentFace[j]][currentFace[(j + 1) % currentFace.size()]] = i;
    }
  }
  return edgeMap;
}

bool DualGraph::isAlreadyEnlisted(const unsigned int currentFaceID, const unsigned int neighborFaceID){
  return std::find_if(
           (*this)[currentFaceID].cbegin(),
           (*this)[currentFaceID].cend(),
           [neighborFaceID](const DualGraphNode& n){
             return n.getRef() == static_cast<int>(neighborFaceID);
           }
         ) != (*this)[currentFaceID].cend();
}
