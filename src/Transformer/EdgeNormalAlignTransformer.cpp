
#include "EdgeNormalAlignTransformer.hpp"

// C++ Headers
#include <cmath>
#include <iostream>
#include <limits>

// Project Headers
#include <Datastructures/Mesh/FaceList.hpp>

EdgeNormalAlignTransformer::EdgeNormalAlignTransformer(UnfoldMesh* const mesh):
  GenericTransformer(mesh){
  initialize();
}

// Public

double EdgeNormalAlignTransformer::getMaximalStepSize() const{
  return 1 - std::numeric_limits<float>::epsilon();
}

double EdgeNormalAlignTransformer::getMinimalStepSize() const{
  return std::numeric_limits<float>::epsilon();
}

double EdgeNormalAlignTransformer::getOptimalStepSize() const{
  return getMaximalStepSize();
}

bool EdgeNormalAlignTransformer::performStep(const double stepSize){
  setupEdgeNormals();
  setupRhs(stepSize);
  solveSystemAndUpdateMesh();
  calculateEnergy();
  return true;
}

void EdgeNormalAlignTransformer::updateProperties(){
  m_cotMatrix = m_mesh->getCotMatrix();
  m_positionSolver.compute(m_cotMatrix);
  m_edgeNormals.reserve(m_mesh->getVertices().rows());
  m_rhs.resize(m_mesh->getVertices().rows(), m_mesh->getVertices().cols());
  setupIndexList();
  calculateEnergy();
}

// Protected

double EdgeNormalAlignTransformer::calculateEnergy(){
  m_currentEnergy = m_mesh->calculateAngleDeficit().squaredNorm() / m_mesh->getVertices().rows();
  return m_currentEnergy;
}

double EdgeNormalAlignTransformer::computeAngle(const std::pair<Eigen::Vector3d, double>& edge, const std::pair<Eigen::Vector3d, double>& otherEdge) const{
  return std::acos(std::get<0>(edge).dot(std::get<0>(otherEdge)) / (std::get<1>(edge) * std::get<1>(otherEdge)));
}

void EdgeNormalAlignTransformer::setupEdgeNormals(){
  m_edgeNormals.clear();
  m_edgeNormals.resize(m_mesh->getVertices().rows());
  const Eigen::MatrixXd& fNormals = m_mesh->getFaceNormals();
  const Eigen::VectorXd& faceAreas = m_mesh->calculateFaceAreas();
  for(Eigen::Index i = 0; i < m_mesh->getVertices().rows(); ++i){
    const std::unordered_map<unsigned int, std::vector<unsigned int>>& neighborIndices = m_indexList[i];
    for(std::unordered_map<unsigned int, std::vector<unsigned int>>::const_iterator it = neighborIndices.cbegin(); it != neighborIndices.cend(); ++it){
      const unsigned int neighborIndex = it->first;
      const std::vector<unsigned int>& adjacentFaceIndices = it->second;
      const Eigen::Vector3d edgeNormal = (fNormals.row(adjacentFaceIndices[0]) * faceAreas[adjacentFaceIndices[0]] + fNormals.row(adjacentFaceIndices[1]) * faceAreas[adjacentFaceIndices[1]]).normalized();
      m_edgeNormals[i][neighborIndex] = edgeNormal;
    }
  }
}

void EdgeNormalAlignTransformer::setupIndexList(){
  m_indexList.clear();
  m_indexList.resize(m_mesh->getVertices().rows());
  const FaceList& faces = m_mesh->getFaces();
  for(FaceList::const_iterator faceIterator = faces.cbegin(); faceIterator != faces.cend(); ++faceIterator){
    const std::vector<unsigned int>& currentFace = *faceIterator;
    for(size_t j = 0; j < currentFace.size(); j++){
      unsigned int currentVertex = currentFace[j];
      unsigned int nextVertex = currentFace[(j + 1) % currentFace.size()];
      m_indexList[currentVertex][nextVertex].push_back(std::distance(faces.cbegin(), faceIterator));
      m_indexList[nextVertex][currentVertex].push_back(std::distance(faces.cbegin(), faceIterator));
    }
  }
}

void EdgeNormalAlignTransformer::setupRhs(const double stepSize){
  m_rhs.setZero();
  const Eigen::VectorXd& vertexMasses = m_mesh->getVertexMassMatrix().diagonal();
  const Eigen::MatrixXd& vertices = m_mesh->getVertices();
  const Eigen::MatrixXd& vNormals = m_mesh->getVertexNormals();
  for(Eigen::Index currentVertexID = 0; currentVertexID < vertices.rows(); ++currentVertexID){
    const Eigen::Vector3d& scaledVertexNormal = vNormals.row(currentVertexID) / vertexMasses[currentVertexID];
    for(std::unordered_map<unsigned int, Eigen::Vector3d>::const_iterator edgeIterator = m_edgeNormals[currentVertexID].cbegin(); edgeIterator != m_edgeNormals[currentVertexID].cend(); ++edgeIterator){
      const unsigned int neighborID = edgeIterator->first;
      const Eigen::Vector3d& edgeNormal = edgeIterator->second;
      const Eigen::Vector3d& scaledNeighborNormal = vNormals.row(neighborID) / vertexMasses[neighborID];
      const Eigen::Vector3d meanVertexNormal = (scaledVertexNormal + scaledNeighborNormal).normalized();
      const Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity().slerp(stepSize, Eigen::Quaterniond::FromTwoVectors(edgeNormal, meanVertexNormal));
      const Eigen::Vector3d edge = vertices.row(neighborID) - vertices.row(currentVertexID);
      m_rhs.row(currentVertexID) += m_cotMatrix.coeff(currentVertexID, neighborID) * (rotation * edge);
      m_rhs.row(neighborID) += m_cotMatrix.coeff(neighborID, currentVertexID) * (rotation * -edge);
    }
  }
}

void EdgeNormalAlignTransformer::solveSystemAndUpdateMesh(){
  m_mesh->setVertices(m_positionSolver.solve(m_rhs));
  m_mesh->shiftAndNormalize();
  m_mesh->setupVertexMassMatrix();
  m_mesh->calculateNormals();
}