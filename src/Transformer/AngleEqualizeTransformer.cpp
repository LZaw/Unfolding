
#include "AngleEqualizeTransformer.hpp"

// C++ Headers
#include <cmath>
#include <limits>

// Project Headers
#include <Datastructures/Mesh/DualGraph.hpp>
#include <Datastructures/Mesh/FaceList.hpp>

AngleEqualizeTransformer::AngleEqualizeTransformer(UnfoldMesh* const mesh):
  GenericTransformer(mesh){
  initialize();
}

// Public

double AngleEqualizeTransformer::getMaximalStepSize() const{
  return 1 - std::numeric_limits<float>::epsilon();
}

double AngleEqualizeTransformer::getMinimalStepSize() const{
  return 0.1;
}

double AngleEqualizeTransformer::getOptimalStepSize() const{
  return getMaximalStepSize();
}

bool AngleEqualizeTransformer::performStep(const double stepSize){
  m_normalizationFactors.setZero();
  m_newVertices.setZero();
  const Eigen::VectorXd faceAreas = m_mesh->calculateFaceAreas();
  const DualGraph& dualGraph = m_mesh->getDualGraph();
  for(DualGraph::const_iterator graphIterator = dualGraph.cbegin(); graphIterator != dualGraph.cend(); ++graphIterator){
    unsigned int i = std::distance(dualGraph.cbegin(), graphIterator);
    for(std::vector<DualGraphNode>::const_iterator nodeIterator = graphIterator->cbegin(); nodeIterator != graphIterator->cend(); ++nodeIterator){
      computeAndAddRotatedEntry(i, nodeIterator->getRef(), stepSize, faceAreas);
    }
  }
  m_newVertices.array().colwise() /= m_normalizationFactors.array();
  m_mesh->setVertices(m_newVertices);
  m_mesh->shiftAndNormalize();
  m_mesh->calculateNormals();
  calculateEnergy();
  return true;
}

void AngleEqualizeTransformer::updateProperties(){
  m_newVertices.resize(m_mesh->getVertices().rows(), m_mesh->getVertices().cols());
  m_newVertices.setZero();
  m_normalizationFactors.resize(m_mesh->getVertices().rows());
  m_normalizationFactors.setZero();
  m_edgeMap.reserve(m_mesh->getVertices().rows());
  calculateEnergy();
}

// Protected

void AngleEqualizeTransformer::calculateEdgeMap(){
  m_edgeMap.clear();
  const std::vector<std::vector<unsigned int>>& faces = m_mesh->getFaces();
  const Eigen::MatrixXd& vertices = m_mesh->getVertices();
  for(std::vector<std::vector<unsigned int>>::const_iterator faceIterator = faces.cbegin(); faceIterator != faces.cend(); ++faceIterator){
    const std::vector<unsigned int>& currentFace = *faceIterator;
    for(unsigned int j = 0; j < currentFace.size(); ++j){
      const unsigned int vertexID = currentFace[j];
      const unsigned int forwardID = currentFace[(j + 1) % currentFace.size()];
      const unsigned int backwardID = currentFace[(j - 1 + currentFace.size()) % currentFace.size()];
      if(m_edgeMap[vertexID].find(forwardID) == m_edgeMap[vertexID].end()){
        const Eigen::Vector3d vector = vertices.row(forwardID) - vertices.row(vertexID);
        const double norm = vector.norm();
        m_edgeMap[vertexID][forwardID] = {vector, norm};
        m_edgeMap[forwardID][vertexID] = {-vector, norm};
      }
      if(m_edgeMap[vertexID].find(backwardID) == m_edgeMap[vertexID].end()){
        const Eigen::Vector3d vector = vertices.row(backwardID) - vertices.row(vertexID);
        const double norm = vector.norm();
        m_edgeMap[vertexID][backwardID] = {vector, norm};
        m_edgeMap[backwardID][vertexID] = {-vector, norm};
      }
    }
  }
}

double AngleEqualizeTransformer::calculateEnergy(){
  calculateEdgeMap();
  const FaceList& faces = m_mesh->getFaces();
  const Eigen::MatrixXd& vertices = m_mesh->getVertices();
  double squaredAngles = 0.;
  for(FaceList::const_iterator faceIterator = faces.cbegin(); faceIterator != faces.cend(); ++faceIterator){
    const std::vector<unsigned int>& currentFace = *faceIterator;
    for(size_t j = 0; j < currentFace.size(); ++j){
      const unsigned int vertexID = currentFace[j];
      const unsigned int forwardID = currentFace[(j + 1) % currentFace.size()];
      const unsigned int backwardID = currentFace[(j - 1 + currentFace.size()) % currentFace.size()];
      const double angle = std::acos(std::get<0>(m_edgeMap[vertexID][forwardID]).dot(std::get<0>(m_edgeMap[vertexID][backwardID])) / (std::get<1>(m_edgeMap[vertexID][forwardID]) * std::get<1>(m_edgeMap[vertexID][backwardID])));
      squaredAngles += std::pow(angle, 2.);
    }
  }
  m_currentEnergy = squaredAngles / vertices.rows();
  return m_currentEnergy;
}

void AngleEqualizeTransformer::computeAndAddRotatedEntry(const size_t i, const size_t j, const double stepSize, const Eigen::VectorXd& weights){
  const DualGraph& dualGraph = m_mesh->getDualGraph();
  const std::vector<unsigned int>& currentFace = m_mesh->getFaces()[i];
  const std::vector<unsigned int>& otherFace = m_mesh->getFaces()[j];
  const UnfoldTransformation& forwardTransformation = dualGraph.findUnfoldTransformation(i, j);
  const UnfoldTransformation& inverseTransformation = dualGraph.findInverseUnfoldTransformation(forwardTransformation);

  const int vertexID = inverseTransformation.m_globalV1;

  const std::pair<Eigen::Vector3d, double>& mainEdge = m_edgeMap[inverseTransformation.m_globalV0][inverseTransformation.m_globalV1];
  const int currentLocalV0 = std::distance(currentFace.cbegin(), std::find(currentFace.cbegin(), currentFace.cend(), inverseTransformation.m_globalV0));
  const int otherLocalV0 = std::distance(otherFace.cbegin(), std::find(otherFace.cbegin(), otherFace.cend(), inverseTransformation.m_globalV0));
  const std::pair<Eigen::Vector3d, double>& currentEdge = m_edgeMap[inverseTransformation.m_globalV0][currentFace[(currentLocalV0 + 1 + currentFace.size()) % currentFace.size()]];
  const std::pair<Eigen::Vector3d, double>& otherEdge = m_edgeMap[inverseTransformation.m_globalV0][otherFace[(otherLocalV0 - 1 + otherFace.size()) % otherFace.size()]];
  const double currentAngle = computeAngle(mainEdge, currentEdge);
  const double otherAngle = computeAngle(mainEdge, otherEdge);
  const double angleDiff = (otherAngle - currentAngle) / 2.;

  const Eigen::AngleAxisd rotation(angleDiff * stepSize, (m_mesh->getVertexNormals().row(vertexID)).normalized());
  const Eigen::Vector3d rotatedVertex = rotation * std::get<0>(mainEdge) + Eigen::Vector3d(m_mesh->getVertices().row(inverseTransformation.m_globalV0));

  m_normalizationFactors[vertexID] += 1;
  m_newVertices.row(vertexID) += rotatedVertex;
}

double AngleEqualizeTransformer::computeAngle(const std::pair<Eigen::Vector3d, double>& edge, const std::pair<Eigen::Vector3d, double>& otherEdge){
  return std::acos(std::get<0>(edge).dot(std::get<0>(otherEdge)) / (std::get<1>(edge) * std::get<1>(otherEdge)));
}
