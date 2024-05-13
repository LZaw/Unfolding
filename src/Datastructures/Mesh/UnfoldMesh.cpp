
#include "UnfoldMesh.hpp"

// C++ Headers
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
#include <queue>
#include <unordered_map>
#include <utility>

// Project Headers
#include <Datastructures/Mesh/OriginalMesh.hpp>
#include <Util/PolygonUtil.hpp>

// Mesh

UnfoldMesh::UnfoldMesh():
  Mesh(),
  m_dualGraph(this){
}

// Public

const Eigen::VectorXd& UnfoldMesh::calculateAngleDeficit(){
  m_angleDeficit.resize(m_vertices.rows());
  m_angleDeficit.setConstant(2. * M_PI);
  for(FaceList::const_iterator faceIterator = m_faces.cbegin(); faceIterator != m_faces.cend(); ++faceIterator){
    const std::vector<unsigned int>& currentFace = *faceIterator;
    std::vector<Eigen::Vector3d> edges(currentFace.size());
    for(size_t j = 0; j < currentFace.size(); ++j){
      edges[j] = m_vertices.row(currentFace[(j + 1) % currentFace.size()]) - m_vertices.row(currentFace[j]);
    }
    for(size_t j = 0; j < currentFace.size(); ++j){
      m_angleDeficit[currentFace[j]] -= std::acos(edges[j].dot(-edges[(j - 1 + currentFace.size()) % currentFace.size()]) / (edges[j].norm() * edges[(j - 1 + currentFace.size()) % currentFace.size()].norm()));
    }
  }
  return m_angleDeficit;
}

const Eigen::VectorXd& UnfoldMesh::calculateFaceAreas(){
  calculateFaceAreaNormals();
  m_faceAreas = m_unnormalizedFaceNormals.rowwise().norm();
  return m_faceAreas;
}

int UnfoldMesh::calculateGenus(){
  unsigned int nV = 0;
  unsigned int nF = 0;
  unsigned int nE = 0;
  std::unordered_map<int, std::unordered_map<int, bool>> alreadyVisited;
  for(FaceList::const_iterator faceIterator = m_faces.cbegin(); faceIterator != m_faces.cend(); ++faceIterator){
    const std::vector<unsigned int>& currentFace = *faceIterator;
    ++nF;
    for(size_t j = 0; j < currentFace.size(); ++j){
      int i0 = currentFace[j];
      int i1 = currentFace[(j + 1) % currentFace.size()];
      if(!alreadyVisited[i0][i1]){
        ++nE;
        alreadyVisited[i0][i1] = true;
        alreadyVisited[i1][i0] = true;
      }
    }
  }
  nV = alreadyVisited.size();
  // V - E + F = 2 - 2g -> g = (2 - V + E - F) / 2
  m_genus = (-nV + nE - nF + 2) / 2;
  if(m_genus < 0){
    std::cerr << "Genus < 0. The mesh might be damaged." << std::endl;
  }
  return m_genus;
}

const Eigen::VectorXd& UnfoldMesh::calculateMeanCurvature(){
  Eigen::SparseMatrix<double> Minv(m_vertexMassMatrix.rows(), m_vertexMassMatrix.cols());
  m_meanCurvature.resize(m_vertices.rows());
  Minv.setIdentity();
  Minv.diagonal() = m_vertexMassMatrix.diagonal().array().inverse();
  // cotMatrix is (cot(a) + cot(b)) / 2
  // -> LB = 0.5 * Minv * L;
  // Laplace-Beltrami * V = -2HN -> -1/2 * Laplace-Beltrami * V = HN
  const Eigen::MatrixXd HN(-1./4. * Minv * m_cotMatrix * m_vertices);
  for(int i = 0; i < m_meanCurvature.rows(); ++i){
    m_meanCurvature[i] = std::copysign(HN.row(i).norm(), m_vNormals.row(i).dot(HN.row(i)));
  }
  return m_meanCurvature;
}

void UnfoldMesh::calculateSumPointsPerFace(){
  m_sumPointsPerFace = 0;
  for(FaceList::const_iterator faceIterator = m_faces.cbegin(); faceIterator != m_faces.cend(); ++faceIterator){
    m_sumPointsPerFace += faceIterator->size();
  }
}

void UnfoldMesh::createDualGraph(bool withTransformations){
  m_dualGraph.create(withTransformations);
}

void UnfoldMesh::drop(){
  m_angleDeficit.resize(0);
  m_cotMatrix.setZero();
  m_dualGraph.clear();
  m_faceAreas.resize(0);
  m_facesPerVertex.clear();
  m_gaussianCurvature.resize(0);
  m_meanCurvature.resize(0);
  m_sumPointsPerFace = 0;
  m_vertexMassMatrix.resize(0, 0);
}

double UnfoldMesh::getAverageDualValence() const{
  return static_cast<double>(m_sumPointsPerFace) / m_faces.size();
}

const LaplaceMatrix& UnfoldMesh::getCotMatrix() const{
  return m_cotMatrix;
}

const DualGraph& UnfoldMesh::getDualGraph() const{
  return m_dualGraph;
}

const Eigen::VectorXd& UnfoldMesh::getFaceAreas() const{
  return m_faceAreas;
}

const std::vector<std::vector<unsigned int>>& UnfoldMesh::getFacesPerVertex() const{
  return m_facesPerVertex;
}

int UnfoldMesh::getGenus() const{
  return m_genus;
}

const Eigen::VectorXd& UnfoldMesh::getMeanCurvature() const{
  return m_meanCurvature;
}

unsigned int UnfoldMesh::getSumPointsPerFace() const{
  return m_sumPointsPerFace;
}

const MassMatrix& UnfoldMesh::getVertexMassMatrix() const{
  return m_vertexMassMatrix;
}

void UnfoldMesh::initializeMembers(){
  postReadInitialization();
}

void UnfoldMesh::normalizeSurfaceArea(){
  // Scale to unit surface area
  calculateFaceAreas();
  double area = m_faceAreas.sum();
  m_vertices /= sqrt(area);
}

void UnfoldMesh::planarize(){
  if(isTriangular()){
    return;
  }
  double eps = std::numeric_limits<float>::epsilon();
  long unsigned int maxIterations = 500;
  bool todo = true;
  Eigen::MatrixXd flowPower = Eigen::MatrixXd::Zero(m_vertices.rows(), m_vertices.cols());
  Eigen::Vector3d fNormal = Eigen::Vector3d::Zero();
  Eigen::Vector3d diffVector = Eigen::Vector3d::Zero();
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::VectorXd normalizeVector(m_vertices.rows());
  double greatestEps = 1;
  long unsigned int iterations = 0;
  while(todo && !(iterations > maxIterations && greatestEps < eps * 1e4)){
    todo = false;
    m_vertices += flowPower;
    greatestEps = -1;
    flowPower.setZero();
    normalizeVector.setZero();
    calculateNormals();
    for(FaceList::const_iterator faceIterator = m_faces.cbegin(); faceIterator != m_faces.cend(); ++faceIterator){
      const std::vector<unsigned int>& currentFace = *faceIterator;
      center.setZero();
      for(std::vector<unsigned int>::const_iterator vertexIndexIterator = faceIterator->cbegin(); vertexIndexIterator != faceIterator->cend(); ++vertexIndexIterator){
        center += m_vertices.row(*vertexIndexIterator);
      }
      center /= static_cast<double>(currentFace.size());
      fNormal = m_fNormals.row(std::distance(m_faces.cbegin(), faceIterator));
      for(std::vector<unsigned int>::const_iterator vertexIndexIterator = faceIterator->cbegin(); vertexIndexIterator != faceIterator->cend(); ++vertexIndexIterator){
        diffVector = (center - Eigen::Vector3d(m_vertices.row(*vertexIndexIterator)));
        double dotProduct = diffVector.dot(fNormal.normalized());
        if(fabs(dotProduct) > eps){
          greatestEps = std::max(dotProduct, greatestEps);
          flowPower.row(*vertexIndexIterator) += dotProduct * fNormal.normalized();
          normalizeVector[*vertexIndexIterator] += 1;
          todo = true;
        }
      }
    }
    for(int i = 0; i < m_vertices.rows(); ++i){
      if(normalizeVector[i] != 0){
        flowPower.row(i) /= normalizeVector[i];
      }
    }
    ++iterations;
  }
  calculateNormals();
  updateAllTransformations();
}

void UnfoldMesh::setupCotMatrix(){
  m_cotMatrix.setup(m_vertices, m_faces, LaplaceMatrix::LaplaceMatrixType::COT);
}

void UnfoldMesh::setupFacesPerVertex(){
  m_facesPerVertex.clear();
  m_facesPerVertex.resize(m_vertices.rows());
  for(std::vector<std::vector<unsigned int>>::iterator fpvIterator = m_facesPerVertex.begin(); fpvIterator != m_facesPerVertex.end(); ++fpvIterator){
    // Typically 6 in a triangle mesh
    (*fpvIterator).reserve(8);
  }
  for(FaceList::const_iterator faceIterator = m_faces.cbegin(); faceIterator != m_faces.cend(); ++faceIterator){
    for(std::vector<unsigned int>::const_iterator vertexIndexIterator = faceIterator->cbegin(); vertexIndexIterator != faceIterator->cend(); ++vertexIndexIterator){
      m_facesPerVertex[*vertexIndexIterator].push_back(std::distance(m_faces.cbegin(), faceIterator));
    }
  }
}

void UnfoldMesh::setupVertexMassMatrix(const MassMatrix::MassMatrixType type){
  m_vertexMassMatrix.setup(m_vertices, m_faces, type);
}

void UnfoldMesh::updateAllTransformations(){
  m_dualGraph.updateAllTransformations();
}

void UnfoldMesh::updateMeanCurvature(bool updateCotMatrix){
  if(updateCotMatrix){
    setupCotMatrix();
  }
  setupVertexMassMatrix();
  calculateNormals();
  calculateMeanCurvature();
}

// Protected

void UnfoldMesh::postReadInitialization(){
  shiftBoundingBoxCenterToOrigin();
  calculateSumPointsPerFace();
  setupFacesPerVertex();
  calculateGenus();
  planarize();
  createDualGraph();
  calculateIsTriangular();
  if(m_isTriangular){
    setupCotMatrix();
    updateMeanCurvature();
  }
}

// Private

void UnfoldMesh::updateFaceNormal(const unsigned int faceID){
  if(m_faces.size() <= faceID){
    std::cerr << "Face not found." << std::endl;
    return;
  }
  if(m_fNormals.rows() < static_cast<int>(m_faces.size())){
    std::cerr << "Create normal fist, before updating." << std::endl;
    return;
  }
  calculateFaceAreaNormal(faceID);
  m_fNormals.row(faceID) = m_unnormalizedFaceNormals.row(faceID).normalized();
}

void UnfoldMesh::updateFaceNormalsAroundVertex(const unsigned int vertexID){
  for(std::vector<unsigned int>::const_iterator it = m_facesPerVertex[vertexID].cbegin(); it != m_facesPerVertex[vertexID].cend(); ++it){
    updateFaceNormal(*it);
  }
}
