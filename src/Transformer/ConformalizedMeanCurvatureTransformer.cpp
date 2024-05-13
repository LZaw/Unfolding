
#include "ConformalizedMeanCurvatureTransformer.hpp"

// C++ Headers
#include <algorithm>
#include <iostream>
#include <limits>
#include <tuple>

// Project Headers
#include <Datastructures/Matrices/LaplaceMatrix.hpp>
#include <Datastructures/Matrices/MassMatrix.hpp>
#include <Util/DDGUtil.hpp>

// Eigen Headers
#include <Eigen/Dense>
#include <Eigen/Sparse>

ConformalizedMeanCurvatureTransformer::ConformalizedMeanCurvatureTransformer(UnfoldMesh* const mesh):
  GenericTransformer(mesh){
  if(m_mesh->getGenus() != 0){
    std::cerr << "Genus != 0, this function won't work well." << std::endl;
  }
  initialize();
}

// Public

double ConformalizedMeanCurvatureTransformer::getMaximalStepSize() const{
  return m_maximalStepSize;
}

double ConformalizedMeanCurvatureTransformer::getMinimalStepSize() const{
  double minimalStepSize = m_maximalStepSize * m_mesh->getAverageDualValence() / m_mesh->getFaces().size();
  return std::min(m_maximalStepSize, minimalStepSize);
}

double ConformalizedMeanCurvatureTransformer::getOptimalStepSize() const{
  return m_maximalStepSize;
}

bool ConformalizedMeanCurvatureTransformer::performStep(const double stepSize){
  // Get Matrices
  const MassMatrix& massMatrix = m_mesh->getVertexMassMatrix();

  // Solve System
  m_linearSolver.factorize(massMatrix - stepSize * m_cotMatrix);
  Eigen::MatrixXd newVertices = m_linearSolver.solve(massMatrix * m_mesh->getVertices());
  m_mesh->setVertices(newVertices);

  // Normalize Mesh
  m_mesh->shiftAndNormalize();
  m_mesh->updateMeanCurvature();

  // Update Energy
  calculateEnergy();

  return true;
}

void ConformalizedMeanCurvatureTransformer::updateProperties(){
  m_cotMatrix = m_mesh->getCotMatrix();
  double largestEigenvalue;
  std::tie(std::ignore, largestEigenvalue) = DDGUtil::calculateLargestEigenpair(m_cotMatrix);
  largestEigenvalue = std::fabs(largestEigenvalue);
  m_maximalStepSize = 1. / (2. * largestEigenvalue);
  m_maximalStepSize -= std::min(static_cast<double>(std::numeric_limits<float>::epsilon()), m_maximalStepSize / 1000.);
  m_linearSolver.analyzePattern(m_cotMatrix);
  calculateEnergy();
}

// Protected

double ConformalizedMeanCurvatureTransformer::calculateEnergy(){
  // Euler Charakteristik: X(S) = 2 - 2g,
  // Integral über Gaußkrümmung: I(K) = 2PI * X(S)
  // I(K) / 4 = (2 - 2g) * 2PI / 4 -> (1 - g) * PI
  m_currentEnergy = (m_mesh->getVertexMassMatrix() * m_mesh->getMeanCurvature()).sum() - (1. - m_mesh->getGenus()) * M_PI;
  return m_currentEnergy;
}
