
#include "MassSpringTransformer.hpp"

// C++ Headers
#include <cmath>
#include <limits>
#include <random>
#include <tuple>
#include <vector>

// Project Headers
#include <Util/DDGUtil.hpp>

MassSpringTransformer::MassSpringTransformer(UnfoldMesh* const mesh):
  GenericTransformer(mesh){
  initialize();
}

// Public

double MassSpringTransformer::getMaximalStepSize() const{
  return m_maximalStepSize;
}

double MassSpringTransformer::getMinimalStepSize() const{
  return 0.1;
}

double MassSpringTransformer::getOptimalStepSize() const{
  return m_maximalStepSize;
}

bool MassSpringTransformer::performStep(const double stepSize){
  setupLinearSystem(stepSize);
  solveSystemAndUpdateMesh();
  calculateEnergy();
  return true;
}

void MassSpringTransformer::updateProperties(){
  m_solver.compute(m_mesh->getCotMatrix());
  std::tie(std::ignore, m_maximalStepSize) = DDGUtil::calculateLargestEigenpair(m_mesh->getCotMatrix());
  m_maximalStepSize = std::fabs(m_maximalStepSize);
  calculateEnergy();
}

// Protected

double MassSpringTransformer::calculateEnergy(){
  // Euler Charakteristik: X(S) = 2 - 2g,
  // Integral über Gaußkrümmung: I(K) = 2PI * X(S)
  // I(K) / 4 = (2 - 2g) * 2PI / 4 -> (1 - g) * PI
  m_currentEnergy = m_mesh->getMeanCurvature().transpose() * m_mesh->getVertexMassMatrix() * m_mesh->getMeanCurvature() -
                   (1. - m_mesh->getGenus()) * M_PI;
  return m_currentEnergy;
}

void MassSpringTransformer::setupLinearSystem(const double stepSize){
  // RHS
  const Eigen::SparseMatrix<double>& M = m_mesh->getVertexMassMatrix();
  m_rhs = stepSize * M * m_mesh->getVertexNormals();
}

void MassSpringTransformer::solveSystemAndUpdateMesh(){
  m_mesh->setVertices(m_mesh->getVertices() - m_solver.solve(m_rhs));
  m_mesh->shiftAndNormalize();
  m_mesh->updateMeanCurvature();
}