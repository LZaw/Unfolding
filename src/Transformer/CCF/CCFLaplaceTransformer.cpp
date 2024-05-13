
#include "CCFLaplaceTransformer.hpp"

// C++ Headers
#include <algorithm>
#include <cmath>
#include <iostream>
#include <tuple>

// Project Headers
#include <Util/DDGUtil.hpp>

CCFLaplaceTransformer::CCFLaplaceTransformer(UnfoldMesh* const m):
  CCFTransformer(m){
  updateProperties();
}

// Protected

double CCFLaplaceTransformer::calculateEnergy(){
  // Euler Charakteristik: X(S) = 2 - 2g,
  // Integral über Gaußkrümmung: I(K) = 2PI * X(S)
  // I(K) / 4 = (2 - 2g) * 2PI / 4 -> (1 - g) * PI
  m_currentEnergy = (m_mesh->getVertexMassMatrix() * m_mesh->getMeanCurvature()).sum() - (1. - m_mesh->getGenus()) * M_PI;
  return m_currentEnergy;
}

void CCFLaplaceTransformer::filter(){
  const Eigen::SparseMatrix<double>& M = m_mesh->getVertexMassMatrix();
  const Eigen::SparseMatrix<double> stepMatrix(m_cotMatrix);

  double filterStepSize = std::min(1. / (2. * m_largestEigenValue), m_sigma);
  size_t steps = std::ceil(m_sigma / filterStepSize);
  if(steps > 1e6){
    m_doFilter = false;
    std::cerr << "Stopped filtering" << std::endl;
  }else{
    filterStepSize = m_sigma / steps;
    m_filterSolver.factorize(M - filterStepSize * stepMatrix);
    Eigen::VectorXd filter(m_rho);
    for(size_t i = 0; i < steps; ++i){
      filter = m_filterSolver.solve(M * filter).eval();
    }
    m_rho -= filter;
    m_sigma += std::sqrt(m_sigma);
  }
}

void CCFLaplaceTransformer::updateFilterSolver(){
  m_cotMatrix = m_mesh->getCotMatrix();
  std::tie(std::ignore, m_largestEigenValue) = DDGUtil::calculateLargestEigenpair(m_cotMatrix);
  m_largestEigenValue = std::fabs(m_largestEigenValue);
  if(std::isnormal(m_largestEigenValue)){
    // Analyze Filterpattern
    m_filterSolver.analyzePattern(m_cotMatrix);
    m_sigma = 1. / (2. * m_largestEigenValue);
  }else{
    std::cerr << "Can't Filter." << std::endl;
    m_doFilter = false;
  }
}
