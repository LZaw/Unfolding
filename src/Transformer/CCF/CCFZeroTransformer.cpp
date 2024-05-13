
#include "CCFZeroTransformer.hpp"

// C++ Headers
#include <algorithm>
#include <cmath>
#include <iostream>

// Project Headers
#include <Util/DDGUtil.hpp>

CCFZeroTransformer::CCFZeroTransformer(UnfoldMesh* const m):
  CCFTransformer(m){
  updateProperties();
}

// Protected

double CCFZeroTransformer::calculateEnergy(){
  m_currentEnergy = (m_mesh->getVertexMassMatrix() * m_mesh->getMeanCurvature()).sum() - (1. - m_mesh->getGenus()) * M_PI;
  return m_currentEnergy;
}

void CCFZeroTransformer::filter(){
}

void CCFZeroTransformer::updateFilterSolver(){
  m_doFilter = false;
}
