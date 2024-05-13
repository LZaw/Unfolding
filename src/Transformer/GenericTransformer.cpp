
#include "GenericTransformer.hpp"

GenericTransformer::GenericTransformer(UnfoldMesh* const mesh):
  m_mesh(mesh){
}

// Public

double GenericTransformer::getCurrentEnergy() const{
  return m_currentEnergy;
}

void GenericTransformer::initialize(){
  updateProperties();
}
