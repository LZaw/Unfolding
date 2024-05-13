
#include "CollisionSolverHolisticBase.hpp"

CollisionSolverHolisticBase::CollisionSolverHolisticBase(Model2D* model2D, Model3D* model3D, Unfolder* unfolder):
  CollisionSolverBase(&unfolder->getUnfoldTreeList(), &model2D->getPolygonListRef(), &model2D->getCollisionDetector()),
  m_mesh(model3D->getUnfoldMesh()),
  m_model2D(model2D),
  m_model3D(model3D),
  m_unfolder(unfolder){
}

// Public

CollisionSolver2DBase* CollisionSolverHolisticBase::getSolver(){
  return m_collisionSolver.get();
}

// Protected 

bool CollisionSolverHolisticBase::isTimeLeft(const unsigned long seconds) const{
  return getRuntime() < seconds;
}
