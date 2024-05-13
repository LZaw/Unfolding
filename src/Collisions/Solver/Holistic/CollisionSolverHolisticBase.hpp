
#pragma once

// Project Headers
#include <Collisions/Solver/CollisionSolverBase.hpp>
#include <Model/Model2D.hpp>
#include <Model/Model3D.hpp>
#include <Unfolding/Unfolder.hpp>

class CollisionSolverHolisticBase: public CollisionSolverBase{
public:
  CollisionSolverHolisticBase(Model2D* model2D, Model3D* model3D, Unfolder* unfolder);

  CollisionSolver2DBase* getSolver();

protected:
  std::unique_ptr<
    CollisionSolver2DBase>  m_collisionSolver;
  UnfoldMesh*               m_mesh;
  Model2D*                  m_model2D;
  Model3D*                  m_model3D;
  Unfolder*                 m_unfolder;

  virtual void abort(bool restore = false) = 0;

  virtual bool invTransformMesh(const unsigned long seconds) = 0;
  bool isTimeLeft(const unsigned long seconds) const;

  virtual bool transformMesh(const unsigned long seconds) = 0;

  virtual bool unfoldMesh(const unsigned long seconds) = 0;
};
