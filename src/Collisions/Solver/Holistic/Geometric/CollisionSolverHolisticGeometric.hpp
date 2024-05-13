
#pragma once

// C++ Headers
#include <memory>
#include <vector>

// Project Headers
#include <Collisions/Solver/Holistic/CollisionSolverHolisticBase.hpp>
#include <Collisions/Solver/2D/CollisionSolver2DBase.hpp>
#include <Datastructures/Mesh/UnfoldMesh.hpp>
#include <Model/Model2D.hpp>
#include <Model/Model3D.hpp>
#include <Transformer/GenericTransformer.hpp>
#include <Unfolding/Unfolder.hpp>

// Eigen Headers
#include <Eigen/Dense>

class CollisionSolverHolisticGeometric: public CollisionSolverHolisticBase{
public:
  CollisionSolverHolisticGeometric(Model2D* model2D, Model3D* model3D, Unfolder* unfolder, GenericTransformer* transformer);

  int solve(const unsigned long seconds = 0) override;

protected:
  Eigen::VectorXd           m_initialFaceAreas;
  Eigen::MatrixXd           m_lastVertices;
  std::vector<
    Eigen::MatrixXd>        m_timeline;
  std::unique_ptr<
    GenericTransformer>     m_transformer;

  void abort(bool restore = false) override;

  void cacheValidState();

  bool invTransformMesh(const unsigned long seconds) override;
  bool isTransformMeshConverged(const std::array<std::pair<double, double>, 2>& oldEnergies, const double currentEnergy, const double accumulatedStepSize);

  void restoreLastValidState();

  bool transformMesh(const unsigned long seconds) override;

  bool unfoldMesh(const unsigned long seconds) override;
};
