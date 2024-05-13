
#pragma once

// Project Headers
#include <Datastructures/Matrices/LaplaceMatrix.hpp>
#include <Transformer/GenericTransformer.hpp>

// Eigen Headers
#include <Eigen/Sparse>

class ConformalizedMeanCurvatureTransformer: public GenericTransformer{
public:
  ConformalizedMeanCurvatureTransformer(UnfoldMesh* const mesh);

  double getMaximalStepSize() const override;
  double getMinimalStepSize() const override;
  double getOptimalStepSize() const override;

  bool performStep(const double stepSize = 0) override;

  void updateProperties() override;

protected:
  double calculateEnergy() override;

  LaplaceMatrix                   m_cotMatrix;
  Eigen::SimplicialLDLT<
    Eigen::SparseMatrix<double>>  m_linearSolver;
};
