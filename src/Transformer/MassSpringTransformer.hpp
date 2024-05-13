
#pragma once

// Project Headers
#include <Transformer/GenericTransformer.hpp>

class MassSpringTransformer: public GenericTransformer{
public:
  MassSpringTransformer(UnfoldMesh* const mesh);

  double getMaximalStepSize() const override;
  double getMinimalStepSize() const override;
  double getOptimalStepSize() const override;

  bool performStep(const double stepSize = 0) override;

  void updateProperties() override;

protected:
  double calculateEnergy() override;

  void setupLinearSystem(const double stepSize);
  void solveSystemAndUpdateMesh();

  Eigen::SimplicialLDLT<
    Eigen::SparseMatrix<
      double>>            m_solver;
  Eigen::MatrixXd         m_rhs;
};
