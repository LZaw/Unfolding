
#pragma once

// C++ Headers
#include <vector>

// Project Headers
#include <Datastructures/Matrices/LaplaceMatrix.hpp>
#include <Datastructures/Mesh/UnfoldMesh.hpp>
#include <Transformer/GenericTransformer.hpp>
#include <Transformer/CCF/DerivedQuaterniond.hpp>

// Eigen Headers
#include <Eigen/Dense>
#include <Eigen/Sparse>

class CCFTransformer : public GenericTransformer{
public:
  CCFTransformer() = delete;
  CCFTransformer(UnfoldMesh* const m);

  double getMaximalStepSize() const override;
  double getMinimalStepSize() const override;
  double getOptimalStepSize() const override;

  bool performStep(const double stepSize) override;

  void updateProperties() override;

private:
  void buildAndProjectOutConstraints();
  void buildPoissonProblem();

  void copyVerticesAndUpdateMesh();

  void mapQuaternionToBlockTriplets(const DerivedQuaterniond& q, int i, int j, std::vector<Eigen::Triplet<double>>& triplets);

  void orthornormalize(Eigen::MatrixXd& Matrix);

  void projectOutConstraints(const Eigen::MatrixXd& C);

  bool restorePositions();

  void setupEigenproblem();
  void setupRho(double stepSize);
  bool solveSmallestEigenproblem();

  void updateSolvers();

protected:
  LaplaceMatrix                   m_cotMatrix;
  bool                            m_doFilter = true;
  Eigen::SparseMatrix<double>     m_eigenMatrix;
  Eigen::SimplicialLDLT<
    Eigen::SparseMatrix<double>>  m_eigenSolver;
  Eigen::SimplicialLDLT<
    Eigen::SparseMatrix<double>>  m_filterSolver;
  double                          m_largestEigenValue = 0.;
  Eigen::VectorXd                 m_lambda;
  Eigen::MatrixXd                 m_newVertices;
  Eigen::MatrixXd                 m_omega;
  Eigen::VectorXd                 m_rho;
  Eigen::SimplicialLDLT<
    Eigen::SparseMatrix<double>>  m_positionSolver;
  double                          m_sigma = 0.;

  virtual void filter() = 0;

  virtual void updateFilterSolver() = 0;
};
