
#pragma once

// C++ Headers
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

// Project Headers
#include <Transformer/GenericTransformer.hpp>

class EdgeNormalAlignTransformer: public GenericTransformer{
public:
  EdgeNormalAlignTransformer(UnfoldMesh* const mesh);

  double getMaximalStepSize() const override;
  double getMinimalStepSize() const override;
  double getOptimalStepSize() const override;

  bool performStep(const double stepSize = 0) override;

  void updateProperties() override;

protected:
  double calculateEnergy() override;
  inline double computeAngle(const std::pair<Eigen::Vector3d, double>& edge, const std::pair<Eigen::Vector3d, double>& otherEdge) const;

  void setupEdgeNormals();
  void setupIndexList();
  void setupRhs(const double stepSize);
  void solveSystemAndUpdateMesh();

  LaplaceMatrix                                   m_cotMatrix;
  std::vector<
    std::unordered_map<
      unsigned int, Eigen::Vector3d>>             m_edgeNormals;
  std::vector<
    std::unordered_map<
      unsigned int, std::vector<unsigned int>>>   m_indexList;
  Eigen::SimplicialLDLT<
    Eigen::SparseMatrix<double>>                  m_positionSolver;
  Eigen::MatrixXd                                 m_rhs;
};
