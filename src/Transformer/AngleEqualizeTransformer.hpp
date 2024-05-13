
#pragma once

// C++ Headeres
#include <unordered_map>
#include <utility>

// Project Headers
#include <Transformer/GenericTransformer.hpp>

class AngleEqualizeTransformer: public GenericTransformer{
public:
  AngleEqualizeTransformer(UnfoldMesh* const mesh);

  double getMaximalStepSize() const override;
  double getMinimalStepSize() const override;
  double getOptimalStepSize() const override;

  bool performStep(const double stepSize = 0) override;

  void updateProperties() override;

protected:
  void calculateEdgeMap();
  double calculateEnergy() override;
  void computeAndAddRotatedEntry(const size_t i, const size_t j, const double stepSize, const Eigen::VectorXd& weights);
  inline double computeAngle(const std::pair<Eigen::Vector3d, double>& edge, const std::pair<Eigen::Vector3d, double>& otherEdge);

  std::unordered_map<
    size_t, std::unordered_map<
      size_t, std::pair<
        Eigen::Vector3d, double>>>  m_edgeMap;
  Eigen::MatrixXd                   m_newVertices;
  Eigen::VectorXd                   m_normalizationFactors;
};
