
#pragma once

// C++ Headers

// Project Headers
#include <Datastructures/Mesh/UnfoldMesh.hpp>

// Eigen Headers
#include <Eigen/Dense>

class GenericTransformer{
public:
  GenericTransformer(UnfoldMesh* const mesh);

  virtual double getCurrentEnergy() const;
  virtual double getMaximalStepSize() const = 0;
  virtual double getMinimalStepSize() const = 0;
  virtual double getOptimalStepSize() const = 0;

  void initialize();

  virtual bool performStep(const double stepSize = 0) = 0;

  virtual void updateProperties() = 0;

protected:
  double            m_currentEnergy = 0.;
  double            m_maximalStepSize = 0.;
  UnfoldMesh* const m_mesh = nullptr;

  virtual double calculateEnergy() = 0;
};
