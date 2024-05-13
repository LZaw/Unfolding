
#pragma once

// C++ Headers
#include <vector>

// Project Headers
#include <Datastructures/Mesh/FaceList.hpp>

// Eigen Headers
#include <Eigen/Dense>
#include <Eigen/Sparse>

class LaplaceMatrix: public Eigen::SparseMatrix<double>{
public:
  enum class LaplaceMatrixType{
    TOPOLOGICAL,
    COT,
    CLIPPEDCOT
  };

  void setup(const Eigen::MatrixXd& vertices, const FaceList& faces, LaplaceMatrixType type);
};
