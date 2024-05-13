
#pragma once

// C++ Headers
#include <vector>

// Project Headers
#include <Datastructures/Mesh/FaceList.hpp>

// Eigen Headers
#include <Eigen/Dense>
#include <Eigen/Sparse>

class MassMatrix : public Eigen::SparseMatrix<double>{
public:
  enum class MassMatrixType{
    BARYCENTRIC,
    SURROUNDINGTRIANGLES,
    MIXEDVORONOI,
    CLIPPEDVORONOI,
    PROPORTIONALVORONOI,
    VORONOI
  };

  void setup(const Eigen::MatrixXd& vertices, const FaceList& faces, const MassMatrixType type);

private:
  std::vector<Eigen::Triplet<double>> calculateVoronoiAreas(const std::vector<unsigned int>& currentFace, const std::vector<Eigen::Vector3d>& edges, const std::vector<double>& cotHalfs);
};
