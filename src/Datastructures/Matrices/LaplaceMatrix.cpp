
#include "LaplaceMatrix.hpp"

// C++ Headers
#include <algorithm>

// Project Headers
#include <Util/DDGUtil.hpp>

// Public

void LaplaceMatrix::setup(const Eigen::MatrixXd& vertices, const FaceList& faces, LaplaceMatrixType type){
  setZero();
  std::vector<Eigen::Triplet<double>> triplets;
  switch(type){
  case LaplaceMatrixType::TOPOLOGICAL:{
    // Valence of 6, 4 triplets per vertex-vertex pair
    triplets.reserve(vertices.rows() * 24);
    std::unordered_map<int, std::unordered_map<int, bool>> visited;
    visited.reserve(vertices.rows());
    for(FaceList::const_iterator faceIterator = faces.cbegin(); faceIterator != faces.cend(); ++faceIterator){
      const std::vector<unsigned int>& currentFace = *faceIterator;
      for(size_t j = 0; j < currentFace.size(); ++j){
        int i0 = currentFace[(j + 0) % 3];
        int i1 = currentFace[(j + 1) % 3];
        if(!visited[i0][i1]){
          triplets.push_back(Eigen::Triplet<double>(i0, i1, 1));
          triplets.push_back(Eigen::Triplet<double>(i1, i0, 1));
          triplets.push_back(Eigen::Triplet<double>(i0, i0, -1));
          triplets.push_back(Eigen::Triplet<double>(i1, i1, -1));
          visited[i0][i1] = true;
          visited[i1][i0] = true;
        }
      }
    }
    break;
  }
  case LaplaceMatrixType::COT:
  case LaplaceMatrixType::CLIPPEDCOT:{
    // Calculates (cota + cotb) / 2.
    // Valence of 6, 4 triplets per vertex-vertex pair, 1 triplet per vertex
    triplets.reserve(vertices.rows() * 25);
    for(FaceList::const_iterator faceIterator = faces.cbegin(); faceIterator != faces.cend(); ++faceIterator){
      const std::vector<unsigned int>& currentFace = *faceIterator;
      std::vector<Eigen::Vector3d> edges = DDGUtil::calculateEdges(vertices, currentFace);
      std::vector<double> cotHalfs = DDGUtil::calculateCotHalfs(edges);
      for(size_t j = 0; j < currentFace.size(); ++j){
        int i1 = currentFace[(j + 1) % 3];
        int i2 = currentFace[(j + 2) % 3];
        double cotHalf = cotHalfs[j];
        if(type == LaplaceMatrixType::CLIPPEDCOT){
          cotHalf = std::max(-0.5, cotHalf);
        }
        triplets.push_back(Eigen::Triplet<double>(i1, i2, cotHalf));
        triplets.push_back(Eigen::Triplet<double>(i2, i1, cotHalf));
        triplets.push_back(Eigen::Triplet<double>(i1, i1, -cotHalf));
        triplets.push_back(Eigen::Triplet<double>(i2, i2, -cotHalf));
      }
    }
    break;
  }
  default:{
    break;
  }
  }
  resize(vertices.rows(), vertices.rows());
  setFromTriplets(triplets.begin(), triplets.end());
  makeCompressed();
}
