
#include "MassMatrix.hpp"

// C++ Headers
#include <algorithm>
#include <array>
#include <iterator>
#include <utility>

// Project Headers
#include <Util/DDGUtil.hpp>

// Public

void MassMatrix::setup(const Eigen::MatrixXd& vertices, const FaceList& faces, const MassMatrixType type){
  setZero();
  std::vector<Eigen::Triplet<double>> triplets;
  size_t reserve = 0;
  for(FaceList::const_iterator faceIterator = faces.cbegin(); faceIterator != faces.cend(); ++faceIterator){
    reserve += faceIterator->size();
  }
  switch(type){
  case MassMatrixType::BARYCENTRIC:
  case MassMatrixType::SURROUNDINGTRIANGLES:{
    triplets.reserve(reserve);
    for(FaceList::const_iterator faceIterator = faces.cbegin(); faceIterator != faces.cend(); ++faceIterator){
      const std::vector<unsigned int>& currentFace = *faceIterator;
      Eigen::Vector3d e0 = vertices.row(currentFace[1]) - vertices.row(currentFace[0]);
      Eigen::Vector3d e1 = vertices.row(currentFace[2]) - vertices.row(currentFace[0]);
      double area;
      switch(type){
      case MassMatrixType::BARYCENTRIC:{
        // 1/3 of triangleareas
        area = e0.cross(e1).norm() / 6.;
        break;
      }
      case MassMatrixType::SURROUNDINGTRIANGLES:{
        // full triangleareas
        area = e0.cross(e1).norm() / 2.;
        break;
      }
      default:{
        area = 0;
        break;
      }
      }
      for(std::vector<unsigned int>::const_iterator vertexIt = faceIterator->cbegin(); vertexIt != faceIterator->cend(); ++vertexIt){
        triplets.push_back({static_cast<int>(*vertexIt), static_cast<int>(*vertexIt), area});
      }
    }
    break;
  }
  case MassMatrixType::MIXEDVORONOI:
  case MassMatrixType::CLIPPEDVORONOI:
  case MassMatrixType::PROPORTIONALVORONOI:
  case MassMatrixType::VORONOI:{
    triplets.reserve(reserve * 3);
    for(FaceList::const_iterator faceIterator = faces.cbegin(); faceIterator != faces.cend(); ++faceIterator){
      const std::vector<unsigned int>& currentFace = *faceIterator;
      std::vector<Eigen::Vector3d> edges = DDGUtil::calculateEdges(vertices, currentFace);
      std::vector<double> cotHalfs = DDGUtil::calculateCotHalfs(edges);
      int negativeIndex = -1;
      for(std::vector<double>::const_iterator cotIt = cotHalfs.cbegin(); cotIt != cotHalfs.cend(); ++cotIt){
        if(*cotIt < 0){
          negativeIndex = std::distance(cotHalfs.cbegin(), cotIt);
        }
      }
      bool allPositive = negativeIndex < 0;
      if(allPositive){
        // Umkresimittelpunkt innerhalb des Dreiecks -> Voronoi
        std::vector<Eigen::Triplet<double>> triangleTriplets = calculateVoronoiAreas(currentFace, edges, cotHalfs);
        std::move(std::begin(triangleTriplets), std::end(triangleTriplets), std::back_inserter(triplets));
      }else{
        // Umkreismittelpunkt außerhalb des Dreiecks
        double triangleArea = edges[0].cross(-edges[2]).norm() / 2.;
        switch(type){
        case MassMatrixType::VORONOI:{
          // Trotzdem Voroni
          std::vector<Eigen::Triplet<double>> triangleTriplets = calculateVoronoiAreas(currentFace, edges, cotHalfs);
          std::move(std::begin(triangleTriplets), std::end(triangleTriplets), std::back_inserter(triplets));
          break;
        }
        case MassMatrixType::MIXEDVORONOI:{
          // Hybrid
          double quadArea = (0.5 * edges[negativeIndex]).cross((0.5 * -edges[(negativeIndex + 2) % 3])).norm();
          double rest = triangleArea - quadArea;
          triplets.push_back({static_cast<int>(currentFace[negativeIndex]), static_cast<int>(currentFace[negativeIndex]), quadArea});
          triplets.push_back({static_cast<int>(currentFace[(negativeIndex + 1) % 3]), static_cast<int>(currentFace[(negativeIndex + 1) % 3]), rest / 2.});
          triplets.push_back({static_cast<int>(currentFace[(negativeIndex + 2) % 3]), static_cast<int>(currentFace[(negativeIndex + 2) % 3]), rest / 2.});
          break;
        }
        case MassMatrixType::CLIPPEDVORONOI:{
          // Clipped
          double remainingTriangleArea = triangleArea;
          // PointIndex, EdgeIndex
          std::array<std::pair<int, int>, 2> adjacentIndices = {std::pair{(negativeIndex + 2) % 3, (negativeIndex + 2) % 3},
                                                                std::pair{(negativeIndex + 1) % 3, negativeIndex}};
          // subTriangleArea = Tan(a) * |e/2| * |e/2| / 2
          // Tan(a) = 1 / (2 * cothalf)
          for(const std::pair<int, int>& indexPair: adjacentIndices){
            double subTriangleArea = (edges[indexPair.second] / 2.).squaredNorm() / (4. * cotHalfs[indexPair.first]);
            remainingTriangleArea -= subTriangleArea;
            triplets.push_back({static_cast<int>(currentFace[indexPair.first]), static_cast<int>(currentFace[indexPair.first]), subTriangleArea});
          }
          triplets.push_back({static_cast<int>(currentFace[negativeIndex]), static_cast<int>(currentFace[negativeIndex]), remainingTriangleArea});
          break;
        }
        case MassMatrixType::PROPORTIONALVORONOI:{
          // Proportional Voronoi
          for(std::vector<double>::iterator cotIt = cotHalfs.begin(); cotIt != cotHalfs.end(); ++cotIt){
            (*cotIt) = std::fabs(*cotIt);
          }
          std::vector<Eigen::Triplet<double>> triangleTriplets = calculateVoronoiAreas(currentFace, edges, cotHalfs);
          double summedPartialAreas = 0.;
          for(std::vector<Eigen::Triplet<double>>::const_iterator tripletIt = triangleTriplets.cbegin(); tripletIt != triangleTriplets.cend(); ++tripletIt){
            summedPartialAreas += tripletIt->value();
          }
          double factor = triangleArea / summedPartialAreas;
          std::transform(
            triangleTriplets.cbegin(),
            triangleTriplets.cend(),
            triangleTriplets.begin(),
            [factor](const Eigen::Triplet<double>& t){
              return Eigen::Triplet<double>(t.row(), t.col(), t.value() * factor);
            }
          );
          std::move(std::begin(triangleTriplets), std::end(triangleTriplets), std::back_inserter(triplets));
          break;
        }
        default:{
          break;
        }
        }
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

// Private

std::vector<Eigen::Triplet<double>> MassMatrix::calculateVoronoiAreas(const std::vector<unsigned int>& currentFace, const std::vector<Eigen::Vector3d>& edges, const std::vector<double>& cotHalfs){
  std::vector<Eigen::Triplet<double>> triplets;
  for(size_t i = 0; i < currentFace.size(); ++i){
    int i1 = (i + 1) % 3;
    int i2 = (i + 2) % 3;
    const Eigen::Vector3d& edge = edges[i1];
    // Dreiecksfläche: Cot / 2. * e² / 2. = e * h / 2.
    // Cot / 2 hat ein e im Nenner -> e²
    // Vertexanteile: Halbe Fläche
    double partialTriangleArea = edge.squaredNorm() * cotHalfs[i] * 0.25;
    triplets.push_back({static_cast<int>(currentFace[i1]), static_cast<int>(currentFace[i1]), partialTriangleArea});
    triplets.push_back({static_cast<int>(currentFace[i2]), static_cast<int>(currentFace[i2]), partialTriangleArea});
  }
  return triplets;
}
