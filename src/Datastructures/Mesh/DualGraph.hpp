
#pragma once

// C++ Headers
#include <unordered_map>
#include <vector>

// Project Headers
#include <Datastructures/Mesh/DualGraphNode.hpp>

// Eigen Headers
#include <Eigen/Dense>

class UnfoldMesh;
class DualGraph : public std::vector<std::vector<DualGraphNode>>{
public:
  DualGraph(const UnfoldMesh* const mesh);

  bool areNeighbors(const unsigned int sourceFaceID, const unsigned int targetFaceID) const;

  void create(bool withTransformations);

  DualGraphNode& findDualGraphNode(const unsigned int sourceFaceID, const unsigned int targetFaceID);
  const DualGraphNode& findDualGraphNode(const unsigned int sourceFaceID, const unsigned int targetFaceID) const;
  UnfoldTransformation findInverseUnfoldTransformation(const UnfoldTransformation& transformation) const;
  UnfoldTransformation findUnfoldTransformation(const int sourceID, const int targetID) const;

  void updateAllTransformations();
  void updateTransformations(const unsigned int faceID);

private:
  const UnfoldMesh* m_mesh;

  inline void addEntry(const unsigned int faceID, const std::unordered_map<unsigned int, std::unordered_map<unsigned int, unsigned int>>& edgeMap, bool withTransformations);

  inline std::unordered_map<unsigned int, std::unordered_map<unsigned int, unsigned int>> createEdgeMap();

  inline bool isAlreadyEnlisted(const unsigned int currentFaceID, const unsigned int neighborFaceID);
};
