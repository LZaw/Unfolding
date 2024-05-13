
#pragma once

// C++ Headers
#include <memory>
#include <string>
#include <vector>

// Project Headers
#include <Datastructures/Mesh/UnfoldMesh.hpp>
#include <Datastructures/Polygon/Polygon2D.hpp>

class UnfoldTree;

class UnfoldTreeList: public std::vector<std::unique_ptr<UnfoldTree>>{
public:
  UnfoldTreeList(const UnfoldMesh* mesh);
  UnfoldTreeList(size_t size) = delete;

  void cacheCurrentTree();
  std::vector<UnfoldTree*> calculateMovableNodes();
  bool createTreeFromTreeStructure(const int faceID, int parent, const std::vector<std::vector<int>>& treeStructure);

  void drop();

  const UnfoldTree* findConstTreeNodeByIndex(const size_t index) const;
  UnfoldTree* findTreeNodeByIndex(const size_t index);

  UnfoldTree* getRoot();
  const UnfoldTree* getRoot() const;
  int getRootIndex() const;

  bool readTreeFromFile(const std::string& path);
  void reroot(const int newRootIndex);
  void resize(size_t newSize);
  void restoreCachedTree();

  void setRootIndex(int index);

  void traverseTreeToPolygonList(std::vector<Polygon2D>& polygonList, const Eigen::Affine3d& initialTransformation = Eigen::Affine3d::Identity(), bool useFaceIndicesAsListIndices = true) const;

  void updateTransformations();

  void writeFoldInstructionsToFile(const std::string& path) const;
  void writeTreeToFile(const std::string& path) const;

private:
  struct UnfoldTreeCache: public std::vector<int>{
    int m_rootNodeIndex = -1;
  };

  UnfoldTreeCache   m_unfoldTreeCache;
  const UnfoldMesh* m_mesh;
  int               m_rootNodeIndex;
};
