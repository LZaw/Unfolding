
#pragma once

// C++ Headers
#include <vector>

// Project Headers
#include <Datastructures/Helpers/UnfoldTransformation.hpp>
#include <Datastructures/Mesh/Mesh.hpp>
#include <Datastructures/Polygon/Polygon2D.hpp>
#include <Datastructures/Unfolding/UnfoldTreeList.hpp>

class UnfoldTree{
  friend class UnfoldTreeList;

public:
  UnfoldTree();
  UnfoldTree(const UnfoldTree* otherTree);

  void attachToNewParent(UnfoldTree* newParent);
  void attachToNewParent(UnfoldTree* newParent, UnfoldTree* newSubTreeRootNode);

  int calculateDistanceFromRoot() const;
  std::vector<unsigned int> calculatePossibleNeighbors(const int subTreeRootNodeIndex = -1) const;
  std::vector<unsigned int> calculateSubTreeIndices() const;
  unsigned int calculateTreeHeight() const;
  unsigned int calculateTreeSize() const;
  bool createNode(const int faceIndex, UnfoldTree* const parent, const UnfoldTransformation& transformation);

  void drop();

  const UnfoldTree* findConstTreeNodeByIndex(const int index) const;
  UnfoldTree* findTreeNodeByIndex(const int index);

  static const UnfoldTreeList* getUnfoldTreeList();

  bool hasLoops() const;

  bool isChildOf(const UnfoldTree* other) const;
  bool isConnectableTo(const int otherTreeNodeIndex) const;
  bool isConnectableTo(const int otherTreeNodeIndex, const int subTreeRootNodeIndex) const;
  bool isNeighborTo(const int otherTreeNodeIndex) const;

  static void setMesh(const UnfoldMesh* mesh);
  static void setUnfoldTreeList(UnfoldTreeList* unfoldTreeList);

  void traverseTreeToPolygonList(std::vector<Polygon2D>& polygonList, const Eigen::Affine3d& initialTransformation = Eigen::Affine3d::Identity(), bool useFaceIndicesAsListIndices = true) const;

  void updateNodeTransformation();
  void updateTreeTransformations();

  std::vector<
      UnfoldTree*>        m_children;
  UnfoldTree*             m_parent;
  int                     m_ref;
  UnfoldTransformation    m_transformation;

private:
  inline void connectToNewParent(UnfoldTree* newParent);

  inline void detachFromParent();

  void eraseChild(int childIndex);

  inline void setLinkToNewParent(UnfoldTree* newParent);

  static const UnfoldMesh* s_mesh;
  static UnfoldTreeList*   s_unfoldTreeList;
};
