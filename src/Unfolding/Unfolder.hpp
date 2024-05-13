
#pragma once

// C++ Headers
#include <string>

// Project Headers
#include <Datastructures/Mesh/UnfoldMesh.hpp>
#include <Datastructures/Unfolding/UnfoldTree.hpp>
#include <Datastructures/Unfolding/UnfoldTreeList.hpp>

class Unfolder{
public:
enum class UnfoldMethod{
  BFS,
  DFS,
  MSTDiff,
  MSTQuotient,
  SteepestEdge,
  WeightedSteepestEdge,
};

  Unfolder(const UnfoldMesh* mesh);

  UnfoldTree* createNewUnfoldTree(const Eigen::VectorXd& weights = Eigen::VectorXd::Zero(1));

  void drop();

  UnfoldTree* getUnfoldTree();
  const UnfoldTree* getUnfoldTree() const;
  UnfoldTreeList& getUnfoldTreeList();
  const UnfoldTreeList& getUnfoldTreeList() const;

  void loadUnfoldTree(const std::string& path);

  void setUnfoldMethod(UnfoldMethod method);

  void rerootTree(const int newRootIndex);
  void resetTree();

  void saveFoldingInstructions(const std::string& path) const;
  void saveUnfoldTree(const std::string& path) const;

protected:
  void resizeUnfoldTreeList();

  UnfoldMethod                m_method = UnfoldMethod::SteepestEdge;
  const UnfoldMesh*           m_mesh = nullptr;
  UnfoldTreeList              m_unfoldTreeList;
};
