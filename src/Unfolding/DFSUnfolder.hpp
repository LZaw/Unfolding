
# pragma once

// Project Headers
#include <Datastructures/Mesh/Mesh.hpp>
#include <Datastructures/Unfolding/UnfoldTree.hpp>
#include <Datastructures/Unfolding/UnfoldTreeList.hpp>

// Eigen Headers
#include <Eigen/Dense>

class DFSUnfolder{
public:
  DFSUnfolder();

  UnfoldTree* createNewUnfoldTree(const UnfoldMesh* mesh, UnfoldTreeList& unfoldTreeList);
};
