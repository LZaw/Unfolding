
#include "DFSUnfolder.hpp"

// C++ Headers
#include <cstdlib>
#include <stack>
#include <vector>

// Project Headers
#include <Datastructures/Mesh/FaceList.hpp>

DFSUnfolder::DFSUnfolder(){
}

// Public

UnfoldTree* DFSUnfolder::createNewUnfoldTree(const UnfoldMesh* mesh, UnfoldTreeList& unfoldTreeList){
  // Select Random Face for Root
  unsigned int initialFace = std::rand() % mesh->getFaces().size();

  // Create Root
  unfoldTreeList.setRootIndex(initialFace);
  UnfoldTree* rootNode = unfoldTreeList.getRoot();
  rootNode->createNode(initialFace, nullptr, UnfoldTransformation(initialFace, -1));

  // Set up DFS Datastrucutres
  std::vector<bool> inserted(mesh->getFaces().size(), false);
  inserted[initialFace] = true;
  std::stack<UnfoldTree*> nextTreeNodes;
  nextTreeNodes.push(rootNode);

  // DFS the DualGraph
  while(!nextTreeNodes.empty()){
    UnfoldTree* currentTreeNode = nextTreeNodes.top();
    nextTreeNodes.pop();

    const std::vector<DualGraphNode>& possibleChildren = mesh->getDualGraph()[currentTreeNode->m_ref];
    for(const DualGraphNode& possibleChild: possibleChildren){
      if(!inserted[possibleChild.getRef()]){
        UnfoldTree* childNode = unfoldTreeList.findTreeNodeByIndex(possibleChild.getRef());
        const UnfoldTransformation& inverseTransformation = mesh->getDualGraph().findInverseUnfoldTransformation(possibleChild.getUnfoldTransformation());
        childNode->createNode(possibleChild.getRef(), currentTreeNode, inverseTransformation);
        currentTreeNode->m_children.push_back(childNode);
        nextTreeNodes.push(childNode);
        inserted[possibleChild.getRef()] = true;
      }
    }
  }
  return unfoldTreeList.getRoot();
}
