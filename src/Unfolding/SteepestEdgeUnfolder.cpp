
#include "SteepestEdgeUnfolder.hpp"

// C++ Headers
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <queue>

// Project Headers
#include <Datastructures/Mesh/DualGraph.hpp>
#include <Datastructures/Mesh/DualGraphNode.hpp>
#include <Datastructures/Mesh/FaceList.hpp>

// Public

UnfoldTree* SteepestEdgeUnfolder::createNewUnfoldTree(const UnfoldMesh* mesh, UnfoldTreeList& unfoldTreeList, const Eigen::VectorXd& weights){
  // Cacluate Cut Direction
  const Eigen::Vector3d steepestDirection = generateSteepestDirection(mesh, weights);

  // Calculate Cut Edges
  std::vector<std::vector<unsigned int>> cutEdges = calculateCutEdges(mesh, steepestDirection);

  // Set Up Root Node
  int rootIndex = calculateRootIndex(mesh, weights);
  unfoldTreeList.setRootIndex(rootIndex);
  UnfoldTree* rootNode = unfoldTreeList[rootIndex].get();
  rootNode->createNode(rootIndex, nullptr, UnfoldTransformation(rootIndex, -1));

  // Create Tree By CutStructure
  setUpTree(mesh, unfoldTreeList, cutEdges);

  return rootNode;
}

// Protected

std::vector<std::vector<unsigned int>> SteepestEdgeUnfolder::calculateCutEdges(const UnfoldMesh* mesh, const Eigen::Vector3d& steepestDirection){
  const Eigen::MatrixXd& vertices = mesh->getVertices();
  const std::vector<std::vector<unsigned int>>& faces = mesh->getFaces();
  std::vector<std::vector<unsigned int>> cutEdges(vertices.rows());
  for(Eigen::Index i = 0; i < vertices.rows(); ++i){
    int bestNeighborVertexIndex = -1;
    double maxDotProduct = -1;
    const Eigen::Vector3d& currentVertex = vertices.row(i);
    for(unsigned int j : mesh->getFacesPerVertex()[i]){
      int currentVertexIndex = -1;
      const std::vector<unsigned int>& currentFace = faces[j];
      for(size_t k = 0; k < currentFace.size(); ++k){
        if(currentFace[k] == i){
          currentVertexIndex = k;
          break;
        }
      }
      for(int indexOffset: {-1, 1}){
        int currentNeighborIndex = currentFace[(currentVertexIndex + indexOffset + currentFace.size()) % currentFace.size()];
        const Eigen::Vector3d& precedingVertex = vertices.row(currentNeighborIndex);
        double dotProduct = steepestDirection.dot((currentVertex - precedingVertex).normalized());
        if(dotProduct > maxDotProduct){
          maxDotProduct = dotProduct;
          bestNeighborVertexIndex = currentNeighborIndex;
        }
      }
    }
    if(maxDotProduct > 0 && bestNeighborVertexIndex != -1){
      cutEdges[i].push_back(bestNeighborVertexIndex);
      cutEdges[bestNeighborVertexIndex].push_back(i);
    }
  }
  return cutEdges;
}

int SteepestEdgeUnfolder::calculateRootIndex(const UnfoldMesh* mesh, const Eigen::VectorXd& weights){
  // Unused
  (void)weights;

  return std::rand() % mesh->getFaces().size();
}

Eigen::Vector3d SteepestEdgeUnfolder::generateSteepestDirection(const UnfoldMesh* mesh, const Eigen::VectorXd& weights){
  // Unused
  (void)weights;
  (void)mesh;

  return Eigen::Vector3d::Random().normalized();
}

void SteepestEdgeUnfolder::setUpTree(const UnfoldMesh* mesh, UnfoldTreeList& unfoldTreeList, const std::vector<std::vector<unsigned int>>& cutEdges){
  std::queue<UnfoldTree*> nextTreeNodes;
  std::vector<bool> inserted(mesh->getFaces().size(), false);
  inserted[unfoldTreeList.getRootIndex()] = true;
  nextTreeNodes.push(unfoldTreeList.getRoot());
  while(!nextTreeNodes.empty()){
    UnfoldTree* currenTreeNode = nextTreeNodes.front();
    nextTreeNodes.pop();
    for(const DualGraphNode& possibleChildNode: mesh->getDualGraph()[currenTreeNode->m_ref]){
      const UnfoldTransformation& transformation = possibleChildNode.getUnfoldTransformation();
      if(inserted[possibleChildNode.getRef()] ||
         std::find(cutEdges[transformation.m_globalV0].begin(), cutEdges[transformation.m_globalV0].end(), transformation.m_globalV1) != cutEdges[transformation.m_globalV0].end() ||
         std::find(cutEdges[transformation.m_globalV1].begin(), cutEdges[transformation.m_globalV1].end(), transformation.m_globalV0) != cutEdges[transformation.m_globalV1].end()){
        continue;
      }
      UnfoldTree* newChild = unfoldTreeList.findTreeNodeByIndex(possibleChildNode.getRef());
      const UnfoldTransformation& inverseTransformation = mesh->getDualGraph().findInverseUnfoldTransformation(transformation);
      newChild->createNode(possibleChildNode.getRef(), currenTreeNode, inverseTransformation);
      currenTreeNode->m_children.push_back(newChild);
      inserted[currenTreeNode->m_children.back()->m_ref] = true;
      nextTreeNodes.push(currenTreeNode->m_children.back());
    }
  }
  // Insert Lost nodes
  std::queue<size_t> lostChildren;
  for(size_t i = 0; i < inserted.size(); ++i){
    if(!inserted[i]){
      std::cout << "LostChild found" << std::endl;
      lostChildren.push(i);
    }
  }
  while(!lostChildren.empty()){
    size_t lostChildIndex = lostChildren.front();
    lostChildren.pop();
    UnfoldTree* lostChild = unfoldTreeList.findTreeNodeByIndex(lostChildIndex);
    const std::vector<DualGraphNode>& parentList = mesh->getDualGraph()[lostChildIndex];
    std::vector<int> filteredParentList;
    filteredParentList.reserve(parentList.size());
    for(size_t j = 0; j < parentList.size(); ++j){
      if(parentList[j].getRef() != -1){
        filteredParentList.push_back(j);
      }
    }
    if(filteredParentList.empty()){
      lostChildren.push(lostChildIndex);
      continue;
    }
    const DualGraphNode& newParent = parentList[std::rand() % parentList.size()];
    UnfoldTree* newParentNode = unfoldTreeList.findTreeNodeByIndex(newParent.getRef());
    const UnfoldTransformation& inverseTransformation = mesh->getDualGraph().findInverseUnfoldTransformation(newParent.getUnfoldTransformation());
    lostChild->createNode(lostChildIndex, newParentNode, inverseTransformation);
    newParentNode->m_children.push_back(lostChild);
    inserted[lostChildIndex] = true;
  }
  for(size_t i = 0; i < inserted.size(); ++i){
    if(!inserted[i]){
      std::cerr << "Face " << i << " not inserted." << std::endl;
    }
  }
}
