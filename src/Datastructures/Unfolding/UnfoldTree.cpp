
#include "UnfoldTree.hpp"

// C++ Headers
#include <algorithm>
#include <iostream>
#include <queue>
#include <tuple>
#include <utility>

// Project Headers
#include <Datastructures/Mesh/DualGraph.hpp>
#include <Datastructures/Mesh/DualGraphNode.hpp>

const UnfoldMesh* UnfoldTree::s_mesh = nullptr;
UnfoldTreeList* UnfoldTree::s_unfoldTreeList = nullptr;

UnfoldTree::UnfoldTree():
  m_parent(nullptr),
  m_ref(-1){
}

// Deep copy
UnfoldTree::UnfoldTree(const UnfoldTree* otherTree):
  m_parent(otherTree->m_parent),
  m_ref(otherTree->m_ref),
  m_transformation(otherTree->m_transformation){
  m_children.reserve(otherTree->m_children.size());
  for(std::vector<UnfoldTree*>::const_iterator it = otherTree->m_children.begin(); it != otherTree->m_children.end(); ++it){
    m_children.push_back(*it);
  }
}

// Public

void UnfoldTree::attachToNewParent(UnfoldTree* newParent){
  if(m_parent == nullptr){
    std::cerr << "Can't move root" << std::endl;
    return;
  }
  if(!isConnectableTo(newParent->m_ref)){
    std::cerr << "Can't connect " << m_ref << " to " << newParent->m_ref << std::endl;
    return;
  }
  setLinkToNewParent(newParent);
}

void UnfoldTree::attachToNewParent(UnfoldTree* newParent, UnfoldTree* newSubTreeRootNode){
  if(newSubTreeRootNode->m_ref == m_ref){
    attachToNewParent(newParent);
    return;
  }
  if(m_parent == nullptr){
    std::cerr << "Can't move root" << std::endl;
    return;
  }
  if(!newSubTreeRootNode->isConnectableTo(newParent->m_ref, m_ref)){
    std::cerr << "Can't connect " << newSubTreeRootNode->m_ref << " to " << newParent->m_ref << std::endl;
    return;
  }
  UnfoldTree* currentTreeNode = newSubTreeRootNode;
  while(currentTreeNode->m_parent != nullptr &&
        currentTreeNode != (*s_unfoldTreeList)[m_ref].get()){
    currentTreeNode = currentTreeNode->m_parent;
  }
  if(currentTreeNode != (*s_unfoldTreeList)[m_ref].get()){
    std::cerr << newSubTreeRootNode->m_ref << " not in subTree of " << m_ref << std::endl;
    return;
  }
  currentTreeNode = newSubTreeRootNode;
  UnfoldTree* newIteratorParent = newParent;
  while(currentTreeNode != (*s_unfoldTreeList)[m_ref].get()){
    UnfoldTree* oldParent = currentTreeNode->m_parent;
    currentTreeNode->setLinkToNewParent(newIteratorParent);
    newIteratorParent = currentTreeNode;
    currentTreeNode = oldParent;
  }
  currentTreeNode->setLinkToNewParent(newIteratorParent);
}

int UnfoldTree::calculateDistanceFromRoot() const{
  int ret = 0;
  const UnfoldTree* currentTree = this;
  while(currentTree->m_parent != nullptr){
    currentTree = currentTree->m_parent;
    ++ret;
  }
  return ret;
}

std::vector<unsigned int> UnfoldTree::calculatePossibleNeighbors(const int subTreeRootNodeIndex) const{
  const std::vector<DualGraphNode>& neighbors = s_mesh->getDualGraph()[m_ref];
  std::vector<unsigned int> possibleNeighborList;
  // Root Node
  if(m_parent == nullptr){
    return possibleNeighborList;
  }
  possibleNeighborList.reserve(s_mesh->getFaces()[m_ref].size());
  for(std::vector<DualGraphNode>::const_iterator neighborIterator = neighbors.cbegin(); neighborIterator != neighbors.cend(); ++neighborIterator){
    int neighbor = neighborIterator->getRef();
    if(subTreeRootNodeIndex == -1){
      if(isConnectableTo(neighbor)){
        possibleNeighborList.push_back(neighbor);
      }
    }else{
      if(isConnectableTo(neighbor, subTreeRootNodeIndex)){
        possibleNeighborList.push_back(neighbor);
      }
    }
  }
  return possibleNeighborList;
}

std::vector<unsigned int> UnfoldTree::calculateSubTreeIndices() const{
  std::vector<unsigned int> ret;
  ret.reserve(calculateTreeSize());
  std::queue<const UnfoldTree*> nextTreeNodes;
  nextTreeNodes.push(this);
  while(!nextTreeNodes.empty()){
    const UnfoldTree* currentTreeNode = nextTreeNodes.front();
    nextTreeNodes.pop();
    ret.push_back(currentTreeNode->m_ref);
    for(std::vector<UnfoldTree*>::const_iterator it = currentTreeNode->m_children.cbegin(); it != currentTreeNode->m_children.cend(); ++it){
      nextTreeNodes.push(*it);
    }
  }
  return ret;
}
unsigned int UnfoldTree::calculateTreeHeight() const{
  if(m_children.empty()){
    return 0;
  }
  unsigned int childHeight = 0;
  for(std::vector<UnfoldTree*>::const_iterator it = m_children.cbegin(); it != m_children.cend(); ++it){
    childHeight = std::max(childHeight, (*it)->calculateTreeHeight());
  }
  return childHeight + 1;
}

unsigned int UnfoldTree::calculateTreeSize() const{
  int size = 0;
  std::queue<const UnfoldTree*> nextTreeNodes;
  nextTreeNodes.push(this);
  while(!nextTreeNodes.empty()){
    const UnfoldTree* currentTreeNode = nextTreeNodes.front();
    nextTreeNodes.pop();
    size++;
    for(std::vector<UnfoldTree*>::const_iterator it = currentTreeNode->m_children.cbegin(); it != currentTreeNode->m_children.cend(); ++it){
      nextTreeNodes.push(*it);
    }
  }
  return size;
}

bool UnfoldTree::createNode(const int faceIndex, UnfoldTree*  const parent, const UnfoldTransformation& t){
  if(s_mesh == nullptr || s_unfoldTreeList == nullptr){
    return false;
  }
  m_ref = faceIndex;
  m_parent = parent;
  m_transformation = t;
  m_children.reserve(s_mesh->getFaces()[m_ref].size());
  return true;
}

void UnfoldTree::drop(){
  m_ref = -1;
  m_children.clear();
  m_parent = nullptr;
  m_transformation = UnfoldTransformation();
}

const UnfoldTree* UnfoldTree::findConstTreeNodeByIndex(const int index) const{
  return s_unfoldTreeList->findConstTreeNodeByIndex(index);
}

UnfoldTree* UnfoldTree::findTreeNodeByIndex(const int index){
  return s_unfoldTreeList->findTreeNodeByIndex(index);
}

const UnfoldTreeList* UnfoldTree::getUnfoldTreeList(){
  return s_unfoldTreeList;
}

bool UnfoldTree::hasLoops() const{
  std::vector<bool> alreadyVisited(s_mesh->getFaces().size(), false);
  std::queue<const UnfoldTree*> nextTreeNodes;
  nextTreeNodes.push(this);
  while(!nextTreeNodes.empty()){
    const UnfoldTree* currentTreeNode = nextTreeNodes.front();
    nextTreeNodes.pop();
    if(alreadyVisited[currentTreeNode->m_ref]){
      return true;
    }
    alreadyVisited[currentTreeNode->m_ref] = true;
    for(std::vector<UnfoldTree*>::const_iterator childIterator = currentTreeNode->m_children.cbegin(); childIterator != currentTreeNode->m_children.cend(); ++childIterator){
      nextTreeNodes.push(*childIterator);
    }
  }
  return false;
}

bool UnfoldTree::isChildOf(const UnfoldTree* other) const{
  if(m_parent == nullptr || m_parent->m_ref != other->m_ref){
    return false;
  }
  return true;
}

bool UnfoldTree::isConnectableTo(const int otherTreeNodeIndex) const{
  // Exclude Parent
  if(m_parent->m_ref == otherTreeNodeIndex){
    return false;
  }

  // Check for neighborhood in dualGraph
  if(!s_mesh->getDualGraph().areNeighbors(m_ref, otherTreeNodeIndex)){
    return false;
  }

  // Exclude Descendants of Other TreeNode
  const UnfoldTree* otherTreeNode = s_unfoldTreeList->findTreeNodeByIndex(otherTreeNodeIndex);
  if(otherTreeNode == nullptr){
    std::cerr << otherTreeNodeIndex << " not in this tree." << std::endl;
    return false;
  }
  while(otherTreeNode->m_parent != nullptr){
    if(otherTreeNode->m_ref == m_ref){
      return false;
    }
    otherTreeNode = otherTreeNode->m_parent;
  }
  return true;
}

bool UnfoldTree::isConnectableTo(const int otherTreeNodeIndex, const int subTreeRootNodeIndex) const{
  // Exclude Parent
  if(m_parent->m_ref == otherTreeNodeIndex){
    return false;
  }

  // Check for neighborhood in dualGraph
  if(!s_mesh->getDualGraph().areNeighbors(m_ref, otherTreeNodeIndex)){
    return false;
  }

  // Exclude Descendants of Other TreeNode
  const UnfoldTree* otherTreeNode = s_unfoldTreeList->findTreeNodeByIndex(otherTreeNodeIndex);
  if(otherTreeNode == nullptr){
    std::cerr << otherTreeNodeIndex << " not in this tree." << std::endl;
    return false;
  }
  while(otherTreeNode->m_parent != nullptr){
    if(otherTreeNode->m_ref == m_ref ||
       otherTreeNode->m_ref == subTreeRootNodeIndex){
      return false;
    }
    otherTreeNode = otherTreeNode->m_parent;
  }
  return true;
}

bool UnfoldTree::isNeighborTo(const int otherTreeNodeIndex) const{
  // Check Parent
  if(m_parent != nullptr && m_parent->m_ref == otherTreeNodeIndex){
    return true;
  }
  // Check Children
  if(std::find_if(
       m_children.cbegin(),
       m_children.cend(),
       [otherTreeNodeIndex](const UnfoldTree* childNode){
         return childNode->m_ref == otherTreeNodeIndex;
       }
     ) != m_children.cend()){
    return true;
  }
  return false;
}

void UnfoldTree::setMesh(const UnfoldMesh* mesh){
  s_mesh = mesh;
}

void UnfoldTree::setUnfoldTreeList(UnfoldTreeList* unfoldTreeList){
  s_unfoldTreeList = unfoldTreeList;
}

void UnfoldTree::traverseTreeToPolygonList(std::vector<Polygon2D>& polygonList, const Eigen::Affine3d& initialTransformation, bool useFaceIndicesAsListIndices) const{
  std::queue<std::pair<const UnfoldTree*, Eigen::Affine3d>> nextTreeNodes;
  nextTreeNodes.push({this, initialTransformation});
  int currentListIndex = 0;
  // Could also be done recursively, but only with the current index as a parameter
  while(!nextTreeNodes.empty()){
    const UnfoldTree* currentTreeNode;
    Eigen::Affine3d currentTransformation;
    std::tie(currentTreeNode, currentTransformation) = nextTreeNodes.front();
    nextTreeNodes.pop();
    UnfoldTransformation unfoldTransformation = currentTreeNode->m_transformation;
    currentTransformation = currentTransformation * unfoldTransformation.translation * unfoldTransformation.rotation * unfoldTransformation.translation.inverse();
    if(useFaceIndicesAsListIndices){
      polygonList[currentTreeNode->m_ref].create(currentTreeNode->m_ref, currentTransformation, currentTreeNode->m_transformation);
    }else{
      polygonList[currentListIndex].create(currentTreeNode->m_ref, currentTransformation, currentTreeNode->m_transformation);
      ++currentListIndex;
    }
    for(std::vector<UnfoldTree*>::const_iterator childIterator = currentTreeNode->m_children.cbegin(); childIterator != currentTreeNode->m_children.cend(); ++childIterator){
      nextTreeNodes.push({*childIterator, currentTransformation});
    }
  }
}

void UnfoldTree::updateNodeTransformation(){
  if(m_parent == nullptr){
    m_transformation = UnfoldTransformation();
    return;
  }
  const DualGraphNode& neighborNode = s_mesh->getDualGraph().findDualGraphNode(m_ref, m_parent->m_ref);
  m_transformation = neighborNode.getUnfoldTransformation();
}

void UnfoldTree::updateTreeTransformations(){
  std::queue<UnfoldTree*> nextTreeNodes;
  nextTreeNodes.push(this);
  while(!nextTreeNodes.empty()){
    UnfoldTree* currentTreeNode = nextTreeNodes.front();
    nextTreeNodes.pop();
    currentTreeNode->updateNodeTransformation();
    for(std::vector<UnfoldTree*>::const_iterator childIterator = currentTreeNode->m_children.cbegin(); childIterator != currentTreeNode->m_children.cend(); ++childIterator){
      nextTreeNodes.push(*childIterator);
    }
  }
}

// Private

void UnfoldTree::connectToNewParent(UnfoldTree* newParent){
  m_parent = newParent;
  newParent->m_children.push_back((*s_unfoldTreeList)[m_ref].get());
  m_transformation = s_mesh->getDualGraph().findUnfoldTransformation(m_ref, newParent->m_ref);
}

void UnfoldTree::detachFromParent(){
  m_parent->eraseChild(m_ref);
  m_parent = nullptr;
}

void UnfoldTree::eraseChild(int childIndex){
  std::vector<UnfoldTree*>::iterator childPosition =
      std::find_if(
        m_children.begin(),
        m_children.end(),
        [childIndex](const UnfoldTree* childNode){
          return childNode->m_ref == childIndex;
        }
      );
  if(childPosition == m_children.end()){
    return;
  }
  m_children.erase(childPosition);
}

void UnfoldTree::setLinkToNewParent(UnfoldTree* newParent){
  detachFromParent();
  connectToNewParent(newParent);
}
