
#include "UnfoldTreeList.hpp"

// C++ Headers
#include <algorithm>
#include <iterator>

// Project Headers
#include <Datastructures/Polygon/Polygon2D.hpp>
#include <Datastructures/Unfolding/UnfoldTree.hpp>

UnfoldTreeList::UnfoldTreeList(const UnfoldMesh* mesh):
  std::vector<std::unique_ptr<UnfoldTree>>(mesh->getFaces().size()),
  m_mesh(mesh),
  m_rootNodeIndex(-1){
  UnfoldTree::setUnfoldTreeList(this);
  UnfoldTree::setMesh(mesh);
  for(iterator unfoldTreeIterator = begin(); unfoldTreeIterator != end(); ++unfoldTreeIterator){
    unfoldTreeIterator->reset(new UnfoldTree());
  }
  m_unfoldTreeCache.resize(mesh->getFaces().size());
}

// Public

void UnfoldTreeList::cacheCurrentTree(){
  for(const_iterator it = cbegin(); it != cend(); ++it){
    if((*it) == nullptr || (*it)->m_ref == -1){
      // Node does not exist
      m_unfoldTreeCache[std::distance(cbegin(), it)] = -2;
      continue;
    }
    if((*it)->m_parent == nullptr){
      // Root node
      m_unfoldTreeCache[(*it)->m_ref] = -1;
      m_unfoldTreeCache.m_rootNodeIndex = (*it)->m_ref;
    }else{
      // Regular node
      m_unfoldTreeCache[(*it)->m_ref] = (*it)->m_parent->m_ref;
    }
  }
}

std::vector<UnfoldTree*> UnfoldTreeList::calculateMovableNodes(){
  std::vector<UnfoldTree*> ret;
  ret.reserve(m_mesh->getFaces().size());
  for(const_iterator nodeIterator = cbegin(); nodeIterator != cend(); ++nodeIterator){
    if((*nodeIterator)->m_parent == nullptr){
      continue;
    }
    size_t index = std::distance(cbegin(), nodeIterator);
    if((*nodeIterator)->m_children.size() < m_mesh->getDualGraph()[index].size() - 1){
      ret.push_back(nodeIterator->get());
    }
  }
  return ret;
}

void UnfoldTreeList::drop(){
  for(iterator it = begin(); it != end(); ++it){
    (*it)->drop();
  }
}

const UnfoldTree* UnfoldTreeList::findConstTreeNodeByIndex(const size_t index) const{
  if(index >= size()){
    return nullptr;
  }
  return (*this)[index].get();
}

UnfoldTree* UnfoldTreeList::findTreeNodeByIndex(const size_t index){
  return const_cast<UnfoldTree*>(const_cast<const UnfoldTreeList*>(this)->findConstTreeNodeByIndex(index));
}

UnfoldTree* UnfoldTreeList::getRoot(){
  return const_cast<UnfoldTree*>(const_cast<const UnfoldTreeList*>(this)->getRoot());
}

const UnfoldTree* UnfoldTreeList::getRoot() const{
  if(m_rootNodeIndex < 0 || static_cast<size_t>(m_rootNodeIndex) >= size()){
    return nullptr;
  }
  return (*this)[m_rootNodeIndex].get();
}

int UnfoldTreeList::getRootIndex() const{
  return m_rootNodeIndex;
}

void UnfoldTreeList::reroot(const int newRootIndex){
  UnfoldTree* currentTreeNode = findTreeNodeByIndex(newRootIndex);
  UnfoldTree* newParent = nullptr;
  UnfoldTree* oldParent = nullptr;
  while(currentTreeNode != nullptr){
    oldParent = currentTreeNode->m_parent;
    currentTreeNode->m_parent = newParent;
    currentTreeNode->updateNodeTransformation();
    if(oldParent != nullptr){
      oldParent->eraseChild(currentTreeNode->m_ref);
      oldParent->updateNodeTransformation();
      currentTreeNode->m_children.push_back(oldParent);
    }
    newParent = currentTreeNode;
    currentTreeNode = oldParent;
  }
  m_rootNodeIndex = newRootIndex;
}

void UnfoldTreeList::resize(size_t newSize){
  size_t oldSize = size();
  std::vector<std::unique_ptr<UnfoldTree>>::resize(newSize);
  for(unsigned int i = oldSize; i < size(); ++i){
    (*this)[i].reset(new UnfoldTree());
  }
  m_unfoldTreeCache.resize(newSize);
}

void UnfoldTreeList::restoreCachedTree(){
  drop();
  setRootIndex(m_unfoldTreeCache.m_rootNodeIndex);
  for(unsigned int i = 0; i < m_unfoldTreeCache.size(); ++i){
    if(m_unfoldTreeCache[i] != -2){
      (*this)[i]->m_ref = i;
      if(m_unfoldTreeCache[i] != -1){
        (*this)[i]->m_parent = findTreeNodeByIndex(m_unfoldTreeCache[i]);
        (*this)[m_unfoldTreeCache[i]]->m_children.push_back(findTreeNodeByIndex(i));
      }
    }
  }
  getRoot()->updateTreeTransformations();
}

void UnfoldTreeList::setRootIndex(int index){
  m_rootNodeIndex = index;
}

void UnfoldTreeList::traverseTreeToPolygonList(std::vector<Polygon2D>& polygonList, const Eigen::Affine3d& initialTransformation, bool useFaceIndicesAsListIndices) const{
  getRoot()->traverseTreeToPolygonList(polygonList, initialTransformation, useFaceIndicesAsListIndices);
}

void UnfoldTreeList::updateTransformations(){
  for(iterator it = begin(); it != end(); ++it){
    (*it)->updateNodeTransformation();
  }
}
