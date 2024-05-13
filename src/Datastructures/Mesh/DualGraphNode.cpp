
#include "DualGraphNode.hpp"

// C++ Headers
#include <utility>

DualGraphNode DualGraphNode::s_invalidNode = DualGraphNode();

// Public

int DualGraphNode::getRef() const{
  return m_ref;
}

const UnfoldTransformation& DualGraphNode::getUnfoldTransformation() const{
  return m_transformation;
}

UnfoldTransformation& DualGraphNode::getUnfoldTransformation(){
  return m_transformation;
}

const DualGraphNode& DualGraphNode::invalidNode(){
  return s_invalidNode;
}

void DualGraphNode::setRef(int ref){
  m_ref = ref;
}

void DualGraphNode::updateUnfoldTransformation(const UnfoldTransformation& transformation){
  m_transformation = transformation;
}

void DualGraphNode::updateUnfoldTransformation(UnfoldTransformation&& transformation){
  m_transformation = std::move(transformation);
}
