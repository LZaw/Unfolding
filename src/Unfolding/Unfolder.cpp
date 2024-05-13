
#include "Unfolder.hpp"

// C++ Headers
#include <iostream>

// Project Headers
#include <Unfolding/BFSUnfolder.hpp>
#include <Unfolding/DFSUnfolder.hpp>
#include <Unfolding/MSTDiffUnfolder.hpp>
#include <Unfolding/MSTQuotientUnfolder.hpp>
#include <Unfolding/SteepestEdgeUnfolder.hpp>
#include <Unfolding/WeightedSteepestEdgeUnfolder.hpp>

Unfolder::Unfolder(const UnfoldMesh* mesh):
  m_mesh(mesh),
  m_unfoldTreeList(m_mesh){
}

// Public

UnfoldTree* Unfolder::createNewUnfoldTree(const Eigen::VectorXd& weights){
  // Resize Map if needed
  resizeUnfoldTreeList();

  // Reset Tree
  resetTree();

  switch(m_method){
  case UnfoldMethod::BFS:{
    BFSUnfolder unfolder;
    return unfolder.createNewUnfoldTree(m_mesh, m_unfoldTreeList);
  }
  case UnfoldMethod::DFS:{
    DFSUnfolder unfolder;
    return unfolder.createNewUnfoldTree(m_mesh, m_unfoldTreeList);
  }
  case UnfoldMethod::MSTDiff:{
    MSTDiffUnfolder unfolder;
    return unfolder.createNewUnfoldTree(m_mesh, m_unfoldTreeList, weights);
  }
  case UnfoldMethod::MSTQuotient:{
    MSTQuotientUnfolder unfolder;
    return unfolder.createNewUnfoldTree(m_mesh, m_unfoldTreeList, weights);
  }
  case UnfoldMethod::SteepestEdge:{
    SteepestEdgeUnfolder unfolder;
    return unfolder.createNewUnfoldTree(m_mesh, m_unfoldTreeList, weights);
  }
  case UnfoldMethod::WeightedSteepestEdge:{
    WeightedSteepestEdgeUnfolder unfolder;
    return unfolder.createNewUnfoldTree(m_mesh, m_unfoldTreeList, weights);
  }
  default:{
    return nullptr;
  }
  }
}

void Unfolder::drop(){
  m_unfoldTreeList.clear();
}

UnfoldTree* Unfolder::getUnfoldTree(){
  return m_unfoldTreeList.getRoot();
}

const UnfoldTree* Unfolder::getUnfoldTree() const{
  return m_unfoldTreeList.getRoot();
}

UnfoldTreeList& Unfolder::getUnfoldTreeList(){
  return m_unfoldTreeList;
}

const UnfoldTreeList& Unfolder::getUnfoldTreeList() const{
  return m_unfoldTreeList;
}

void Unfolder::loadUnfoldTree(const std::string& path){
  resizeUnfoldTreeList();
  if(!m_unfoldTreeList.readTreeFromFile(path)){
    std::cerr << "Could not load Unfoldtree. Creating new one." << std::endl;
    createNewUnfoldTree();
  }
}

void Unfolder::setUnfoldMethod(UnfoldMethod method){
  m_method = method;
}

void Unfolder::rerootTree(const int newRootIndex){
  m_unfoldTreeList.reroot(newRootIndex);
}

void Unfolder::resetTree(){
  for(UnfoldTreeList::iterator it = m_unfoldTreeList.begin(); it != m_unfoldTreeList.end(); ++it){
    (*it)->drop();
  }
}

void Unfolder::saveFoldingInstructions(const std::string& path) const{
  m_unfoldTreeList.writeFoldInstructionsToFile(path);
}

void Unfolder::saveUnfoldTree(const std::string& path) const{
  m_unfoldTreeList.writeTreeToFile(path);
}

// Protected

void Unfolder::resizeUnfoldTreeList(){
  m_unfoldTreeList.resize(m_mesh->getFaces().size());
}
