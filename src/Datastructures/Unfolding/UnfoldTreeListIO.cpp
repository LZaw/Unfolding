
#include "UnfoldTreeList.hpp"

// C++ Headers
#include <fstream>
#include <iomanip>
#include <iostream>

// Project Headers
#include <Datastructures/Helpers/UnfoldTransformation.hpp>
#include <Datastructures/Unfolding/UnfoldTree.hpp>

// Public

bool UnfoldTreeList::readTreeFromFile(const std::string& path){
  // Open file
  std::fstream file(path, std::ios_base::in);
  if(!file){
    std::cerr << "Could not open " << path << "." << std::endl;
    return false;
  }

  // Drop old tree
  drop();

  // Read File
  std::string line;
  while(std::getline(file, line)){
    std::istringstream iss(line);
    int subTreeRootIndex;
    iss >> subTreeRootIndex;
    (*this)[subTreeRootIndex]->m_ref = subTreeRootIndex;
    int currentIndex;
    UnfoldTree* subTreeRootNode = (*this)[subTreeRootIndex].get();
    while(iss >> currentIndex){
      (*this)[subTreeRootIndex]->m_children.push_back((*this)[currentIndex].get());
      (*this)[currentIndex]->m_parent = subTreeRootNode;
      UnfoldTransformation transform = m_mesh->getDualGraph().findUnfoldTransformation(currentIndex,subTreeRootIndex);
      if(!(*this)[currentIndex]->createNode(currentIndex, subTreeRootNode, transform)){
        drop();
        return false;
      }
    }
  }
  for(const_iterator it = cbegin(); it != cend(); ++it){
    if((*it)->m_parent == nullptr && (*it)->m_ref != -1){
      m_rootNodeIndex = (*it)->m_ref;
    }
  }
  return true;
}

void UnfoldTreeList::writeFoldInstructionsToFile(const std::string& path) const{
  std::cout << "Writing Unfold Instructions to File " << path << std::endl;
  std::vector<std::vector<int>> instructions(m_mesh->getFaces().size());

  for(UnfoldTreeList::const_iterator nodeIterator = cbegin(); nodeIterator != cend(); ++nodeIterator){
    size_t index = std::distance(cbegin(), nodeIterator);
    instructions[(*nodeIterator)->m_ref].reserve(m_mesh->getFaces()[index].size());
    for(std::vector<DualGraphNode>::const_iterator possibleNeighborIterator = m_mesh->getDualGraph()[(*nodeIterator)->m_ref].cbegin(); possibleNeighborIterator != m_mesh->getDualGraph()[(*nodeIterator)->m_ref].cend(); ++possibleNeighborIterator){
      if((*nodeIterator)->isNeighborTo(possibleNeighborIterator->getRef())){
        continue;
      }
      instructions[(*nodeIterator)->m_ref].push_back(possibleNeighborIterator->getRef());
    }
  }
  std::fstream file(path, std::ios_base::out);
  file << std::fixed << std::setprecision(3);
  file << "Missing Connections:\n";
  for(unsigned int i = 0; i < instructions.size(); ++i){
    if(!instructions[i].empty()){
      file << i << ": ";
      for(unsigned int j = 0; j < instructions[i].size(); ++j){
        file << instructions[i][j];
        if(j < instructions[i].size() - 1){
          file << ", ";
        }
      }
      file << "\n";
    }
  }
  file << "\nAngles:\n";

  for(UnfoldTreeList::const_iterator nodeIterator = cbegin(); nodeIterator != cend(); ++nodeIterator){
    if((*nodeIterator)->m_parent == nullptr){
      continue;
    }
    const Eigen::Vector3d& currentFaceNormal = m_mesh->getFaceNormals().row((*nodeIterator)->m_ref);
    const Eigen::Vector3d& parentFaceNormal = m_mesh->getFaceNormals().row((*nodeIterator)->m_parent->m_ref);
    double angle = std::acos(currentFaceNormal.dot(parentFaceNormal)) / M_PI * 180.;
    file << (*nodeIterator)->m_parent->m_ref << ", " << (*nodeIterator)->m_ref << ": " << angle << "\n";
  }
  file.flush();
  file.close();
}

void UnfoldTreeList::writeTreeToFile(const std::string& path) const{
  std::cout << "Writing Unfoldtree to File " << path << std::endl;
  std::fstream file(path, std::ios_base::out);
  for(UnfoldTreeList::const_iterator nodeIterator = cbegin(); nodeIterator != cend(); ++nodeIterator){
    file << (*nodeIterator)->m_ref;
    for(std::vector<UnfoldTree*>::const_iterator childIterator = (*nodeIterator)->m_children.cbegin(); childIterator != (*nodeIterator)->m_children.cend(); ++childIterator){
      file << " " << (*childIterator)->m_ref;
    }
    file << "\n";
  }
  file.close();
  std::cout << "Finished." << std::endl;
}
