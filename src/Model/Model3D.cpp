
#include "Model3D.hpp"

// C++ Headers
#include <cmath>
#include <iostream>
#include <limits>

Model3D::Model3D():
  m_originalMesh(new OriginalMesh()),
  m_unfoldMesh(new UnfoldMesh()){
}

// Public

void Model3D::dropFoldedMesh(){
  m_unfoldMesh->drop();
}

void Model3D::dropOriginalMesh(){
  m_originalMesh->drop();
  dropFoldedMesh();
}

const OriginalMesh* Model3D::getOriginalMesh() const{
  return m_originalMesh.get();
}

const UnfoldMesh* Model3D::getUnfoldMesh() const{
  return m_unfoldMesh.get();
}

UnfoldMesh* Model3D::getUnfoldMesh(){
  return m_unfoldMesh.get();
}

void Model3D::loadFoldedMesh(const std::string& path){
  m_unfoldMesh->drop();
  std::cout << "Loading folded model " << path << std::endl;
  if(!m_unfoldMesh->readFromFile(path)){
    m_unfoldMesh->drop();
    std::cerr << "Loading folded model failed." << std::endl;
    return;
  }
  std::cout << m_unfoldMesh->getVertices().rows() << " Vertices and " << m_unfoldMesh->getFaces().size() << " Faces read." << std::endl;
}

void Model3D::loadOriginalMesh(const std::string& path){
  dropOriginalMesh();
  std::cout << "Loading model " << path << std::endl;
  if(!m_originalMesh->readFromFile(path)){
    std::cerr << "Loading model failed." << std::endl;
    return;
  }
  if(!m_originalMesh->isTriangular()){
    dropOriginalMesh();
    std::cout << "Please load a triangular mesh." << std::endl;
    return;
  }
  std::cout << m_originalMesh->getVertices().rows() << " Vertices and " << m_originalMesh->getFaces().size() << " Faces read." << std::endl;
  std::cout << "Success." << std::endl;
}

void Model3D::saveFoldedMesh(const std::string& path){
  std::cout << "Saving folded Model" << std::endl;
  if(!m_unfoldMesh->write(path)){
    std::cerr << "Writing failed." << std::endl;
    return;
  }
  std::cout << "Success." << std::endl;
}

void Model3D::saveOriginalMesh(const std::string& path){
  std::cout << "Saving Model" << std::endl;
  if(!m_originalMesh->write(path)){
    std::cerr << "Writing failed." << std::endl;
  }
  std::cout << "Success." << std::endl;
}

void Model3D::simplifyOriginalMesh(const int numFaces){
  if(!m_originalMesh.operator bool()){
    std::cerr << "No Model loaded." << std::endl;
    return;
  }
  m_unfoldMesh->drop();
  m_originalMesh->simplify(*m_unfoldMesh, numFaces);
  m_unfoldMesh->initializeMembers();
}

