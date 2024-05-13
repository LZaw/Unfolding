
#include "Mesh.hpp"

// C++ Headers
#include <fstream>

// Project Headers
#include <Util/FileUtil.hpp>

// libIGL Headers
#include <igl/read_triangle_mesh.h>
#include <igl/write_triangle_mesh.h>

Mesh::IO::IO(Mesh* mesh):
  m_mesh(mesh){
}

// Public

bool Mesh::IO::readFromFile(const std::string& path) const{
  m_mesh->drop();
  if(FileUtil::stringEndsWith(path, ".off")){
    if(!readOff(path)){
      return false;
    }
    postProcessRead();
    return true;
  }
  Eigen::MatrixXi F;
  if(igl::read_triangle_mesh(path, m_mesh->m_vertices, F)){
    m_mesh->m_faces.resize(F.rows());
    for(Eigen::Index i = 0; i < F.rows(); ++i){
      m_mesh->m_faces[i].resize(F.cols());
      for(Eigen::Index j = 0; j < F.cols(); ++j){
        m_mesh->m_faces[i][j] = F(i, j);
      }
    }
    postProcessRead();
    return true;
  }
  return false;
}

bool Mesh::IO::writeToFile(const std::string& path) const{
  if(!m_mesh->isTriangular() || FileUtil::stringEndsWith(path, "off")){
    return writeOff(FileUtil::removeFileEnding(path) + ".off");
  }else{
    Eigen::MatrixXi F(m_mesh->m_faces.size(), 3);
    F.setZero();
    for(size_t i = 0; i < m_mesh->m_faces.size(); ++i){
      for(size_t j = 0; j < m_mesh->m_faces[i].size(); ++j){
        F(i, j) = m_mesh->m_faces[i][j];
      }
    }
    return igl::write_triangle_mesh(path, m_mesh->m_vertices, F);
  }
}

// Private

void Mesh::IO::postProcessRead() const{
  m_mesh->calculateIsTriangular();
  m_mesh->calculateNormals();
  m_mesh->updateBoundingBox();
  m_mesh->postReadInitialization();
}

bool Mesh::IO::readOff(const std::string& path) const{
  std::fstream file(path, std::ios_base::in);
  if(!file){
    std::cerr << "Could not open " << path << "." << std::endl;
    return false;
  }
  std::string s;
  file >> s;
  if(s.compare(0, 3, "OFF") != 0){
    return false;
  }
  int vertexCount, faceCount, edgeCount, pointsPerFace, vertexRef;
  double val;
  file >> vertexCount;
  file >> faceCount;
  file >> edgeCount;
  m_mesh->m_vertices.resize(vertexCount, 3);
  m_mesh->m_faces.resize(faceCount);
  for(int i = 0; i < vertexCount; ++i){
    for(int j = 0; j < 3; ++j){
      file >> val;
      m_mesh->m_vertices.coeffRef(i, j) = val;
    }
  }
  for(int i = 0; i < faceCount; ++i){
    file >> pointsPerFace;
    m_mesh->m_faces[i].resize(pointsPerFace);
    for(int j = 0; j < pointsPerFace; ++j){
      file >> vertexRef;
      m_mesh->m_faces[i][j] = vertexRef;
    }
  }
  file.close();
  return true;
}

bool Mesh::IO::writeOff(const std::string& path) const{
  std::fstream file(path, std::ios_base::out);
  if(!file){
    std::cerr << "Could not open " << path << "." << std::endl;
    return false;
  }
  file << "OFF" << std::endl << m_mesh->m_vertices.rows() << " " << m_mesh->m_faces.size() << " " << 0 << std::endl;
  Eigen::Vector3d cVertex;
  for(unsigned int i = 0; i < m_mesh->m_vertices.rows(); ++i){
    cVertex = m_mesh->m_vertices.row(i);
    file << cVertex[0] << " " << cVertex[1] << " " << cVertex[2] << std::endl;
  }
  for(FaceList::const_iterator faceIterator = m_mesh->m_faces.cbegin(); faceIterator != m_mesh->m_faces.cend(); ++faceIterator){
    file << faceIterator->size() << " ";
    for(size_t j = 0; j < faceIterator->size(); ++j){
      file << (*faceIterator)[j];
      if(j < faceIterator->size() - 1){
        file << " ";
      }
    }
    file << std::endl;
  }
  file.close();
  return true;
}
