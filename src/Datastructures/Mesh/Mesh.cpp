
#include "Mesh.hpp"

Mesh::Mesh():
  m_io(new Mesh::IO(this)),
  m_isTriangular(false){
}

// Public

const Eigen::MatrixXd& Mesh::calculateFaceNormals(){
  m_fNormals.resize(m_faces.size(), 3);
  calculateFaceAreaNormals();
  m_fNormals = m_unnormalizedFaceNormals.rowwise().normalized();
  return m_fNormals;
}

bool Mesh::calculateIsTriangular(){
  m_isTriangular = true;
  for(FaceList::const_iterator faceIterator = m_faces.cbegin(); faceIterator != m_faces.cend(); ++faceIterator){
    if(faceIterator->size() != 3){
      m_isTriangular = false;
      return false;
    }
  }
  return true;
}

void Mesh::calculateNormals(){
  calculateFaceNormals();
  calculateVertexNormals();
}

const Eigen::MatrixXd& Mesh::calculateVertexNormals(){
  m_vNormals.resize(m_vertices.rows(), 3);
  m_vNormals.setZero();
  for(std::size_t i = 0; i < m_faces.size(); ++i){
    for(std::vector<unsigned int>::const_iterator it = m_faces[i].cbegin(); it != m_faces[i].cend(); ++it){
      m_vNormals.row(*it) += m_unnormalizedFaceNormals.row(i);
    }
  }
  m_vNormals.rowwise().normalize();
  return m_vNormals;
}

double Mesh::calculateVolume() const{
  double volume = 0.;
  for(FaceList::const_iterator faceIterator = m_faces.cbegin(); faceIterator != m_faces.cend(); ++faceIterator){
    Eigen::Vector3d midpoint;
    midpoint.setZero();
    for(std::vector<unsigned int>::const_iterator vertexIndexIterator = faceIterator->cbegin(); vertexIndexIterator != faceIterator->cend(); ++vertexIndexIterator){
      midpoint += m_vertices.row(*vertexIndexIterator);
    }
    midpoint /= faceIterator->size();
    for(size_t j = 0; j < faceIterator->size(); ++j){
      const Eigen::Vector3d& v0 = m_vertices.row((*faceIterator)[j]);
      const Eigen::Vector3d& v1 = m_vertices.row((*faceIterator)[(j + 1) % faceIterator->size()]);
      volume += v0.dot(v1.cross(midpoint)) / 6.;
    }
  }
  volume = std::fabs(volume);
  return volume;
}

void Mesh::drop(){
  m_bBox.setEmpty();
  m_faces.clear();
  m_fNormals.resize(0, 3);
  m_unnormalizedFaceNormals.resize(0, 3);
  m_vertices.resize(0, 3);
  m_vNormals.resize(0, 3);
}

const Eigen::AlignedBox3d& Mesh::getBoundingBox() const{
  return m_bBox;
}

const Eigen::MatrixXd& Mesh::getFaceNormals() const{
  return m_fNormals;
}

const FaceList& Mesh::getFaces() const{
  return m_faces;
}

FaceList& Mesh::getFaces(){
  return m_faces;
}

const Eigen::MatrixXd& Mesh::getVertexNormals() const{
  return m_vNormals;
}

const Eigen::MatrixXd& Mesh::getVertices() const{
  return m_vertices;
}

Eigen::MatrixXd& Mesh::getVertices(){
  return m_vertices;
}

bool Mesh::isTriangular() const{
  return m_isTriangular;
}

void Mesh::normalizeBoundingBox(){
  m_vertices /= m_bBox.diagonal().norm();
  updateBoundingBox();
}

void Mesh::normalizeVolume(){
  // Scale to unit volume
  const Eigen::Vector3d mean = m_vertices.colwise().mean();
  m_vertices.rowwise() -= mean.transpose();
  double volume = calculateVolume();
  m_vertices /= std::cbrt(volume);
  m_vertices.rowwise() += mean.transpose();
}

bool Mesh::readFromFile(const std::string& path){
  drop();
  bool success = m_io->readFromFile(path);

  return success;
}

void Mesh::setFaces(const FaceList& newFaces){
  m_faces = newFaces;
}

void Mesh::setVertices(const Eigen::MatrixXd& vertices){
  m_vertices = Eigen::MatrixXd(vertices);
}

void Mesh::shiftAndNormalize(){
  shiftToOrigin();
  normalizeVolume();
  updateBoundingBox();
}

void Mesh::shiftBoundingBoxCenterToOrigin(){
  m_vertices.rowwise() -= Eigen::Vector3d(m_bBox.center()).transpose();
  updateBoundingBox();
}

void Mesh::shiftToOrigin(){
  Eigen::RowVector3d mean = m_vertices.colwise().mean();
  m_vertices.rowwise() -= mean;
}

void Mesh::updateBoundingBox(){
  m_bBox.setEmpty();
  m_bBox.extend(Eigen::Vector3d(m_vertices.colwise().minCoeff()));
  m_bBox.extend(Eigen::Vector3d(m_vertices.colwise().maxCoeff()));
}

bool Mesh::write(const std::string& path) const{
  return m_io->writeToFile(path);
}

// Private

void Mesh::calculateFaceAreaNormal(const int faceID){
  const std::vector<unsigned int>& currentFace = m_faces[faceID];
  Eigen::MatrixXd B(currentFace.size(), 3);
  Eigen::MatrixXd E(3, currentFace.size());
  for(size_t j = 0; j < currentFace.size(); ++j){
    const Eigen::Vector3d ej = m_vertices.row(currentFace[(j + 1) % currentFace.size()]) - m_vertices.row(currentFace[j]);
    B.row(j) = Eigen::Vector3d(m_vertices.row(currentFace[j])) + ej / 2.;
    E.col(j) = ej;
  }
  const Eigen::Matrix3d EB = E * B;
  m_unnormalizedFaceNormals.row(faceID) = Eigen::Vector3d(-EB(1, 2), EB(0, 2), -EB(0, 1));
}

void Mesh::calculateFaceAreaNormals(){
  m_unnormalizedFaceNormals.resize(m_faces.size(), 3);
  for(size_t i = 0; i < m_faces.size(); ++i){
    calculateFaceAreaNormal(i);
  }
}