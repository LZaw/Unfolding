
#pragma once

// C++ Headers
#include <memory>
#include <string>

// Project Headers
#include <Datastructures/Mesh/FaceList.hpp>

// Eigen Headers
#include <Eigen/Dense>
#include <Eigen/Geometry>

// TODO: Klasse ausdünnen und Dinge auslagern
class Mesh{
public:
  Mesh();

  bool calculateIsTriangular();
  const Eigen::MatrixXd& calculateFaceNormals();
  void calculateNormals();
  const Eigen::MatrixXd& calculateVertexNormals();
  double calculateVolume() const;

  void drop();

  const Eigen::AlignedBox3d& getBoundingBox() const;
  const Eigen::MatrixXd& getFaceNormals() const;
  const FaceList& getFaces() const;
  FaceList& getFaces();
  const Eigen::MatrixXd& getVertexNormals() const;
  const Eigen::MatrixXd& getVertices() const;
  Eigen::MatrixXd& getVertices();

  bool isTriangular() const;

  void normalizeBoundingBox();
  void normalizeVolume();

  bool readFromFile(const std::string& path);

  void setFaces(const FaceList& newFaces);
  void setVertices(const Eigen::MatrixXd& vertices);
  void shiftAndNormalize();
  void shiftBoundingBoxCenterToOrigin();
  void shiftToOrigin();

  void updateBoundingBox();

  bool write(const std::string& path) const;

protected:
  class IO{
  public:
    IO(Mesh* mesh);

    bool readFromFile(const std::string& path) const;

    bool writeToFile(const std::string& path) const;

  private:
    Mesh* m_mesh;

    void postProcessRead() const;

    bool readOff(const std::string& path) const;

    bool writeOff(const std::string& path) const;
  };

  Eigen::AlignedBox3d             m_bBox;
  FaceList                        m_faces;
  Eigen::MatrixXd                 m_fNormals;
  std::unique_ptr<Mesh::IO>       m_io;
  bool                            m_isTriangular;
  Eigen::MatrixXd                 m_unnormalizedFaceNormals;
  Eigen::MatrixXd                 m_vertices;
  Eigen::MatrixXd                 m_vNormals;

  void calculateFaceAreaNormal(const int faceID);
  void calculateFaceAreaNormals();

  virtual void postReadInitialization() = 0;
};
