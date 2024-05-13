
#pragma once

// C++ Headers
#include <memory>
#include <string>

// Project Headers
#include <Datastructures/Mesh/OriginalMesh.hpp>
#include <Datastructures/Mesh/UnfoldMesh.hpp>

// Eigen Headers
#include <Eigen/Dense>

class Model3D{
public:
  explicit Model3D();

  void dropFoldedMesh();
  void dropOriginalMesh();

  const OriginalMesh* getOriginalMesh() const;
  const UnfoldMesh* getUnfoldMesh() const;
  UnfoldMesh* getUnfoldMesh();

  void loadFoldedMesh(const std::string& path);
  void loadOriginalMesh(const std::string& path);

  void saveFoldedMesh(const std::string& path);
  void saveOriginalMesh(const std::string& path);
  void simplifyOriginalMesh(const int numFaces);

private:
  std::unique_ptr<OriginalMesh> m_originalMesh;
  std::unique_ptr<UnfoldMesh>   m_unfoldMesh;
};
