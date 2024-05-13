
#pragma once

// Project Headers
#include <Datastructures/Mesh/UnfoldMesh.hpp>
#include <View/Drawer/Drawer.hpp>

// QT Headers
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>

class UnfoldMeshDrawer : public Drawer{
public:
  UnfoldMeshDrawer(const UnfoldMesh* mesh);

  void buffer();

  void draw(QOpenGLShaderProgram* program);

  void setFaceColors(const Eigen::MatrixXf& colors);

  void toggleHightlightRoot(bool toggle);

  void updateAllVertices();

private:
  Eigen::MatrixXf         m_faceColors;
  bool                    m_highlightRoot;
  const UnfoldMesh* const m_mesh;
  QOpenGLBuffer           m_meshLineIndexBuffer;
  QOpenGLBuffer           m_meshPointIndexBuffer;
  QOpenGLBuffer           m_meshTriangleIndexBuffer;
  QOpenGLBuffer           m_meshVertexBuffer;

  void setupShaderProperties(QOpenGLShaderProgram* program);
  void setupIndices();
};
