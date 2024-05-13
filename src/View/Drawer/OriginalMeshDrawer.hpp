
#pragma once

// Project Headers
#include <Datastructures/Mesh/OriginalMesh.hpp>
#include <View/Drawer/Drawer.hpp>

// QT Headers
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>

class OriginalMeshDrawer : public Drawer{
public:
  OriginalMeshDrawer(const OriginalMesh* mesh);

  void buffer();

  void draw(QOpenGLShaderProgram* program);

private:
  const OriginalMesh* const m_mesh;
  QOpenGLBuffer             m_meshVertexBuffer;
  QOpenGLBuffer             m_meshVertexIndexBuffer;

  void setupShaderProperties(QOpenGLShaderProgram* program);
};
