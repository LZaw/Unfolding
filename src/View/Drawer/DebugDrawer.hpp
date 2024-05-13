
#pragma once

// Project Headers
#include <Datastructures/Mesh/UnfoldMesh.hpp>
#include <Datastructures/Polygon/PolygonList.hpp>
#include <View/Drawer/Drawer.hpp>

// QT Headers
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>

class DebugDrawer : public Drawer{
public:
  DebugDrawer(const UnfoldMesh* mesh, const PolygonList& polygonList);

  void buffer();

  void draw(QOpenGLShaderProgram* program);

  void updateAllVertices();

private:
  const UnfoldMesh* const m_mesh;
  QOpenGLBuffer           m_meshLineIndexBuffer;
  QOpenGLBuffer           m_meshPointIndexBuffer;
  QOpenGLBuffer           m_meshTriangleIndexBuffer;
  QOpenGLBuffer           m_meshVertexBuffer;
  const PolygonList&      m_polygonList;

  void setupShaderProperties(QOpenGLShaderProgram* program);
  void setupIndices();
};
