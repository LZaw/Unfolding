
#pragma once

// Project Headers
#include <Datastructures/Mesh/UnfoldMesh.hpp>
#include <Datastructures/Unfolding/UnfoldTree.hpp>
#include <Datastructures/Unfolding/UnfoldTreeList.hpp>
#include <View/Drawer/Drawer.hpp>

// QT Headers
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>

class CutTreeDrawer : public Drawer{
public:
  CutTreeDrawer(const UnfoldMesh* mesh, const UnfoldTreeList* unfoldTreeList);

  void draw(QOpenGLShaderProgram* program);

private:
  const UnfoldMesh* const m_mesh;
  const UnfoldTreeList*   m_unfoldTreeList;

  void setupShaderProperties(QOpenGLShaderProgram* program);
};
