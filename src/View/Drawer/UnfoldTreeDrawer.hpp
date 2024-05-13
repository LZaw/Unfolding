
#pragma once

// C++ Headers
#include <unordered_map>
#include <utility>
#include <vector>

// Project Headers
#include <Datastructures/Mesh/UnfoldMesh.hpp>
#include <Datastructures/Unfolding/UnfoldTree.hpp>
#include <Datastructures/Unfolding/UnfoldTreeList.hpp>
#include <View/Drawer/Drawer.hpp>

// QT Headers
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>

class UnfoldTreeDrawer : public Drawer{
public:
  UnfoldTreeDrawer(const UnfoldMesh* mesh, const UnfoldTreeList* unfoldTreeList);

  void buffer();

  void draw(QOpenGLShaderProgram* program);

  void toggleHighlightRoot(bool toggle);

  void updateAllNodes();
  void updateNode(const std::pair<int, int>& oldConnection, const std::pair<int, int>& newConnection);

private:
  bool                        m_highlightRoot;
  std::vector<
    std::unordered_map<
      int, int>>              m_midpointIndexMap;
  const UnfoldMesh* const     m_mesh;
  QOpenGLBuffer               m_unfoldTreeVertexBuffer;
  QOpenGLBuffer               m_unfoldTreeLineIndexBuffer;
  const UnfoldTreeList*       m_unfoldTreeList;
  QOpenGLBuffer               m_unfoldTreePointIndexBuffer;

  void setupShaderProperties(QOpenGLShaderProgram* program);
};
