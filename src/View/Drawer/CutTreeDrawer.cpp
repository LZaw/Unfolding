
#include "CutTreeDrawer.hpp"

// C++ Headers
#include <iostream>
#include <unordered_map>
#include <vector>

// Project Headers
#include <View/Variables/Colorpalette.hpp>

// QT Headers
#include <QColor>

CutTreeDrawer::CutTreeDrawer(const UnfoldMesh* mesh, const UnfoldTreeList* unfoldTreeList):
  Drawer(),
  m_mesh(mesh),
  m_unfoldTreeList(unfoldTreeList){
}

// Public

void CutTreeDrawer::draw(QOpenGLShaderProgram* program){
  program->bind();

  QColor c = Colorpalette::cutTreeColor;
  program->setUniformValue("color", c.redF(), c.greenF(), c.blueF(), 1.f);

  if(m_screenshotLines){
    glLineWidth(6.f);
  }else{
    glLineWidth(3.f);
  }
  glBegin(GL_LINES);
  for(size_t i = 0; i < m_mesh->getDualGraph().size(); ++i){
    const std::vector<DualGraphNode>& nodes = m_mesh->getDualGraph()[i];
    std::unordered_map<unsigned int, bool> neighborMap(nodes.size());
    for(size_t j = 0; j < nodes.size(); ++j){
      neighborMap[nodes[j].getRef()] = true;
    }
    const UnfoldTree* treeNode = m_unfoldTreeList->findConstTreeNodeByIndex(i);
    if(treeNode->m_parent){
      neighborMap[treeNode->m_parent->m_ref] = false;
    }
    for(std::vector<UnfoldTree*>::const_iterator it = treeNode->m_children.cbegin(); it != treeNode->m_children.cend(); ++it){
      neighborMap[(*it)->m_ref] = false;
    }
    for(size_t j = 0; j < nodes.size(); ++j){
      if(neighborMap[nodes[j].getRef()]){
        const UnfoldTransformation& t = nodes[j].getUnfoldTransformation();
        const Eigen::Vector3d& p0 = m_mesh->getVertices().row(t.m_globalV0);
        const Eigen::Vector3d& p1 = m_mesh->getVertices().row(t.m_globalV1);
        glVertex3f(p0.x(), p0.y(), p0.z());
        glVertex3f(p1.x(), p1.y(), p1.z());
      }
    }
  }
  glEnd();

  GLenum glError = glGetError();
  if(glError){
    std::cerr << "CutTreeDrawer: " << glError << std::endl;
  }
}

// Private

void CutTreeDrawer::setupShaderProperties(QOpenGLShaderProgram *program){
  int vertexLocation;
  int offset = 0;

  vertexLocation = program->attributeLocation("position");
  program->enableAttributeArray(vertexLocation);
  program->setAttributeBuffer(vertexLocation, GL_FLOAT, offset, 3, sizeof(Eigen::Vector3f));
}
