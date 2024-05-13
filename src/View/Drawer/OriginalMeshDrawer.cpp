
#include "OriginalMeshDrawer.hpp"

// C++ Headers
#include <iostream>
#include <vector>

// Project Headers
#include <View/Variables/Colorpalette.hpp>

// QT Headers
#include <QColor>

OriginalMeshDrawer::OriginalMeshDrawer(const OriginalMesh* mesh):
  Drawer(),
  m_mesh(mesh),
  m_meshVertexBuffer(QOpenGLBuffer(QOpenGLBuffer::VertexBuffer)),
  m_meshVertexIndexBuffer(QOpenGLBuffer(QOpenGLBuffer::IndexBuffer)){
}

// Public

void OriginalMeshDrawer::buffer(){
  m_meshVertexBuffer.destroy();
  m_meshVertexIndexBuffer.destroy();

  std::vector<MeshVertexData> vertexData(m_mesh->getVertices().rows());
  // Only Triangular Meshes allowed as Original Meshes
  std::vector<GLuint> indices(m_mesh->getFaces().size() * 3);

  for(int i = 0; i < m_mesh->getVertices().rows(); ++i){
    const Eigen::Vector3d& p = m_mesh->getVertices().row(i);
    const Eigen::Vector3d& n = m_mesh->getVertexNormals().row(i);
    vertexData[i].normal = n.cast<float>();
    vertexData[i].position = p.cast<float>();
  }
  for(size_t i = 0; i < m_mesh->getFaces().size(); ++i){
    for(size_t j = 0; j < m_mesh->getFaces()[i].size(); ++j){
      indices[j + i * m_mesh->getFaces()[i].size()] = GLuint(m_mesh->getFaces()[i][j]);
    }
  }

  m_meshVertexBuffer.create();
  m_meshVertexBuffer.setUsagePattern(QOpenGLBuffer::StaticDraw);
  m_meshVertexBuffer.bind();
  m_meshVertexBuffer.allocate(vertexData.data(), vertexData.size() * sizeof(MeshVertexData));
  m_meshVertexBuffer.release();

  m_meshVertexIndexBuffer.create();
  m_meshVertexIndexBuffer.setUsagePattern(QOpenGLBuffer::StaticDraw);
  m_meshVertexIndexBuffer.bind();
  m_meshVertexIndexBuffer.allocate(indices.data(), m_mesh->getFaces().size() * 3 * sizeof(GLuint));
  m_meshVertexIndexBuffer.release();
}

void OriginalMeshDrawer::draw(QOpenGLShaderProgram* program){
  if(!m_meshVertexBuffer.isCreated() || !m_meshVertexIndexBuffer.isCreated()){
    return;
  }
  program->bind();
  QColor c;

  glEnable(GL_CULL_FACE);
  glShadeModel(GL_SMOOTH);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  c = Colorpalette::modelColor;

  m_meshVertexBuffer.bind();
  m_meshVertexIndexBuffer.bind();

  setupShaderProperties(program);
  program->setUniformValue("color", c.redF(), c.greenF(), c.blueF(), 0.5f);

  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glDrawElements(GL_TRIANGLES, m_mesh->getFaces().size() * 3, GL_UNSIGNED_INT, nullptr);

  m_meshVertexIndexBuffer.release();
  m_meshVertexBuffer.release();
  glDisable(GL_BLEND);
  GLenum glError = glGetError();
  if(glError){
    std::cerr << "OriginalMeshDrawer: " << glError << std::endl;
  }
}

// Private

void OriginalMeshDrawer::setupShaderProperties(QOpenGLShaderProgram *program){
  int vertexLocation, normalLocation;
  int offset = 0;

  normalLocation = program->attributeLocation("normal");
  program->enableAttributeArray(normalLocation);
  program->setAttributeBuffer(normalLocation, GL_FLOAT, offset, 3, sizeof(MeshVertexData));

  offset += sizeof(Eigen::Vector3f);

  vertexLocation = program->attributeLocation("position");
  program->enableAttributeArray(vertexLocation);
  program->setAttributeBuffer(vertexLocation, GL_FLOAT, offset, 3, sizeof(MeshVertexData));
}
