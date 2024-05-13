
#include "DebugDrawer.hpp"

// C++ Headers
#include <iostream>
#include <vector>

// Project Headers
#include <View/Variables/Colorpalette.hpp>

// QT Headers
#include <QColor>

DebugDrawer::DebugDrawer(const UnfoldMesh* mesh, const PolygonList& polygonList):
  Drawer(),
  m_mesh(mesh),
  m_meshLineIndexBuffer(QOpenGLBuffer(QOpenGLBuffer::IndexBuffer)),
  m_meshPointIndexBuffer(QOpenGLBuffer(QOpenGLBuffer::IndexBuffer)),
  m_meshTriangleIndexBuffer(QOpenGLBuffer(QOpenGLBuffer::IndexBuffer)),
  m_meshVertexBuffer(QOpenGLBuffer(QOpenGLBuffer::VertexBuffer)),
  m_polygonList(polygonList){
}

// Public

void DebugDrawer::buffer(){
  m_meshLineIndexBuffer.destroy();
  m_meshPointIndexBuffer.destroy();
  m_meshTriangleIndexBuffer.destroy();
  m_meshVertexBuffer.destroy();

  m_meshLineIndexBuffer.create();
  m_meshLineIndexBuffer.setUsagePattern(QOpenGLBuffer::StaticDraw);
  m_meshLineIndexBuffer.bind();
  m_meshLineIndexBuffer.allocate(m_mesh->getSumPointsPerFace() * 2 * sizeof(GLuint));
  m_meshLineIndexBuffer.release();

  m_meshPointIndexBuffer.create();
  m_meshPointIndexBuffer.setUsagePattern(QOpenGLBuffer::StaticDraw);
  m_meshPointIndexBuffer.bind();
  m_meshPointIndexBuffer.allocate(m_mesh->getSumPointsPerFace() * sizeof(GLuint));
  m_meshPointIndexBuffer.release();

  m_meshTriangleIndexBuffer.create();
  m_meshTriangleIndexBuffer.setUsagePattern(QOpenGLBuffer::StaticDraw);
  m_meshTriangleIndexBuffer.bind();
  m_meshTriangleIndexBuffer.allocate(m_mesh->getSumPointsPerFace() * 3 * sizeof(GLuint));
  m_meshTriangleIndexBuffer.release();

  setupIndices();

  m_meshVertexBuffer.create();
  m_meshVertexBuffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);
  m_meshVertexBuffer.bind();
  m_meshVertexBuffer.allocate((m_mesh->getSumPointsPerFace() + m_mesh->getFaces().size()) * sizeof(MeshVertexColorData));
  m_meshVertexBuffer.release();

  updateAllVertices();
}

void DebugDrawer::draw(QOpenGLShaderProgram* program){
  if(!m_meshVertexBuffer.isCreated() || !m_meshTriangleIndexBuffer.isCreated()){
    return;
  }
  program->bind();
  QColor c;
  float alpha = 1.f;

  glDisable(GL_CULL_FACE);

  glShadeModel(GL_FLAT);
  c = Colorpalette::debugFillColor;

  m_meshVertexBuffer.bind();
  setupShaderProperties(program);
  program->setUniformValue("uniformColor", c.redF(), c.greenF(), c.blueF(), alpha);

  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  m_meshTriangleIndexBuffer.bind();
  glDrawElements(GL_TRIANGLES, m_mesh->getSumPointsPerFace() * 3, GL_UNSIGNED_INT, nullptr);
  m_meshTriangleIndexBuffer.release();

  // Set color to black
  program->setUniformValue("useIndividualColors", false);
  program->setUniformValue("uniformColor", 0.f, 0.f, 0.f, 1.f);
  // Shift lines and points a bit out
  glEnable(GL_POLYGON_OFFSET_LINE);
  glPolygonOffset(-1.f, 1.f);

  // Set mode to lines
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  glLineWidth(1.f);
  m_meshLineIndexBuffer.bind();
  glDrawElements(GL_LINES, m_mesh->getSumPointsPerFace() * 2, GL_UNSIGNED_INT, nullptr);
  m_meshLineIndexBuffer.release();

  // Draw points
  glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
  glEnable(GL_POINT_SMOOTH);
  glPointSize(7.5f);
  m_meshPointIndexBuffer.bind();
  glDrawElements(GL_POINTS, m_mesh->getSumPointsPerFace(), GL_UNSIGNED_INT, nullptr);
  glDisable(GL_POINT_SMOOTH);
  m_meshPointIndexBuffer.release();

  m_meshVertexBuffer.release();

  glDisable(GL_BLEND);
  GLenum glError = glGetError();
  if(glError){
    std::cerr << "DebugDrawer: " << glError << std::endl;
  }
}

void DebugDrawer::updateAllVertices(){
  std::vector<MeshVertexColorData> vertexData(m_mesh->getSumPointsPerFace() + m_mesh->getFaces().size());
  int index = 0;
  for(size_t i = 0; i < m_mesh->getFaces().size(); ++i){
    Eigen::Vector3d midpoint;
    midpoint.setZero();
    const Eigen::Vector3d& n = m_mesh->getFaceNormals().row(i);
    const Eigen::Affine3d& transformation = m_polygonList[i].m_globalTransformation;
    for(size_t j = 0; j < m_mesh->getFaces()[i].size(); ++j){
      const Eigen::Vector3d& p = m_mesh->getVertices().row(m_mesh->getFaces()[i][j]);
      midpoint += p;
      vertexData[index + j].normal = (transformation.rotation() * n).cast<float>();
      vertexData[index + j].position = (transformation * p).cast<float>();
      vertexData[index + j].color = m_mesh->getFaceNormals().row(i).cast<float>().cwiseAbs();
    }
    index += m_mesh->getFaces()[i].size();
    midpoint /= m_mesh->getFaces()[i].size();
    vertexData[m_mesh->getSumPointsPerFace() + i].normal = (transformation.rotation() * n).cast<float>();
    vertexData[m_mesh->getSumPointsPerFace() + i].position = (transformation * midpoint).cast<float>();
    vertexData[m_mesh->getSumPointsPerFace() + i].color = m_mesh->getFaceNormals().row(i).cast<float>().cwiseAbs();
  }
  m_meshVertexBuffer.bind();
  m_meshVertexBuffer.write(0, vertexData.data(), vertexData.size() * sizeof(MeshVertexColorData));
  m_meshVertexBuffer.release();
}

// Private

void DebugDrawer::setupShaderProperties(QOpenGLShaderProgram *program){
  int vertexLocation, normalLocation, colorLocation;
  int offset = 0;

  normalLocation = program->attributeLocation("normal");
  program->enableAttributeArray(normalLocation);
  program->setAttributeBuffer(normalLocation, GL_FLOAT, offset, 3, sizeof(MeshVertexColorData));

  offset += sizeof(Eigen::Vector3f);

  vertexLocation = program->attributeLocation("position");
  program->enableAttributeArray(vertexLocation);
  program->setAttributeBuffer(vertexLocation, GL_FLOAT, offset, 3, sizeof(MeshVertexColorData));

  offset += sizeof(Eigen::Vector3f);

  colorLocation = program->attributeLocation("individualColor");
  program->enableAttributeArray(colorLocation);
  program->setAttributeBuffer(colorLocation, GL_FLOAT, offset, 4, sizeof(MeshVertexColorData));
}

void DebugDrawer::setupIndices(){
  std::vector<GLuint> indices;
  int index = 0;
  int bufferLocation = 0;

  // Line Indices
  indices.resize(m_mesh->getSumPointsPerFace() * 2);
  index = 0;
  bufferLocation = 0;
  for(size_t i = 0; i < m_mesh->getFaces().size(); ++i){
    for(size_t j = 0; j < m_mesh->getFaces()[i].size(); ++j){
      indices[index] = bufferLocation + j;
      indices[index + 1] = bufferLocation + (j + 1) % m_mesh->getFaces()[i].size();
      index += 2;
    }
    bufferLocation += m_mesh->getFaces()[i].size();
  }
  m_meshLineIndexBuffer.bind();
  m_meshLineIndexBuffer.write(0, indices.data(), indices.size() * sizeof(GLuint));
  m_meshLineIndexBuffer.release();

  // Point Indices
  indices.resize(m_mesh->getSumPointsPerFace());
  index = 0;
  for(size_t i = 0; i < m_mesh->getFaces().size(); ++i){
    for(size_t j = 0; j < m_mesh->getFaces()[i].size(); ++j){
      indices[index] = index;
      index++;
    }
  }
  m_meshPointIndexBuffer.bind();
  m_meshPointIndexBuffer.write(0, indices.data(), indices.size() * sizeof(GLuint));
  m_meshPointIndexBuffer.release();

  // Triangle Indices
  indices.resize(m_mesh->getSumPointsPerFace() * 3);
  index = 0;
  bufferLocation = 0;
  for(size_t i = 0; i < m_mesh->getFaces().size(); ++i){
    for(size_t j = 0; j < m_mesh->getFaces()[i].size(); ++j){
      indices[index] = bufferLocation + j;
      indices[index + 1] = bufferLocation + (j + 1) % m_mesh->getFaces()[i].size();
      indices[index + 2] = m_mesh->getSumPointsPerFace() + i;
      index += 3;
    }
    bufferLocation += m_mesh->getFaces()[i].size();
  }

  m_meshTriangleIndexBuffer.bind();
  m_meshTriangleIndexBuffer.write(0, indices.data(), indices.size() * sizeof(GLuint));
  m_meshTriangleIndexBuffer.release();
}
