
#include "UnfoldTreeDrawer.hpp"

// C++ Headers
#include <array>
#include <iostream>

// Project Headers
#include <View/Variables/Colorpalette.hpp>

// QT Headers
#include <QColor>

UnfoldTreeDrawer::UnfoldTreeDrawer(const UnfoldMesh* mesh, const UnfoldTreeList* unfoldTreeList):
  Drawer(),
  m_highlightRoot(false),
  m_mesh(mesh),
  m_unfoldTreeVertexBuffer(QOpenGLBuffer(QOpenGLBuffer::VertexBuffer)),
  m_unfoldTreeLineIndexBuffer(QOpenGLBuffer(QOpenGLBuffer::IndexBuffer)),
  m_unfoldTreeList(unfoldTreeList),
  m_unfoldTreePointIndexBuffer(QOpenGLBuffer(QOpenGLBuffer::IndexBuffer)){
}

// Public

void UnfoldTreeDrawer::buffer(){
  m_unfoldTreeVertexBuffer.destroy();
  m_unfoldTreeLineIndexBuffer.destroy();
  m_unfoldTreePointIndexBuffer.destroy();

  std::vector<GLuint> pointIndices(m_mesh->getFaces().size());

  for(size_t i = 0; i < m_mesh->getFaces().size(); ++i){
    // Point Indices
    pointIndices[i] = i;
  }

  m_unfoldTreePointIndexBuffer.create();
  m_unfoldTreePointIndexBuffer.setUsagePattern(QOpenGLBuffer::StaticDraw);
  m_unfoldTreePointIndexBuffer.bind();
  m_unfoldTreePointIndexBuffer.allocate(pointIndices.data(), pointIndices.size() * sizeof(GLuint));
  m_unfoldTreePointIndexBuffer.release();

  m_unfoldTreeVertexBuffer.create();
  m_unfoldTreeVertexBuffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);
  m_unfoldTreeVertexBuffer.bind();
  // V - E + F = 2 - 2g
  // Ein Punkt pro Face, ein Punkt pro Kante -> E + F = V + F - 2 + 2g + F
  m_unfoldTreeVertexBuffer.allocate((m_mesh->getFaces().size() * 2 + m_mesh->getVertices().rows() - 2 + 2 * m_mesh->getGenus()) * sizeof(Eigen::Vector3f));
  m_unfoldTreeVertexBuffer.release();

  m_unfoldTreeLineIndexBuffer.create();
  m_unfoldTreeLineIndexBuffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);
  m_unfoldTreeLineIndexBuffer.bind();
  m_unfoldTreeLineIndexBuffer.allocate(((m_mesh->getFaces().size() - 1) * 4) * sizeof(GLuint));
  m_unfoldTreeLineIndexBuffer.release();
  updateAllNodes();
}

void UnfoldTreeDrawer::draw(QOpenGLShaderProgram* program){
  if(!m_unfoldTreeVertexBuffer.isCreated() ||
     !m_unfoldTreeLineIndexBuffer.isCreated() ||
     !m_unfoldTreePointIndexBuffer.isCreated()){
    return;
  }
  program->bind();

  m_unfoldTreeVertexBuffer.bind();
  m_unfoldTreePointIndexBuffer.bind();

  if(m_screenshotLines){
    glLineWidth(6.f);
  }else{
    glLineWidth(3.f);
  }
  glPointSize(7.5f);
  
  QColor c;
  setupShaderProperties(program);
  if(m_highlightRoot){
    c = Colorpalette::rootPolygonColor;
    program->setUniformValue("rootColor", c.redF(), c.greenF(), c.blueF(), 1.f);
  }
  c = Colorpalette::unfoldTreeColor;
  program->setUniformValue("color", c.redF(), c.greenF(), c.blueF(), 1.f);

  // Draw TreeNodes
  glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
  glEnable(GL_POINT_SMOOTH);
  glDrawElements(GL_POINTS, m_mesh->getFaces().size(), GL_UNSIGNED_INT, nullptr);
  glDisable(GL_POINT_SMOOTH);

  m_unfoldTreeLineIndexBuffer.bind();

  // Set line color back for root
  program->setUniformValue("rootColor", c.redF(), c.greenF(), c.blueF(), 1.f);
  // Draw TreeLines
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  glEnable(GL_LINE_SMOOTH);
  glDrawElements(GL_LINES, (m_mesh->getFaces().size() - 1) * 4, GL_UNSIGNED_INT, nullptr);
  m_unfoldTreeLineIndexBuffer.release();

  m_unfoldTreeVertexBuffer.release();

  GLenum glError = glGetError();
  if(glError){
    std::cerr << "UnfoldTreeDrawer: " << glError << std::endl;
  }
}

void UnfoldTreeDrawer::toggleHighlightRoot(bool toggle){
  m_highlightRoot = toggle;
}

void UnfoldTreeDrawer::updateAllNodes(){
  m_midpointIndexMap.clear();
  m_midpointIndexMap.resize(m_mesh->getFaces().size());

  // Calculate Midpoints
  std::vector<Eigen::Vector3f> unfoldTreeMidPointData(m_mesh->getFaces().size());

  for(size_t i = 0; i < m_mesh->getFaces().size(); ++i){
    // Face-Centers
    Eigen::Vector3d p;
    p.setZero();
    for(size_t j = 0; j < m_mesh->getFaces()[i].size(); ++j){
      p += m_mesh->getVertices().row(m_mesh->getFaces()[i][j]);
    }
    p /= static_cast<double>(m_mesh->getFaces()[i].size());
    // Ein bisschen rausversetzen, um z-Fighting zu vermeiden
    p += m_mesh->getFaceNormals().row(i) / 1'000.;
    unfoldTreeMidPointData[i] = p.cast<float>();
  }

  m_unfoldTreeVertexBuffer.bind();
  m_unfoldTreeVertexBuffer.write(0, unfoldTreeMidPointData.data(), unfoldTreeMidPointData.size() * sizeof(Eigen::Vector3f));
  m_unfoldTreeVertexBuffer.release();


  // Calculate Edge-points
  // V - E + F = 2 - 2g
  // Ein Punkt pro Face, ein Punkt pro Kante -> E + F = V + F - 2 + 2g + F
  // Die Mittelpunkte wurden bereits oben berechnet -> -1F -> V + F - 2 + 2g
  std::vector<Eigen::Vector3f> unfoldTreePointData(m_mesh->getFaces().size() + m_mesh->getVertices().rows() - 2 + 2 * m_mesh->getGenus());
  int currentIndex = 0;
  for(size_t i = 0; i < m_mesh->getFaces().size(); ++i){
    const std::vector<unsigned int>& currentFace = m_mesh->getFaces()[i];
    Eigen::Vector3d midPointFace;
    midPointFace.setZero();
    for(size_t j = 0; j < currentFace.size(); ++j){
      midPointFace += m_mesh->getVertices().row(currentFace[j]);
    }
    midPointFace /= static_cast<double>(currentFace.size());
    for(const DualGraphNode& neighbor: m_mesh->getDualGraph()[i]){
      if(m_midpointIndexMap[neighbor.getRef()].find(i) == m_midpointIndexMap[neighbor.getRef()].end()){
        const std::vector<unsigned int>& neighborFace = m_mesh->getFaces()[neighbor.getRef()];
        Eigen::Vector3d midPointNeighbor;
        midPointNeighbor.setZero();
        for(size_t j = 0; j < neighborFace.size(); ++j){
          midPointNeighbor += m_mesh->getVertices().row(neighborFace[j]);
        }
        midPointNeighbor /= static_cast<double>(neighborFace.size());
        const Eigen::Vector3d sidelineV0 = m_mesh->getVertices().row(neighbor.getUnfoldTransformation().m_globalV0);
        const Eigen::Vector3d sidelineV1 = m_mesh->getVertices().row(neighbor.getUnfoldTransformation().m_globalV1);
        const Eigen::Vector3d e0 = midPointNeighbor - midPointFace;
        const Eigen::Vector3d e1 = sidelineV1 - sidelineV0;
        const Eigen::Vector3d cross = e0.cross(e1);
        Eigen::Matrix3d M;
        M.col(0) = e0;
        M.col(1) = cross;
        M.col(2) = e1;
        Eigen::PartialPivLU<Eigen::Ref<Eigen::Matrix3d>> solver(M);
        const Eigen::Vector3d result = solver.solve(sidelineV0 - midPointFace);
        unfoldTreePointData[currentIndex] = (sidelineV0 - e1 * result[2] + cross.normalized() / 1'000.).cast<float>();
        m_midpointIndexMap[i][neighbor.getRef()] = currentIndex + m_mesh->getFaces().size();
        m_midpointIndexMap[neighbor.getRef()][i] = currentIndex + m_mesh->getFaces().size();
        currentIndex++;
      }
    }
  }

  m_unfoldTreeVertexBuffer.bind();
  m_unfoldTreeVertexBuffer.write(m_mesh->getFaces().size() * sizeof(Eigen::Vector3f), unfoldTreePointData.data(), unfoldTreePointData.size() * sizeof(Eigen::Vector3f));
  m_unfoldTreeVertexBuffer.release();

  // Traverse Tree:
  // -1 weil jeder Baum V - 1 Kanten hat, * 4 weil jede Kante drei Punkte hat (= 2 Linien)
  std::vector<GLuint> lineIndices((m_mesh->getFaces().size() - 1) * 4);

  int rootOffset = 0;
  for(UnfoldTreeList::const_iterator unfoldTreeIterator = m_unfoldTreeList->cbegin(); unfoldTreeIterator != m_unfoldTreeList->cend(); ++unfoldTreeIterator){
    if((*unfoldTreeIterator)->m_parent == nullptr){
      continue;
    }
    int nodeIndex = (*unfoldTreeIterator)->m_ref;
    if(nodeIndex > m_unfoldTreeList->getRootIndex()){
      rootOffset = 1;
    }else{
      rootOffset = 0;
    }
    int parentIndex = (*unfoldTreeIterator)->m_parent->m_ref;
    lineIndices[4 * (nodeIndex - rootOffset) + 0] = nodeIndex;
    lineIndices[4 * (nodeIndex - rootOffset) + 1] = m_midpointIndexMap[nodeIndex][parentIndex];
    lineIndices[4 * (nodeIndex - rootOffset) + 2] = m_midpointIndexMap[nodeIndex][parentIndex];
    lineIndices[4 * (nodeIndex - rootOffset) + 3] = parentIndex;
  }

  m_unfoldTreeLineIndexBuffer.bind();
  m_unfoldTreeLineIndexBuffer.write(0, lineIndices.data(), lineIndices.size() * sizeof(GLuint));
  m_unfoldTreeLineIndexBuffer.release();
}

void UnfoldTreeDrawer::updateNode(const std::pair<int, int>& oldConnection, const std::pair<int, int>& newConnection){
  int index = oldConnection.first;
  if(index > m_unfoldTreeList->getRootIndex()){
    --index;
  }
  std::array<GLuint, 4> lineIndices{static_cast<GLuint>(newConnection.first),
                                    static_cast<GLuint>(m_midpointIndexMap[newConnection.first][newConnection.second]),
                                    static_cast<GLuint>(m_midpointIndexMap[newConnection.first][newConnection.second]),
                                    static_cast<GLuint>(newConnection.second)};
  m_unfoldTreeLineIndexBuffer.bind();
  m_unfoldTreeLineIndexBuffer.write(4 * index * sizeof(GLuint), lineIndices.data(), lineIndices.size() * sizeof(GLuint));
  m_unfoldTreeLineIndexBuffer.release();
}

// Private

void UnfoldTreeDrawer::setupShaderProperties(QOpenGLShaderProgram *program){
  int vertexLocation;
  int offset = 0;

  vertexLocation = program->attributeLocation("position");
  program->enableAttributeArray(vertexLocation);
  program->setAttributeBuffer(vertexLocation, GL_FLOAT, offset, 3, sizeof(Eigen::Vector3f));
}
