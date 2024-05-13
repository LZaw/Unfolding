
#include "Widget3D.hpp"

// C++ Headers
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

// Project Headers
#include <View/Variables/Colorpalette.hpp>
#include <View/Variables/Controls.hpp>

// Eigen Headers
#include <Eigen/Geometry>

// QT Headers
#include <QMouseEvent>

Widget3D::Widget3D(UnfoldModel* model, QWidget* parent):
  QOpenGLWidget(parent),
  m_cameraPos(-5),
  m_cutTreeDrawProgram(new QOpenGLShaderProgram),
  m_debugMeshDrawProgram(new QOpenGLShaderProgram),
  m_drawState(0),
  m_model(model),
  m_model3D(m_model->getModel3D()),
  m_originalDrawer(new OriginalMeshDrawer(m_model3D->getOriginalMesh())),
  m_originalMeshDrawProgram(new QOpenGLShaderProgram),
  m_unfoldDrawer(new UnfoldMeshDrawer(m_model3D->getUnfoldMesh())),
  m_unfoldMeshDrawProgram(new QOpenGLShaderProgram),
  m_unfoldTreeDrawer(new UnfoldTreeDrawer(m_model3D->getUnfoldMesh(), &m_model->getUnfolder()->getUnfoldTreeList())),
  m_unfoldTreeDrawProgram(new QOpenGLShaderProgram){
  m_cutTreeDrawer.reset(new CutTreeDrawer(m_model3D->getUnfoldMesh(), &m_model->getUnfolder()->getUnfoldTreeList()));
  m_debugDrawer.reset(new DebugDrawer(m_model3D->getUnfoldMesh(), m_model->getModel2D()->getPolygonList()));
}

// Public

void Widget3D::bufferOriginalMesh(){
  m_originalDrawer->buffer();
}

void Widget3D::bufferUnfoldMesh(){
  m_unfoldDrawer->buffer();
  m_unfoldTreeDrawer->buffer();
  m_debugDrawer->buffer();
}

void Widget3D::bufferUnfoldTree(){
  m_unfoldTreeDrawer->buffer();
}

void Widget3D::resetDrawProperties(){
  m_unfoldDrawer->setFaceColors(Eigen::MatrixXf::Zero(0, 0));
}

void Widget3D::setColors(const Eigen::MatrixXf& colors){
  m_unfoldDrawer->setFaceColors(colors);
}

void Widget3D::updateDebugMeshVertices(){
  m_debugDrawer->updateAllVertices();
}

void Widget3D::updateUnfoldMeshVertices(){
  m_unfoldDrawer->updateAllVertices();
}

void Widget3D::updateUnfoldTree(){
  m_unfoldTreeDrawer->updateAllNodes();
}

// Private

void Widget3D::initShaders()
{
  if(!m_originalMeshDrawProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, "shader/originalMeshVertexShader.glsl") ||
     !m_originalMeshDrawProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, "shader/originalMeshFragmentShader.glsl") ||
     !m_originalMeshDrawProgram->link() ||
     !m_originalMeshDrawProgram->bind()){
    close();
  }
  if(!m_unfoldMeshDrawProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, "shader/unfoldMeshVertexShader.glsl") ||
     !m_unfoldMeshDrawProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, "shader/unfoldMeshFragmentShader.glsl") ||
     !m_unfoldMeshDrawProgram->link() ||
     !m_unfoldMeshDrawProgram->bind()){
    close();
  }
  if(!m_debugMeshDrawProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, "shader/unfoldMeshVertexShader.glsl") ||
     !m_debugMeshDrawProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, "shader/unfoldMeshFragmentShader.glsl") ||
     !m_debugMeshDrawProgram->link() ||
     !m_debugMeshDrawProgram->bind()){
    close();
  }
  if(!m_unfoldTreeDrawProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, "shader/unfoldTreeVertexShader.glsl") ||
     !m_unfoldTreeDrawProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, "shader/unfoldTreeFragmentShader.glsl") ||
     !m_unfoldTreeDrawProgram->link() ||
     !m_unfoldTreeDrawProgram->bind()){
    close();
  }
  if(!m_cutTreeDrawProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, "shader/cutTreeVertexShader.glsl") ||
     !m_cutTreeDrawProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, "shader/cutTreeFragmentShader.glsl") ||
     !m_cutTreeDrawProgram->link() ||
     !m_cutTreeDrawProgram->bind()){
    close();
  }
}

// Protected

void Widget3D::initializeGL()
{
  initializeOpenGLFunctions();
  std::cout << glGetString(GL_VERSION) << std::endl;
  QColor bgColor = Colorpalette::widgetBackgroundColor;
  glClearColor(bgColor.red() / 255.f, bgColor.green() / 255.f, bgColor.blue() / 255.f, 1);
  initShaders();

  glEnable(GL_MULTISAMPLE);

  glEnable(GL_DEPTH_TEST);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void Widget3D::mouseDoubleClickEvent(QMouseEvent* event){
  this->paintGL();
  event->accept();
}

void Widget3D::mouseMoveEvent(QMouseEvent* event){
  if(Controls::Mouse::isMoveViewButtonPressed()){
    QVector2D diff = QVector2D(event->localPos()) - m_mousePressPosition;
    m_rotationAxis = QVector3D(diff.y(), diff.x(), 0.0).normalized();
    float acc = diff.length() / 2.f;
    m_rotation = QQuaternion::fromAxisAndAngle(m_rotationAxis, acc) * m_rotation;
    m_mousePressPosition = QVector2D(event->localPos());
  }
  update();
  event->accept();
}

void Widget3D::mousePressEvent(QMouseEvent* event){
  if(Controls::Mouse::isMoveViewButtonPressed()){
    m_mousePressPosition = QVector2D(event->localPos());
  }
  update();
  event->accept();
}

void Widget3D::mouseReleaseEvent(QMouseEvent* event){
  m_mousePressPosition = QVector2D(event->localPos());
  update();
  event->accept();
}

void Widget3D::paintGL()
{
  makeCurrent();
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  m_modelview = QMatrix4x4();

  m_modelview.translate(0.0, 0.0, m_cameraPos);
  m_modelview.rotate(m_rotation);

  bool invertable;
  QMatrix4x4 normalMatrix = m_modelview.inverted(&invertable);
  if(!invertable){
    std::cerr << "Modelview Matrix is not invertable." << std::endl;
    return;
  }

  m_originalMeshDrawProgram->bind();
  m_originalMeshDrawProgram->setUniformValue("mvpMatrix", m_projection * m_modelview);
  m_originalMeshDrawProgram->setUniformValue("normalMatrix", normalMatrix.transposed());
  m_originalMeshDrawProgram->release();

  m_unfoldMeshDrawProgram->bind();
  m_unfoldMeshDrawProgram->setUniformValue("mvpMatrix", m_projection * m_modelview);
  m_unfoldMeshDrawProgram->setUniformValue("normalMatrix", normalMatrix.transposed());
  if(m_drawState & DrawState::DRAW_COLORGRADIENT){
    m_unfoldMeshDrawProgram->setUniformValue("useIndividualColors", true);
  }else{
    m_unfoldMeshDrawProgram->setUniformValue("useIndividualColors", false);
  }
  m_unfoldMeshDrawProgram->release();

  m_cutTreeDrawProgram->bind();
  m_cutTreeDrawProgram->setUniformValue("mvpMatrix", m_projection * m_modelview);
  m_cutTreeDrawProgram->release();

  m_unfoldTreeDrawProgram->bind();
  m_unfoldTreeDrawProgram->setUniformValue("mvpMatrix", m_projection * m_modelview);
  m_unfoldTreeDrawProgram->setUniformValue("rootNodeIndex", m_model->getUnfolder()->getUnfoldTreeList().getRootIndex());
  m_unfoldTreeDrawProgram->release();

  m_debugMeshDrawProgram->bind();
  m_debugMeshDrawProgram->setUniformValue("mvpMatrix", m_projection * m_modelview);
  m_debugMeshDrawProgram->setUniformValue("normalMatrix", normalMatrix.transposed());
  if(m_drawState & DrawState::DRAW_COLORGRADIENT){
    m_debugMeshDrawProgram->setUniformValue("useIndividualColors", true);
  }else{
    m_debugMeshDrawProgram->setUniformValue("useIndividualColors", false);
  }
  m_debugMeshDrawProgram->release();

  if(m_drawState & DrawState::DRAW_UNFOLD){
    m_unfoldDrawer->draw(m_unfoldMeshDrawProgram.get());
  }
  if(m_drawState & DrawState::DRAW_UNFOLDTREE){
    m_unfoldTreeDrawer->draw(m_unfoldTreeDrawProgram.get());
  }
  if(m_drawState & DrawState::DRAW_CUTTREE){
    m_cutTreeDrawer->draw(m_cutTreeDrawProgram.get());
  }
  if(m_drawState & DrawState::DRAW_DEBUG){
    m_debugDrawer->updateAllVertices();
    m_debugDrawer->draw(m_debugMeshDrawProgram.get());
  }
  if(m_drawState & DrawState::DRAW_ORIGINAL){
    m_originalDrawer->draw(m_originalMeshDrawProgram.get());
  }
  GLenum glError = glGetError();
  if(glError){
    std::cerr << "Widget3D: " << glError << std::endl;
  }
  doneCurrent();
}

void Widget3D::wheelEvent(QWheelEvent* event){
  // One mousetick is a delta of 120
  float delta = event->angleDelta().y() / 120.f;
  m_cameraPos *= (1.f - delta / 12.f);
  m_cameraPos = std::min(-0.01f, m_cameraPos);
  const Eigen::AlignedBox3d& bBox = m_model3D->getOriginalMesh()->getBoundingBox();
  const float diagonalLength = static_cast<float>(bBox.diagonal().norm());
  if(std::fabs(m_cameraPos) > 10.f * diagonalLength){
    m_cameraPos = -10.f * diagonalLength;
  }
  update();
  event->accept();
}

void Widget3D::resizeGL(int w, int h)
{
  float aspect = static_cast<float>(w) / static_cast<float>(h ? h : 1);
  const float zNear = .1f, zFar = 100000.0f, fov = 60.0f;
  m_projection.setToIdentity();
  m_projection.perspective(fov, aspect, zNear, zFar);
}

// Public Slots

void Widget3D::toggleDrawColorGradient(bool toggle){
  if(toggle){
    m_drawState |= DrawState::DRAW_COLORGRADIENT;
  }else{
    m_drawState &= (-1 ^ DrawState::DRAW_COLORGRADIENT);
  }
  update();
}

void Widget3D::toggleDrawCutTree(bool toggle){
  if(toggle){
    m_drawState |= DrawState::DRAW_CUTTREE;
  }else{
    m_drawState &= (-1 ^ DrawState::DRAW_CUTTREE);
  }
  update();
}

void Widget3D::toggleDrawDebugMesh(bool toggle){
  if(toggle){
    m_drawState |= DrawState::DRAW_DEBUG;
  }else{
    m_drawState &= (-1 ^ DrawState::DRAW_DEBUG);
  }
  update();
}

void Widget3D::toggleDrawOriginalMesh(bool toggle){
  if(toggle){
    m_drawState |= DrawState::DRAW_ORIGINAL;
  }else{
    m_drawState &= (-1 ^ DrawState::DRAW_ORIGINAL);
  }
  update();
}

void Widget3D::toggleDrawUnfoldMesh(bool toggle){
  if(toggle){
    m_drawState |= DrawState::DRAW_UNFOLD;
  }else{
    m_drawState &= (-1 ^ DrawState::DRAW_UNFOLD);
  }
  update();
}

void Widget3D::toggleDrawUnfoldTree(bool toggle){
  if(toggle){
    m_drawState |= DrawState::DRAW_UNFOLDTREE;
  }else{
    m_drawState &= (-1 ^ DrawState::DRAW_UNFOLDTREE);
  }
  update();
}

void Widget3D::toggleHighlightRoot(bool toggle){
  m_unfoldDrawer->toggleHightlightRoot(toggle);
  m_unfoldTreeDrawer->toggleHighlightRoot(toggle);
  update();
}

void Widget3D::toggleScreenshotLines(bool toggle){
  m_cutTreeDrawer->toggleScreenshotLines(toggle);
  m_debugDrawer->toggleScreenshotLines(toggle);
  m_originalDrawer->toggleScreenshotLines(toggle);
  m_unfoldDrawer->toggleScreenshotLines(toggle);
  m_unfoldTreeDrawer->toggleScreenshotLines(toggle);
  update();
}

void Widget3D::updateCameraPos(){
  const Eigen::AlignedBox3d& bBox = m_model3D->getOriginalMesh()->getBoundingBox();
  float diagonalLength = -static_cast<float>(bBox.diagonal().norm());
  m_cameraPos = diagonalLength * 1.5f;
  update();
}

void Widget3D::updateUnfoldTree(const std::pair<int, int>& oldConnection, const std::pair<int, int>& newConnection){
  m_unfoldTreeDrawer->updateNode(oldConnection, newConnection);
  update();
}
