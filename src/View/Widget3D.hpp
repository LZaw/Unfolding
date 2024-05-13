
#pragma once

// C++ Headers
#include <memory>
#include <utility>

// Project Headers
#include <Model/Model3D.hpp>
#include <Model/UnfoldModel.hpp>
#include <View/Drawer/CutTreeDrawer.hpp>
#include <View/Drawer/DebugDrawer.hpp>
#include <View/Drawer/OriginalMeshDrawer.hpp>
#include <View/Drawer/UnfoldMeshDrawer.hpp>
#include <View/Drawer/UnfoldTreeDrawer.hpp>

// QT Headers
#include <QOpenGLFunctions_4_5_Compatibility>
#include <QOpenGLShaderProgram>
#include <QOpenGLWidget>

class Widget3D : public QOpenGLWidget, public QOpenGLFunctions_4_5_Compatibility
{
  Q_OBJECT

public:
  explicit Widget3D(UnfoldModel* model, QWidget* parent = nullptr);

  void bufferOriginalMesh();
  void bufferUnfoldMesh();
  void bufferUnfoldTree();

  void resetDrawProperties();

  void setColors(const Eigen::MatrixXf& colors);

  void updateDebugMeshVertices();
  void updateUnfoldMeshVertices();
  void updateUnfoldTree();


private:
  enum DrawState
  {
    DRAW_ORIGINAL = 1 << 0,
    DRAW_UNFOLD = 1 << 1,
    DRAW_DEBUG = 1 << 2,
    DRAW_UNFOLDTREE = 1 << 3,
    DRAW_CUTTREE = 1 << 4,
    DRAW_COLORGRADIENT = 1 << 5,
  };
  float                   m_cameraPos;
  std::unique_ptr<
    CutTreeDrawer>        m_cutTreeDrawer;
  std::unique_ptr<
    QOpenGLShaderProgram> m_cutTreeDrawProgram;
  std::unique_ptr<
    DebugDrawer>          m_debugDrawer;
  std::unique_ptr<
    QOpenGLShaderProgram> m_debugMeshDrawProgram;
  unsigned int            m_drawState;
  UnfoldModel*            m_model;
  const Model3D*          m_model3D;
  QMatrix4x4              m_modelview;
  QVector2D               m_mousePressPosition;
  std::unique_ptr<
    OriginalMeshDrawer>   m_originalDrawer;
  std::unique_ptr<
    QOpenGLShaderProgram> m_originalMeshDrawProgram;
  QMatrix4x4              m_projection;
  QQuaternion             m_rotation;
  QVector3D               m_rotationAxis;
  std::unique_ptr<
    UnfoldMeshDrawer>     m_unfoldDrawer;
  std::unique_ptr<
    QOpenGLShaderProgram> m_unfoldMeshDrawProgram;
  std::unique_ptr<
    UnfoldTreeDrawer>     m_unfoldTreeDrawer;
  std::unique_ptr<
    QOpenGLShaderProgram> m_unfoldTreeDrawProgram;

  void initShaders();

protected:
  void initializeGL() override;

  virtual void mouseDoubleClickEvent(QMouseEvent* event) override;
  virtual void mouseMoveEvent(QMouseEvent* event) override;
  virtual void mousePressEvent(QMouseEvent* event) override;
  virtual void mouseReleaseEvent(QMouseEvent* event) override;

  void paintGL() override;

  void resizeGL(int w, int h) override;

  virtual void wheelEvent(QWheelEvent* e) override;

signals:
  void requestUpdate2D();

public slots:
  void toggleDrawColorGradient(bool toggle);
  void toggleDrawCutTree(bool toggle);
  void toggleDrawDebugMesh(bool toggle);
  void toggleDrawOriginalMesh(bool toggle);
  void toggleDrawUnfoldMesh(bool toggle);
  void toggleDrawUnfoldTree(bool toggle);
  void toggleHighlightRoot(bool toggle);
  void toggleScreenshotLines(bool toggle);

  void updateCameraPos();
  void updateUnfoldTree(const std::pair<int, int>& oldConnection, const std::pair<int, int>& newConnection);
};
