
#pragma once

// C++ Headers
#include <utility>
#include <vector>

// Project Headers
#include <Datastructures/Polygon/Polygon2D.hpp>
#include <Model/Model2D.hpp>
#include <Model/UnfoldModel.hpp>
#include <View/Variables/Colorpalette.hpp>

// Eigen Headers
#include <Eigen/Dense>

// QT Headers
#include <QPainter>
#include <QPaintEvent>
#include <QWidget>

class Widget2D : public QWidget
{
  Q_OBJECT
public:
  explicit Widget2D(UnfoldModel* model, QWidget* parent = nullptr);

  void calibrateViewParameters();

  void resetSelections();

private:
  enum DrawState
  {
    DRAW_UNFOLDTREE = 1 << 0,
    DRAW_DEBUG = 1 << 1,
    DRAW_FILLED = 1 << 2,
    DRAW_HIGHLIGHT_ROOT = 1 << 3,
    DRAW_SCREENSHOT_LINES = 1 << 4,
  };

  unsigned int              m_drawState;
  Eigen::Vector2d           m_globalTranslation;
  UnfoldModel*              m_model;
  const Model2D*            m_model2D;
  Eigen::Vector2d           m_mouseAncor;
  std::vector<unsigned int> m_possibleNeighbors;
  double                    m_scalingFactor;
  int                       m_selectedPolygonIndex;
  std::vector<unsigned int> m_selectedPolygonList;
  std::vector<Polygon2D>    m_simulatedMovePolygonList;
  std::vector<unsigned int> m_tailPolygons;
  Eigen::Vector2d           m_translation;

  bool calculateIsPossibleNeighbor(const unsigned int index) const;
  void calculatePreviewByMouseCoordinates(const Eigen::Vector2d& mousePositionGlobalCoordinates);
  void clearPreview();

  void drawCentroids(QPainter& painter, const PolygonList& polygonList) const;
  void drawLine(QPainter& painter, const Eigen::Vector2d& begin, const Eigen::Vector2d& end) const;
  void drawPoint(QPainter& painter, const Eigen::Vector2d& point) const;
  void drawPolygon(QPainter& painter, const Polygon2D& polygon) const;
  void drawUnfoldTree(QPainter& painter, const PolygonList& polygonList) const;
  void drawUnfoldTreeRoot(QPainter& painter, const PolygonList& polygonList) const;
  void drawVertices(QPainter& painter, const PolygonList& polygonList) const;

  void fillPolygon(QPainter& painter, const Polygon2D& polygon) const;

  bool isPolygonSelected() const;

  void paintScene(QPainter &painter) const;
  void processSelectPolygonClick(const Eigen::Vector2d& clickPositionGlobalCoordinates);

  Eigen::Vector2d scaleAndTranslate(const Eigen::Vector2d& point) const;
  void selectPolygon(const int index);
  void setupPolygonSelection(std::vector<unsigned int>& clickedPolygons);

  void unselectPolygon();

protected:
  void keyPressEvent(QKeyEvent* event) override;
  void keyReleaseEvent(QKeyEvent* event) override;

  void mouseMoveEvent(QMouseEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;

  void paintEvent(QPaintEvent* event) override;

  void resizeEvent(QResizeEvent* event) override;

  void wheelEvent(QWheelEvent* event) override;

signals:
  void requestUpdate3D();
  void updateUnfoldTree(const std::pair<int, int>& oldConnection, const std::pair<int, int>& newConnection);

public slots:
  void toggleDrawColorGradient(bool toggle);
  void toggleDrawDebug(bool toggle);
  void toggleDrawUnfoldTree(bool toggle);
  void toggleHighlightRoot(bool toggle);
  void toggleScreenshotLines(bool toggle);
};
