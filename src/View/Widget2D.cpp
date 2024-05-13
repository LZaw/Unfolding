
#include "Widget2D.hpp"

// C++ Headers
#include <algorithm>

// Project Headers
#include <Datastructures/Polygon/PolygonList.hpp>
#include <View/Variables/Controls.hpp>

// QT Headers
#include <QApplication>
#include <QPainterPath>

Widget2D::Widget2D(UnfoldModel* model, QWidget* parent):
  QWidget(parent),
  m_drawState(0),
  m_model(model),
  m_model2D(m_model->getModel2D()),
  m_scalingFactor(1),
  m_selectedPolygonIndex(-1){
  m_translation.setZero();
  m_mouseAncor.setZero();
  m_globalTranslation.setZero();
}

// Public

void Widget2D::calibrateViewParameters(){
  const Eigen::Vector2d& lBound = m_model->getModel2D()->getBoundingBox().min();
  const Eigen::Vector2d& bBoxDiagonal = m_model->getModel2D()->getBoundingBox().diagonal();
  m_scalingFactor = std::min(fabs(this->width() / bBoxDiagonal.x()), fabs(this->height() / bBoxDiagonal.y()));
  m_globalTranslation = Eigen::Vector2d(this->width() / 2., this->height() / 2.);
  Eigen::Vector2d midshift = lBound + 0.5 * bBoxDiagonal;
  m_translation = -Eigen::Vector2d(midshift.x() * fabs(this->width() / bBoxDiagonal.x()), midshift.y() * fabs(this->height() / bBoxDiagonal.y())) * 0.5;
  m_scalingFactor *= 0.6;
}

void Widget2D::resetSelections(){
  unselectPolygon();
}

// Private

bool Widget2D::calculateIsPossibleNeighbor(const unsigned int index) const{
  return std::find_if(
           m_possibleNeighbors.begin(),
           m_possibleNeighbors.end(),
           [index](const unsigned int possibleNeighbor){
             return index == possibleNeighbor;
           }
         ) != m_possibleNeighbors.end();
}

void Widget2D::calculatePreviewByMouseCoordinates(const Eigen::Vector2d& mousePositionGlobalCoordinates){
  std::vector<unsigned int> hoveredPolygons = m_model2D->calculateIntersectingPolygonIndicesByCoordinate(mousePositionGlobalCoordinates);
  for(unsigned int possibleNeighborIndex: hoveredPolygons){
    if(calculateIsPossibleNeighbor(possibleNeighborIndex)){
      m_simulatedMovePolygonList = m_model2D->getPolygonList().previewMove(m_selectedPolygonList[m_selectedPolygonIndex], possibleNeighborIndex);
      update();
      return;
    }
  }
}

void Widget2D::clearPreview(){
  m_simulatedMovePolygonList.clear();
  update();
}

void Widget2D::drawCentroids(QPainter &painter, const PolygonList &polygonList) const{
  for(std::vector<Polygon2D>::const_iterator polygonIterator = polygonList.cbegin(); polygonIterator != polygonList.cend(); ++polygonIterator){
    drawPoint(painter, polygonIterator->m_center);
  }
}

void Widget2D::drawLine(QPainter& painter, const Eigen::Vector2d& begin, const Eigen::Vector2d& end) const{
  Eigen::Vector2d scaledAndTranslatedBegin = scaleAndTranslate(begin);
  Eigen::Vector2d scaledAndTranslatedEnd = scaleAndTranslate(end);
  painter.drawLine(scaledAndTranslatedBegin.x(), scaledAndTranslatedBegin.y(),
                   scaledAndTranslatedEnd.x(), scaledAndTranslatedEnd.y());
}

void Widget2D::drawPoint(QPainter& painter, const Eigen::Vector2d& point) const{
  painter.drawPoint(point.x() * m_scalingFactor + m_translation.x() + m_globalTranslation.x(), point.y() * m_scalingFactor + m_translation.y() + m_globalTranslation.y());
}

void Widget2D::drawPolygon(QPainter& painter, const Polygon2D& polygon) const{
  for(unsigned int i = 0; i < polygon.m_vertexList.size(); ++i){
    drawLine(painter, polygon.m_vertexList[i], polygon.m_vertexList[(i + 1) % polygon.m_vertexList.size()]);
  }
}

void Widget2D::drawUnfoldTree(QPainter& painter, const PolygonList& polygonList) const{
  const UnfoldTreeList& unfoldTreeList = m_model->getUnfolder()->getUnfoldTreeList();
  for(UnfoldTreeList::const_iterator unfoldTreeIterator = unfoldTreeList.cbegin(); unfoldTreeIterator != unfoldTreeList.cend(); ++unfoldTreeIterator){
    if((*unfoldTreeIterator)->m_parent == nullptr){
      continue;
    }
    int nodeIndex = (*unfoldTreeIterator)->m_ref;
    int parentIndex = (*unfoldTreeIterator)->m_parent->m_ref;
    drawLine(painter, polygonList[nodeIndex].m_center, polygonList[parentIndex].m_center);
  }
}

void Widget2D::drawUnfoldTreeRoot(QPainter& painter, const PolygonList& polygonList) const{
  // Tree not initialized yet
  if(m_model->getUnfolder()->getUnfoldTree() == nullptr ||
     m_model->getUnfolder()->getUnfoldTree()->m_ref == -1){
    return;
  }
  const Polygon2D& rootPolygon = polygonList[m_model->getUnfolder()->getUnfoldTree()->m_ref];
  drawPoint(painter, rootPolygon.m_center);
}

void Widget2D::fillPolygon(QPainter &painter, const Polygon2D& polygon) const{
  QPainterPath path;
  const Eigen::Vector2d start = scaleAndTranslate(polygon.m_vertexList[0]);
  path.moveTo(start.x(), start.y());
  for(std::vector<Eigen::Vector2d>::const_iterator vertexIt = polygon.m_vertexList.cbegin(); vertexIt != polygon.m_vertexList.cend(); ++vertexIt){
    const Eigen::Vector2d& p = scaleAndTranslate(*vertexIt);
    path.lineTo(p.x(), p.y());
  }
  path.lineTo(start.x(), start.y());
  const Eigen::Vector3f& faceColor = polygon.m_color;
  QColor color(faceColor[0], faceColor[1], faceColor[2]);
  painter.fillPath(path, color);
}

bool Widget2D::isPolygonSelected() const{
  return m_selectedPolygonIndex != -1;
}

void Widget2D::paintScene(QPainter& painter) const{
  const PolygonList polygonList = m_model2D->getPolygonList();
  const std::vector<std::vector<unsigned int>> collisionList = m_model2D->getCollisions();

  int lineThickness = 3;
  if(m_drawState & DRAW_SCREENSHOT_LINES){
    lineThickness = 6;
  }

  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setRenderHint(QPainter::TextAntialiasing, true);

  // All polygons, filled
  painter.setPen(QPen(Colorpalette::unfoldPolygonFrameColor, lineThickness, Qt::SolidLine));
  if(m_drawState & DrawState::DRAW_FILLED){
    for(auto it = polygonList.cbegin(); it != polygonList.cend(); ++it){
      fillPolygon(painter, *it);
    }
  }

  // All polygons, standard color
  painter.setPen(QPen(Colorpalette::unfoldPolygonFrameColor, lineThickness, Qt::SolidLine));
  for(auto it = polygonList.cbegin(); it != polygonList.cend(); ++it){
    drawPolygon(painter, *it);
  }

  // Collisions
  painter.setPen(QPen(Colorpalette::criticalColor, lineThickness, Qt::SolidLine));
  for(unsigned int i = 0; i < collisionList.size(); ++i){
    if(!collisionList[i].empty()){
      drawPolygon(painter, polygonList[i]);
    }
  }

  // Tailpolygons
  painter.setPen(QPen(Colorpalette::lowlightColor, lineThickness, Qt::SolidLine));
  for(std::vector<unsigned int>::const_iterator tailIt = m_tailPolygons.cbegin(); tailIt != m_tailPolygons.cend(); ++tailIt){
    drawPolygon(painter, polygonList[*tailIt]);
  }

  // Root Polygon
  if(m_model->getUnfolder()->getUnfoldTree() &&
     m_model->getUnfolder()->getUnfoldTree()->m_ref != -1 &&
     m_drawState & DrawState::DRAW_HIGHLIGHT_ROOT){
    painter.setPen(QPen(Colorpalette::rootPolygonColor, lineThickness, Qt::SolidLine));
    drawPolygon(painter, polygonList[m_model->getUnfolder()->getUnfoldTree()->m_ref]);
  }

  // If Selection exists
  if(isPolygonSelected()){
    unsigned int selectedPolygonGlobalIndex = m_selectedPolygonList[m_selectedPolygonIndex];

    // Possible Neighbors
    painter.setPen(QPen(Colorpalette::neighborColor, lineThickness, Qt::SolidLine));
    for(unsigned int possibleNeighbor: m_possibleNeighbors){
      drawPolygon(painter, polygonList[possibleNeighbor]);
    }

    // Currently Selected Polygon
    painter.setPen(QPen(Colorpalette::highlightColor, lineThickness, Qt::SolidLine));
    drawPolygon(painter, polygonList[selectedPolygonGlobalIndex]);
  }

  // Simualted Polygons
  painter.setPen(QPen(Colorpalette::predictingColor, lineThickness, Qt::SolidLine));
  for(const Polygon2D& simulatedPolygon: m_simulatedMovePolygonList){
    drawPolygon(painter, simulatedPolygon);
  }

  // UnfoldTree
  if(m_drawState & DrawState::DRAW_UNFOLDTREE){
    // Lines
    painter.setPen(QPen(Colorpalette::unfoldTreeColor, lineThickness, Qt::SolidLine));
    drawUnfoldTree(painter, polygonList);

    // Vertices
    painter.setPen(QPen(Colorpalette::unfoldTreeColor, 7, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    drawCentroids(painter, polygonList);

    // Root
    if(m_drawState & DrawState::DRAW_HIGHLIGHT_ROOT){
      painter.setPen(QPen(Colorpalette::unfoldTreeRootColor, 7, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
      drawUnfoldTreeRoot(painter, polygonList);
    }
  }

  // Debug drawing
  if(m_drawState & DrawState::DRAW_DEBUG){
    // Temporärer Code, nicht commiten.
  }
}

void Widget2D::processSelectPolygonClick(const Eigen::Vector2d& clickPositionGlobalCoordinates){
  std::vector<unsigned int> clickedPolygons = m_model2D->calculateIntersectingPolygonIndicesByCoordinate(clickPositionGlobalCoordinates);
  if(isPolygonSelected()){
    // Select Neighbor
    for(unsigned int clickedPolygonIndex: clickedPolygons){
      std::vector<unsigned int>::const_iterator foundNeighborIterator =
          std::find_if(
            m_possibleNeighbors.begin(),
            m_possibleNeighbors.end(),
            [clickedPolygonIndex](unsigned int index){
              return index == clickedPolygonIndex;
            }
          );
      if(foundNeighborIterator != m_possibleNeighbors.end()){
        const UnfoldTree* selectedPolygon = m_model->getUnfolder()->getUnfoldTree()->findConstTreeNodeByIndex(m_selectedPolygonList[m_selectedPolygonIndex]);
        m_model->attachPolygonToNewNeighbor(m_selectedPolygonList[m_selectedPolygonIndex], *foundNeighborIterator);
        emit(updateUnfoldTree({selectedPolygon->m_ref, selectedPolygon->m_parent->m_ref}, {selectedPolygon->m_ref, *foundNeighborIterator}));
        unselectPolygon();
        return;
      }
    }
  }
  // New Polygon
  setupPolygonSelection(clickedPolygons);
}

Eigen::Vector2d Widget2D::scaleAndTranslate(const Eigen::Vector2d& point) const{
  return point * m_scalingFactor + m_translation + m_globalTranslation;
}

void Widget2D::selectPolygon(const int index){
  clearPreview();
  m_selectedPolygonIndex = index;
  m_possibleNeighbors = m_model2D->calculatePossibleNeighbors(m_selectedPolygonList[m_selectedPolygonIndex]);
  m_tailPolygons = m_model2D->calculateTailPolygons(m_selectedPolygonList[m_selectedPolygonIndex]);
}

void Widget2D::setupPolygonSelection(std::vector<unsigned int>& clickedPolygons){
  if(clickedPolygons.empty()){
    unselectPolygon();
  }else{
    m_selectedPolygonList = std::move(clickedPolygons);
    selectPolygon(0);
  }
}

void Widget2D::unselectPolygon(){
  m_selectedPolygonIndex = -1;
  m_selectedPolygonList.clear();
  m_possibleNeighbors.clear();
  m_tailPolygons.clear();
  m_simulatedMovePolygonList.clear();
  clearPreview();
}

// Protected

void Widget2D::keyPressEvent(QKeyEvent* event){
  if(isPolygonSelected()){
    if(event->key() == Controls::Keyboard::chooseNextPossiblePolygonKey){
      selectPolygon((m_selectedPolygonIndex + 1) % m_selectedPolygonList.size());
    }
    if(event->key() == Controls::Keyboard::choosePriorPossiblePolygonKey){
      selectPolygon((m_selectedPolygonIndex - 1 + m_selectedPolygonList.size()) % m_selectedPolygonList.size());
    }
    if(event->key() == Controls::Keyboard::previewKey){
      Eigen::Vector2d mousePositionGlobalCoordinates((mapFromGlobal(QCursor::pos()).x() - m_translation.x() - m_globalTranslation.x()) / m_scalingFactor, (mapFromGlobal(QCursor::pos()).y() - m_translation.y() - m_globalTranslation.y()) / m_scalingFactor);
      calculatePreviewByMouseCoordinates(mousePositionGlobalCoordinates);
    }
    if(event->key() == Controls::Keyboard::rerootKey){
      m_model->rerootUnfoldTree(m_selectedPolygonList[m_selectedPolygonIndex], true);
      unselectPolygon();
      emit(requestUpdate3D());
    }
    if(event->key() == Controls::Keyboard::selectParentKey){
      std::vector<unsigned int> parentPolygonList(1);
      parentPolygonList[0] = m_model2D->calculateParentPolygon(m_selectedPolygonList[m_selectedPolygonIndex]);
      if(parentPolygonList[0] != m_selectedPolygonList[m_selectedPolygonIndex]){
        setupPolygonSelection(parentPolygonList);
      }
    }
  }
  update();
  event->accept();
}

void Widget2D::keyReleaseEvent(QKeyEvent* event){
  if(event->key() == Controls::Keyboard::previewKey){
    clearPreview();
  }
  event->accept();
}

void Widget2D::mouseMoveEvent(QMouseEvent* event){
  if(Controls::Mouse::isMoveViewButtonPressed()){
    Eigen::Vector2d diff = Eigen::Vector2d(event->localPos().x(), event->localPos().y()) - m_mouseAncor;
    m_translation += diff;
    m_mouseAncor = Eigen::Vector2d(event->localPos().x(), event->localPos().y());
    update();
  }
  if(Controls::Keyboard::isPreviewKeyPressed()){
    Eigen::Vector2d mousePositionGlobalCoordinates((event->localPos().x() - m_translation.x() - m_globalTranslation.x()) / m_scalingFactor, (event->localPos().y() - m_translation.y() - m_globalTranslation.y()) / m_scalingFactor);
    calculatePreviewByMouseCoordinates(mousePositionGlobalCoordinates);
  }
  event->accept();
}

void Widget2D::mousePressEvent(QMouseEvent* event){
  if(Controls::Mouse::isSelectButtonPressed()){
    m_mouseAncor = Eigen::Vector2d(event->localPos().x(), event->localPos().y());
    Eigen::Vector2d clickPositionGlobalCoordinates((event->localPos().x() - m_translation.x() - m_globalTranslation.x()) / m_scalingFactor, (event->localPos().y() - m_translation.y() - m_globalTranslation.y()) / m_scalingFactor);
    processSelectPolygonClick(clickPositionGlobalCoordinates);
  }
  if(Controls::Mouse::isMoveViewButtonPressed()){
    m_mouseAncor = Eigen::Vector2d(event->localPos().x(), event->localPos().y());
  }
  event->accept();
}

void Widget2D::mouseReleaseEvent(QMouseEvent* event){
  update();
  event->accept();
}

void Widget2D::paintEvent(QPaintEvent* event){
  QPainter painter;
  painter.begin(this);
  paintScene(painter);
  painter.end();
  event->accept();
}

void Widget2D::resizeEvent(QResizeEvent* event){
  m_globalTranslation = Eigen::Vector2d(width() / 2., height() / 2.);
  event->accept();
}

void Widget2D::wheelEvent(QWheelEvent* event){
  double oldScalingFactor = m_scalingFactor;
  // One mousetick is a delta of 120
  double delta = event->angleDelta().y() / 120.;
  m_scalingFactor *= (1. + delta / 12.);
  m_translation.x() -= event->pos().x() - m_globalTranslation.x();
  m_translation.y() -= event->pos().y() - m_globalTranslation.y();
  m_translation *= m_scalingFactor / oldScalingFactor;
  m_translation.x() += event->pos().x() - m_globalTranslation.x();
  m_translation.y() += event->pos().y() - m_globalTranslation.y();
  update();
  event->accept();
}

// Public Slots

void Widget2D::toggleDrawColorGradient(bool toggle){
  if(toggle){
    m_drawState |= DrawState::DRAW_FILLED;
  }else{
    m_drawState &= (-1 ^ DrawState::DRAW_FILLED);
  }
  update();
}

void Widget2D::toggleDrawDebug(bool toggle){
  if(toggle){
    m_drawState |= DrawState::DRAW_DEBUG;
  }else{
    m_drawState &= (-1 ^ DrawState::DRAW_DEBUG);
  }
  update();
}

void Widget2D::toggleDrawUnfoldTree(bool toggle){
  if(toggle){
    m_drawState |= DrawState::DRAW_UNFOLDTREE;
  }else{
    m_drawState &= (-1 ^ DrawState::DRAW_UNFOLDTREE);
  }
  update();
}

void Widget2D::toggleHighlightRoot(bool toggle){
  if(toggle){
    m_drawState |= DrawState::DRAW_HIGHLIGHT_ROOT;
  }else{
    m_drawState &= (-1 ^ DrawState::DRAW_HIGHLIGHT_ROOT);
  }
  update();
}

void Widget2D::toggleScreenshotLines(bool toggle){
  if(toggle){
    m_drawState |= DrawState::DRAW_SCREENSHOT_LINES;
  }else{
    m_drawState &= (-1 ^ DrawState::DRAW_SCREENSHOT_LINES);
  }
  update();
}