
#include "SVGWriter.hpp"

// C++ Headers
#include <iomanip>
#include <fstream>
#include <sstream>
#include <vector>

// Project Headers
#include <Util/FileUtil.hpp>

// Eigen Headers
#include <Eigen/Dense>

// Public

SVGWriter::SVGWriter():
  m_borderFactor(0.02),
  m_height(500),
  m_scalingFactor(1000.){
}

void SVGWriter::savePolygonsToSVG(const std::string& path, const PolygonList& polygonList){
  m_currentPolygonList = &polygonList;
  setupFile(path);

  // Draw Triangles
  writeComment("Polygons");
  setWriteMode(WriteMode::Polygons);
  for(auto it = polygonList.cbegin(); it != polygonList.cend(); ++it){
    drawPolygon(*it);
  }
  unsetWriteMode();

  drawEdges();

  finishFile();
}

void SVGWriter::saveUnfoldingToSVG(const std::string& path, const PolygonList& polygonList, bool withFaceIndices){
  m_currentPolygonList = &polygonList;
  setupFile(path);

  if(withFaceIndices){
    writeComment("Indices");
    setWriteMode(WriteMode::Indices);
    for(auto it = polygonList.cbegin(); it != polygonList.cend(); ++it){
      if(it->m_faceIndex == -1){
        continue;
      }
      drawPolygonIndex(*it);
    }
    unsetWriteMode();
  }

  drawEdges();

  finishFile();
}

void SVGWriter::saveUnfoldTreeToSVG(const std::string& path, const PolygonList& polygonList){
  m_currentPolygonList = &polygonList;
  setupFile(path);

  drawEdges();
  drawUnfoldTree();

  finishFile();
}

// Private

void SVGWriter::decreaseIndentation(){
  m_leadingWhiteSpaces = m_leadingWhiteSpaces.substr(0, std::max(static_cast<size_t>(0), m_leadingWhiteSpaces.size() - 2));
}

void SVGWriter::drawLine(const Eigen::Vector2d& v0, const Eigen::Vector2d& v1){
  const Eigen::Vector2d& transformedV0 = transformToGlobalCoordinates(v0);
  const Eigen::Vector2d& transformedV1 = transformToGlobalCoordinates(v1);
  m_file << m_leadingWhiteSpaces << "<line x1=\"" << transformedV0.x() << "\" y1=\"" << transformedV0.y() << "\" x2=\"" << transformedV1.x() << "\" y2=\"" << transformedV1.y() << "\" />\n";
}

void SVGWriter::drawEdges(){
  // Draw CutEdges
  writeComment("CutEdges");
  setWriteMode(WriteMode::CutEdges);
  std::vector<std::vector<std::pair<int, int>>> cutEdges(m_currentPolygonList->size());
  m_currentPolygonList->calculateCutEdges(cutEdges);
  for(std::vector<std::vector<std::pair<int, int>>>::const_iterator cutEdgesPerFaceIterator = cutEdges.cbegin(); cutEdgesPerFaceIterator != cutEdges.cend(); ++cutEdgesPerFaceIterator){
    int faceIndex = std::distance(cutEdges.cbegin(), cutEdgesPerFaceIterator);
    for(const std::pair<int, int>& cutEdge: *cutEdgesPerFaceIterator){
      drawLine((*m_currentPolygonList)[faceIndex].m_vertexList[cutEdge.first], (*m_currentPolygonList)[faceIndex].m_vertexList[cutEdge.second]);
    }
  }
  unsetWriteMode();

  // Draw MountainEdges
  writeComment("MountainEdges");
  setWriteMode(WriteMode::MountainEdges);
  for(PolygonList::const_iterator it = m_currentPolygonList->cbegin(); it != m_currentPolygonList->cend(); ++it){
    const Polygon2D& currentPolygon = (*it);
    if(currentPolygon.m_sign == 1){
      drawLine(currentPolygon.m_vertexList[currentPolygon.m_axisIndices.first], currentPolygon.m_vertexList[currentPolygon.m_axisIndices.second]);
    }
  }
  unsetWriteMode();

  // Draw ValleyEdges
  writeComment("ValleyEdges");
  setWriteMode(WriteMode::ValleyEdges);
  for(PolygonList::const_iterator it = m_currentPolygonList->cbegin(); it != m_currentPolygonList->cend(); ++it){
    const Polygon2D& currentPolygon = (*it);
    if(currentPolygon.m_sign == -1){
      drawLine(currentPolygon.m_vertexList[currentPolygon.m_axisIndices.first], currentPolygon.m_vertexList[currentPolygon.m_axisIndices.second]);
    }
  }
  unsetWriteMode();
}

void SVGWriter::drawPolygon(const Polygon2D& currentPolygon){
  m_file << m_leadingWhiteSpaces << "<polygon points=\"";
  // X Coordinates
  for(std::vector<Eigen::Vector2d>::const_iterator vertexIt = currentPolygon.m_vertexList.cbegin(); vertexIt != currentPolygon.m_vertexList.cend(); ++vertexIt){
    const Eigen::Vector2d v = transformToGlobalCoordinates(*vertexIt);
    m_file << v.x() << ", " << v.y() << " ";
  }
  m_file << "\" style=\"fill:#" << rgb2hex(currentPolygon.m_color) << ";stroke:#000000;stroke-width:0.1\" />\n";
}

void SVGWriter::drawPolygonIndex(const Polygon2D& currentPolygon){
  const Eigen::Vector2d globalCenter = transformToGlobalCoordinates(currentPolygon.m_center);
  m_file << m_leadingWhiteSpaces <<
            "<text "
            "x=\"" << globalCenter.x() << "\" "
            "y=\"" << globalCenter.y() << "\""
            ">" << currentPolygon.m_faceIndex << "</text>\n";
}

void SVGWriter::drawUnfoldTree(){
  writeComment("UnfoldTreeEdges");
  setWriteMode(WriteMode::UnfoldTreeEdges);
  for(std::vector<Polygon2D>::const_iterator polygonIterator = m_currentPolygonList->cbegin(); polygonIterator != m_currentPolygonList->cend(); ++polygonIterator){
    const UnfoldTree* parent = m_currentPolygonList->getUnfoldTreeList()->findConstTreeNodeByIndex(polygonIterator->m_faceIndex)->m_parent;
    if(parent == nullptr){
      continue;
    }
    drawLine(polygonIterator->m_center, (*m_currentPolygonList)[parent->m_ref].m_center);
  }
  unsetWriteMode();
}

void SVGWriter::finishFile(){
  writeFooter();
  m_file.close();
}

void SVGWriter::increaseIndentation(){
  m_leadingWhiteSpaces += "  ";
}

void SVGWriter::setupFile(const std::string& path){
  m_file.open(path, std::ios_base::out);
  std::string title = FileUtil::calculateFileTitle(path);
  writeHeader(title);
}

// TODO: Font size, stroke-width und stroke-dashoffset anhand der BoundingBox setzen.
void SVGWriter::setWriteMode(WriteMode mode){
  switch(mode){
  case WriteMode::Indices:{
    m_file << m_leadingWhiteSpaces <<
              "<g font-family=\"Sans Serif\" "
              "font-size=\"13\" "
              "font-style=\"normal\">\n";
    break;
  }
  case WriteMode::CutEdges:{
    m_file << m_leadingWhiteSpaces <<
              "<g fill=\"none\" "
              "stroke=\"#000000\" "
              "stroke-opacity=\"1\" "
              "stroke-width=\"1\" "
              "stroke-linecap=\"round\" "
              "stroke-linejoin=\"round\">\n";
    break;
  }
  case WriteMode::ValleyEdges:{
    m_file << m_leadingWhiteSpaces <<
              "<g fill=\"none\" "
              "stroke=\"#000000\" "
              "stroke-opacity=\"1\" "
              "stroke-dasharray=\"6,10\" "
              "stroke-dashoffset=\"0\" "
              "stroke-width=\"1\" "
              "stroke-linecap=\"round\" "
              "stroke-linejoin=\"round\">\n";
    break;
  }
  case WriteMode::MountainEdges:{
    m_file << m_leadingWhiteSpaces <<
              "<g fill=\"none\" "
              "stroke=\"#000000\" "
              "stroke-opacity=\"1\" "
              "stroke-dasharray=\"6,4,0.01,4\" "
              "stroke-dashoffset=\"0\" "
              "stroke-width=\"1\" "
              "stroke-linecap=\"round\" "
              "stroke-linejoin=\"round\" "
              "font-family=\"Sans Serif\">\n";
    break;
  }
  case WriteMode::Polygons:{
    m_file << m_leadingWhiteSpaces <<
              "<g "
              "stroke-linecap=\"round\" "
              "stroke-linejoin=\"round\">\n";
    break;
  }
  case WriteMode::UnfoldTreeEdges:{
    m_file << m_leadingWhiteSpaces <<
              "<g fill=\"none\" "
              "stroke=\"#4169E1\" "
              "stroke-opacity=\"1\" "
              "stroke-width=\"1\" "
              "stroke-linecap=\"round\" "
              "stroke-linejoin=\"round\">\n";
    break;
  }
  default:{
    return;
  }
  }
  increaseIndentation();
}

std::string SVGWriter::rgb2hex(const Eigen::Vector3f& rgb) const{
  std::stringstream ss;
  for(unsigned int i = 0; i < 3; ++i){
    ss << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(rgb[i]);
  }
  return ss.str();
}

Eigen::Vector2d SVGWriter::transformToGlobalCoordinates(const Eigen::Vector2d& localCoordinate) const{
  const Eigen::Vector2d boundingBoxDiagonal = m_currentPolygonList->getBoundingBox().diagonal();
  return m_scalingVector.array() * ((localCoordinate - m_currentPolygonList->getBoundingBox().min()).cwiseQuotient(boundingBoxDiagonal)).array();
}

void SVGWriter::unsetWriteMode(){
  decreaseIndentation();
  m_file << m_leadingWhiteSpaces << "</g>\n\n";
}

void SVGWriter::writeComment(const std::string& comment){
  m_file << m_leadingWhiteSpaces << "<!-- " << comment << " -->" << std::endl;
}

void SVGWriter::writeFooter(){
  decreaseIndentation();
  m_file << m_leadingWhiteSpaces << "</svg>";
}

void SVGWriter::writeHeader(const std::string& title){
  const Eigen::Vector2d& boundingBoxDiagonal = m_currentPolygonList->getBoundingBox().diagonal();
  double diagonalMin = boundingBoxDiagonal.minCoeff();
  Eigen::Vector2d downscaledBoundingBoxDiagonal = boundingBoxDiagonal / diagonalMin;
  m_scalingVector = downscaledBoundingBoxDiagonal * m_scalingFactor;
  double aspectRatio = m_scalingVector.x() / m_scalingVector.y();
  Eigen::Vector2d imageCoordinates = -m_scalingVector * m_borderFactor;
  Eigen::Vector2d imageMeasures = m_scalingVector * (1. + 2. * m_borderFactor);
  m_file << m_leadingWhiteSpaces << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
  m_file << m_leadingWhiteSpaces << "<svg xmlns=\"http://www.w3.org/2000/svg\"\n";
  m_file << m_leadingWhiteSpaces << "xmlns:xlink=\"http://www.w3.org/1999/xlink\"\n";
  m_file << m_leadingWhiteSpaces << "version=\"1.1\" baseProfile=\"full\"\n";
  m_file << m_leadingWhiteSpaces << "width=\"" << m_height * aspectRatio << "mm\" height=\"" << m_height << "mm\"\n";
  m_file << m_leadingWhiteSpaces << "viewBox=\"" << imageCoordinates.x() << " " << imageCoordinates.y() << " " << imageMeasures.x() << " " << imageMeasures.y() << "\">\n";
  increaseIndentation();
  m_file << m_leadingWhiteSpaces << "<title>" << title << "</title>\n";
  m_file << m_leadingWhiteSpaces << "<desc>Unfolding of " << title << ".</desc>\n\n";
  m_file << m_leadingWhiteSpaces << "<g fill=\"#ffffff\" fill-opacity=\"1\" font-family=\"Sans Serif\">\n";
  increaseIndentation();
  m_file << m_leadingWhiteSpaces << "<rect x=\"" << imageCoordinates.x()
                                 << "\" y=\"" << imageCoordinates.y()
                                 << "\" width=\"" << imageMeasures.x() - imageCoordinates.x()
                                 << "\" height=\"" << imageMeasures.y() - imageCoordinates.y() << "\"/>\n";
  decreaseIndentation();
  m_file << m_leadingWhiteSpaces << "</g>\n\n";
}
