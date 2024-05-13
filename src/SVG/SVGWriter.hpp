
#pragma once

// C++ Headers
#include <fstream>
#include <string>

// Project Headers
#include <Datastructures/Polygon/PolygonList.hpp>

class SVGWriter{
public:
  SVGWriter();

  void savePolygonsToSVG(const std::string& path, const PolygonList& polygonList);

  void saveUnfoldingToSVG(const std::string& path, const PolygonList& polygonList, bool withFaceIndices);
  void saveUnfoldTreeToSVG(const std::string& path, const PolygonList& polygonList);

private:
  enum class WriteMode{
    Indices,
    CutEdges,
    MountainEdges,
    ValleyEdges,
    Polygons,
    UnfoldTreeEdges
  };

  double              m_borderFactor;
  const PolygonList*  m_currentPolygonList = nullptr;
  std::fstream        m_file;
  int                 m_height; // mm
  std::string         m_leadingWhiteSpaces;
  double              m_scalingFactor;
  Eigen::Vector2d     m_scalingVector;

  void decreaseIndentation();
  void drawLine(const Eigen::Vector2d& v0, const Eigen::Vector2d& v1);
  void drawEdges();
  void drawPolygon(const Polygon2D& currentPolygon);
  void drawPolygonIndex(const Polygon2D& currentPolygon);
  void drawUnfoldTree();

  void finishFile();

  void increaseIndentation();

  std::string rgb2hex(const Eigen::Vector3f& rgb) const;

  void setupFile(const std::string& path);
  void setWriteMode(WriteMode mode);

  Eigen::Vector2d transformToGlobalCoordinates(const Eigen::Vector2d& localCoordinate) const;

  void unsetWriteMode();

  void writeComment(const std::string& comment);
  void writeFooter();
  void writeHeader(const std::string& title);
};
