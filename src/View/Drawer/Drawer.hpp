
#pragma once

// Eigen Headers
#include <Eigen/Dense>

struct MeshVertexData{
  Eigen::Vector3f normal;
  Eigen::Vector3f position;
};

struct MeshVertexColorData{
  Eigen::Vector3f normal;
  Eigen::Vector3f position;
  Eigen::Vector3f color;
};

class Drawer{
public:
  Drawer():
    m_screenshotLines(false){
  }

  void toggleScreenshotLines(bool toggle){
    m_screenshotLines = toggle;
  }

protected:
  bool              m_screenshotLines;
};
