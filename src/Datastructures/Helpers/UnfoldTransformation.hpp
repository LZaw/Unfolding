
#pragma once

// Eigen Headers
#include <Eigen/Dense>

struct UnfoldTransformation{
  UnfoldTransformation(int source, int target):
    UnfoldTransformation(source, target, -1, -1){
  }
  UnfoldTransformation(int source, int target, int globalV0, int globalV1){
    m_globalV0 = globalV0;
    m_globalV1 = globalV1;
    m_sourceID = source;
    m_targetID = target;
  }
  UnfoldTransformation(const UnfoldTransformation& other){
    m_globalV0 = other.m_globalV0;
    m_globalV1 = other.m_globalV1;
    rotation = other.rotation;
    m_sourceID = other.m_sourceID;
    m_targetID = other.m_targetID;
    translation = other.translation;
  }
  UnfoldTransformation() = default;

  int m_globalV0 = -1;
  int m_globalV1 = -1;

  // Rotates SourceID to TargetID
  Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();

  // SouceID is unfolded to align with TargetID
  int m_sourceID = -1;

  int m_targetID = -1;
  // Moving (0, 0, 0) to v0
  Eigen::Translation3d translation = Eigen::Translation3d::Identity();
};

