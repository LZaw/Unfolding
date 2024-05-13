
#pragma once

// C++ Headers
#include <vector>

// Project Headers
#include <Datastructures/Helpers/UnfoldTransformation.hpp>

// Eigen Headers

class DualGraphNode{
public:
  int getRef() const;
  const UnfoldTransformation& getUnfoldTransformation() const;
  UnfoldTransformation& getUnfoldTransformation();

  static const DualGraphNode& invalidNode();

  void setRef(int ref);

  void updateUnfoldTransformation(const UnfoldTransformation& transformation);
  void updateUnfoldTransformation(UnfoldTransformation&& transformation);

private:
  int m_ref = -1;
  static DualGraphNode s_invalidNode;
  UnfoldTransformation m_transformation;
};
