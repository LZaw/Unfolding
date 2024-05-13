
#pragma once

// C++ Headers
#include <memory>

// Project Headers
#include <Datastructures/Mesh/Mesh.hpp>

class UnfoldMesh;
class OriginalMesh: public Mesh{
public:
  OriginalMesh();

  void drop();

  void simplify(UnfoldMesh& targetMesh, const unsigned int numFaces);

protected:
  void postReadInitialization() override;
};
