
#pragma once

// Project Headers
#include <Datastructures/Mesh/Mesh.hpp>
#include <Transformer/CCF/CCFTransformer.hpp>

class CCFWillmoreTransformer : public CCFTransformer{
public:
  CCFWillmoreTransformer() = delete;
  CCFWillmoreTransformer(UnfoldMesh* const m);

protected:
  double calculateEnergy() override;

  void filter() override;

  void updateFilterSolver() override;
};
