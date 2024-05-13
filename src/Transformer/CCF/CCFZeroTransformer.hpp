
#pragma once

// Project Headers
#include <Datastructures/Mesh/Mesh.hpp>
#include <Transformer/CCF/CCFTransformer.hpp>

class CCFZeroTransformer : public CCFTransformer{
public:
  CCFZeroTransformer() = delete;
  CCFZeroTransformer(UnfoldMesh* const m);

protected:
  double calculateEnergy() override;

  void filter() override;

  void updateFilterSolver() override;
};
