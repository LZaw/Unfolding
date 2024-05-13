
#pragma once

// Project Headers
#include <Datastructures/Mesh/Mesh.hpp>
#include <Transformer/CCF/CCFTransformer.hpp>

class CCFLaplaceTransformer : public CCFTransformer{
public:
  CCFLaplaceTransformer() = delete;
  CCFLaplaceTransformer(UnfoldMesh* const m);

protected:
  double calculateEnergy() override;

  void filter() override;

  void updateFilterSolver() override;
};
