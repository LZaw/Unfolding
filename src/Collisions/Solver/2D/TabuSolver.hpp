
#pragma once

// C++ Headers
#include <utility>
#include <vector>

// Project Headers
#include <Collisions/Detector/CollisionDetector.hpp>
#include <Collisions/Solver/2D/Abstracts/Constraints/NoConstraintsBase.hpp>
#include <Collisions/Solver/2D/Abstracts/FaceSelector/RandomMovableCollidingFaceSelectorBase.hpp>
#include <Datastructures/Mesh/Mesh.hpp>
#include <Datastructures/Polygon/PolygonList.hpp>
#include <Datastructures/Unfolding/UnfoldTree.hpp>
#include <Model/Model2D.hpp>
#include <Unfolding/Unfolder.hpp>

// C++ Headers
#include <deque>

class TabuSolver : public NoConstraintsBase,
                   public RandomMovableCollidingFaceSelectorBase{
public:
  TabuSolver(Model2D* model2D, Unfolder* unfolder);

  void resetLocalVariables() override;

  bool takeStep(UnfoldTree*& currentTreeNode) override;

protected:
  bool determineAndResolvePossibleLocalMinima() override;

private:
  inline bool acceptMove(const unsigned int newNumberOfCollisions) const;

  inline void climb(UnfoldTree*& currentTreeNode);

  inline void performMove(UnfoldTree* currentTreeNode, UnfoldTree* newParent);

  inline void removeForbiddenEntry(const unsigned int nodeIndex, const unsigned int forbiddenIndex);

  inline std::pair<UnfoldTree*, unsigned int> selectBestNeighbor(UnfoldTree* currentTreeNode, const std::vector<unsigned int>& possibleNeighbors);
  inline std::vector<unsigned int> setUpPossibleNeighbors(UnfoldTree* currentTreeNode);

  std::deque<
    std::pair<
      unsigned int, unsigned int>>    m_forbiddenMovesDeque;
  std::vector<
    std::vector<
      unsigned int>>                  m_forbiddenMovesVector;
  unsigned int                        m_maxMemoryMoves;
  unsigned int                        m_memoryResetCondition;
  Model2D*                            m_model2D;
  unsigned int                        m_reinitCondition;
};
