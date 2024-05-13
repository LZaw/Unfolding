
#pragma once

// Project Headers
#include <Collisions/Detector/CollisionDetector.hpp>
#include <Collisions/Solver/2D/Abstracts/Constraints/NoConstraintsBase.hpp>
#include <Collisions/Solver/2D/Abstracts/FaceSelector/RandomFaceSelectorBase.hpp>
#include <Collisions/Solver/2D/Abstracts/LocalMinimumResolver/NoResolverBase.hpp>
#include <Collisions/Solver/2D/Abstracts/Stepper/RandomParentStepperBase.hpp>
#include <Datastructures/Polygon/PolygonList.hpp>
#include <Datastructures/Unfolding/UnfoldTree.hpp>
#include <Datastructures/Unfolding/UnfoldTreeList.hpp>

class RandomSolver: public NoConstraintsBase,
                    public NoResolverBase,
                    public RandomFaceSelectorBase,
                    public RandomParentStepperBase{
public:
  RandomSolver(UnfoldTreeList* unfoldTreeList, PolygonList* polygonList, CollisionDetector* collisionDetector);
};
