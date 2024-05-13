
#--------------------------------------------------------------------------------------------------
# Source files.
#--------------------------------------------------------------------------------------------------

# Collisions
set(collision_sources
  Collisions/Detector/CollisionDetector.cpp
  Collisions/Detector/CollisionDetector.hpp
  Collisions/Solver/2D/Abstracts/Constraints/NoConstraintsBase.hpp
  Collisions/Solver/2D/Abstracts/FaceSelector/MostCollidingFaceSelectorBase.hpp
  Collisions/Solver/2D/Abstracts/FaceSelector/RandomCollidingFaceSelectorBase.hpp
  Collisions/Solver/2D/Abstracts/FaceSelector/RandomFaceSelectorBase.hpp
  Collisions/Solver/2D/Abstracts/FaceSelector/RandomMovableCollidingFaceSelectorBase.hpp
  Collisions/Solver/2D/Abstracts/LocalMinimumResolver/NoResolverBase.hpp
  Collisions/Solver/2D/Abstracts/Stepper/RandomParentStepperBase.hpp
  Collisions/Solver/2D/Abstracts/Stepper/SelectParentRateClimbHistoryStepperBase.hpp
  Collisions/Solver/2D/CollisionSolver2DBase.cpp
  Collisions/Solver/2D/CollisionSolver2DBase.hpp
  Collisions/Solver/2D/PredictingRandomCollisionSolver.cpp
  Collisions/Solver/2D/PredictingRandomCollisionSolver.hpp
  Collisions/Solver/2D/RandomCollisionSolver.cpp
  Collisions/Solver/2D/RandomCollisionSolver.hpp
  Collisions/Solver/2D/RandomSolver.cpp
  Collisions/Solver/2D/RandomSolver.hpp
  Collisions/Solver/2D/TabuSolver.cpp
  Collisions/Solver/2D/TabuSolver.hpp
  Collisions/Solver/Holistic/Geometric/CollisionSolverHolisticGeometric.cpp
  Collisions/Solver/Holistic/Geometric/CollisionSolverHolisticGeometric.hpp
  Collisions/Solver/Holistic/CollisionSolverHolisticBase.cpp
  Collisions/Solver/Holistic/CollisionSolverHolisticBase.hpp
  Collisions/Solver/CollisionSolverBase.cpp
  Collisions/Solver/CollisionSolverBase.hpp
)

list(APPEND unfolding_sources
  ${collision_sources}
)

# Datastructures
set(datastructure_sources
  Datastructures/Helpers/UnfoldTransformation.hpp
  Datastructures/Matrices/LaplaceMatrix.cpp
  Datastructures/Matrices/LaplaceMatrix.hpp
  Datastructures/Matrices/MassMatrix.cpp
  Datastructures/Matrices/MassMatrix.hpp
  Datastructures/Mesh/DualGraph.cpp
  Datastructures/Mesh/DualGraph.hpp
  Datastructures/Mesh/DualGraphNode.cpp
  Datastructures/Mesh/DualGraphNode.hpp
  Datastructures/Mesh/FaceList.cpp
  Datastructures/Mesh/FaceList.hpp
  Datastructures/Mesh/Mesh.cpp
  Datastructures/Mesh/Mesh.hpp
  Datastructures/Mesh/MeshIO.cpp
  Datastructures/Mesh/OriginalMesh.cpp
  Datastructures/Mesh/OriginalMesh.hpp
  Datastructures/Mesh/UnfoldMesh.cpp
  Datastructures/Mesh/UnfoldMesh.hpp
  Datastructures/Polygon/Polygon2D.cpp
  Datastructures/Polygon/Polygon2D.hpp
  Datastructures/Polygon/PolygonList.cpp
  Datastructures/Polygon/PolygonList.hpp
  Datastructures/Unfolding/UnfoldTree.cpp
  Datastructures/Unfolding/UnfoldTree.hpp
  Datastructures/Unfolding/UnfoldTreeList.cpp
  Datastructures/Unfolding/UnfoldTreeList.hpp
  Datastructures/Unfolding/UnfoldTreeListIO.cpp
)

list(APPEND unfolding_sources
  ${datastructure_sources}
)

# Model
set(model_sources
  Model/Model2D.cpp
  Model/Model2D.hpp
  Model/Model3D.cpp
  Model/Model3D.hpp
  Model/UnfoldModel.cpp
  Model/UnfoldModel.hpp
)

list(APPEND unfolding_sources
  ${model_sources}
)

# SVG
set(svg_sources
  SVG/SVGWriter.cpp
  SVG/SVGWriter.hpp
)

list(APPEND unfolding_sources
  ${svg_sources}
)

# Transformer
set(transformer_sources
  Transformer/CCF/CCFLaplaceTransformer.cpp
  Transformer/CCF/CCFLaplaceTransformer.hpp
  Transformer/CCF/CCFTransformer.cpp
  Transformer/CCF/CCFTransformer.hpp
  Transformer/CCF/CCFWillmoreTransformer.cpp
  Transformer/CCF/CCFWillmoreTransformer.hpp
  Transformer/CCF/CCFZeroTransformer.cpp
  Transformer/CCF/CCFZeroTransformer.hpp
  Transformer/CCF/DerivedQuaterniond.cpp
  Transformer/CCF/DerivedQuaterniond.hpp
  Transformer/AngleEqualizeTransformer.cpp
  Transformer/AngleEqualizeTransformer.hpp
  Transformer/ConformalizedMeanCurvatureTransformer.cpp
  Transformer/ConformalizedMeanCurvatureTransformer.hpp
  Transformer/EdgeNormalAlignTransformer.cpp
  Transformer/EdgeNormalAlignTransformer.hpp
  Transformer/GenericTransformer.cpp
  Transformer/GenericTransformer.hpp
  Transformer/MassSpringTransformer.cpp
  Transformer/MassSpringTransformer.hpp
)

list(APPEND unfolding_sources
  ${transformer_sources}
)

# Unfolding
set(subunfolding_sources
  Unfolding/BFSUnfolder.cpp
  Unfolding/BFSUnfolder.hpp
  Unfolding/DFSUnfolder.cpp
  Unfolding/DFSUnfolder.hpp
  Unfolding/MSTDiffUnfolder.cpp
  Unfolding/MSTDiffUnfolder.hpp
  Unfolding/MSTQuotientUnfolder.cpp
  Unfolding/MSTQuotientUnfolder.hpp
  Unfolding/MSTUnfolder.cpp
  Unfolding/MSTUnfolder.hpp
  Unfolding/SteepestEdgeUnfolder.cpp
  Unfolding/SteepestEdgeUnfolder.hpp
  Unfolding/Unfolder.cpp
  Unfolding/Unfolder.hpp
  Unfolding/WeightedSteepestEdgeUnfolder.cpp
  Unfolding/WeightedSteepestEdgeUnfolder.hpp
)

list(APPEND unfolding_sources
  ${subunfolding_sources}
)

# Util
set(util_sources
  Util/DDGUtil.cpp
  Util/DDGUtil.hpp
  Util/FileUtil.cpp
  Util/FileUtil.hpp
  Util/PolygonUtil.cpp
  Util/PolygonUtil.hpp
)

list(APPEND unfolding_sources
  ${util_sources}
)

if(WITH_FRONTEND)
  # View
  set(view_sources
    View/Dialogs/CollisionSolverProgressDialog.cpp
    View/Dialogs/CollisionSolverProgressDialog.hpp
    View/Dialogs/CollisionSolverSelectDialog.cpp
    View/Dialogs/CollisionSolverSelectDialog.hpp
    View/Drawer/CutTreeDrawer.cpp
    View/Drawer/CutTreeDrawer.hpp
    View/Drawer/DebugDrawer.cpp
    View/Drawer/DebugDrawer.hpp
    View/Drawer/Drawer.hpp
    View/Drawer/OriginalMeshDrawer.cpp
    View/Drawer/OriginalMeshDrawer.hpp
    View/Drawer/UnfoldMeshDrawer.cpp
    View/Drawer/UnfoldMeshDrawer.hpp
    View/Drawer/UnfoldTreeDrawer.cpp
    View/Drawer/UnfoldTreeDrawer.hpp
    View/Variables/Colorpalette.cpp
    View/Variables/Colorpalette.hpp
    View/Variables/Controls.cpp
    View/Variables/Controls.hpp
    View/Mainwindow.cpp
    View/Mainwindow.hpp
    View/Widget2D.cpp
    View/Widget2D.hpp
    View/Widget3D.cpp
    View/Widget3D.hpp
  )

  list(APPEND unfolding_sources
    ${view_sources}
  )
endif()

if(WITH_FRONTEND)
  list(APPEND unfolding_sources
    main-Frontend.cpp
  )
else()
  list(APPEND unfolding_sources
    main-cli.cpp
  )
endif()

#--------------------------------------------------------------------------------------------------
# Output.
#--------------------------------------------------------------------------------------------------

# Set executable
add_executable(Unfolding ${unfolding_sources})

# Make it appear in the base build folder
set_target_properties(Unfolding PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
