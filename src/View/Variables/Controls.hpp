
#pragma once

// QT Headers
#include <QApplication>
#include <QKeySequence>
#include <Qt>

namespace Controls{
  namespace Keyboard{
    namespace Sequences{
      extern const QKeySequence colorSelect;
      extern const QKeySequence computeDrawColors;
      extern const QKeySequence createNewUnfolding;

      extern const QKeySequence dropFoldedMesh;
      extern const QKeySequence dropOriginalMesh;

      extern const QKeySequence loadFoldedMesh;
      extern const QKeySequence loadOriginalMesh;
      extern const QKeySequence loadUnfoldTree;

      extern const QKeySequence quit;

      extern const QKeySequence runDebug;

      extern const QKeySequence saveColoredUnfolding;
      extern const QKeySequence saveFoldedMesh;
      extern const QKeySequence saveFoldingInstructions;
      extern const QKeySequence saveOriginalMesh;
      extern const QKeySequence saveUnfoldTree;
      extern const QKeySequence saveUnfolding;
      extern const QKeySequence showConfigMenu;
      extern const QKeySequence simplifyOriginalMesh;
      extern const QKeySequence solveCollisions2D;
      extern const QKeySequence solveCollisionsGeometric;

      extern const QKeySequence toggleFullScreen;
      extern const QKeySequence toggleHighlightRoot;
      extern const QKeySequence toggleScreenshotLineThickness;
      extern const QKeySequence toggleShowColorGradient;
      extern const QKeySequence toggleShowCutTree;
      extern const QKeySequence toggleShowDebug;
      extern const QKeySequence toggleShowFoldedMesh;
      extern const QKeySequence toggleShowOriginalMesh;
      extern const QKeySequence toggleShowUnfoldTree;
    }
    extern const Qt::KeyboardModifier previewKeyModifier;

    extern const Qt::Key chooseNextPossiblePolygonKey;
    extern const Qt::Key choosePriorPossiblePolygonKey;

    extern const Qt::Key moveVertexKey;

    extern const Qt::Key previewKey;

    extern const Qt::Key rerootKey;

    extern const Qt::Key selectParentKey;

    bool isPreviewKeyPressed();
  }
  namespace Mouse{
    extern const Qt::MouseButton selectButton;

    extern const Qt::MouseButton moveViewButton;

    bool isSelectButtonPressed();
    bool isMoveViewButtonPressed();
  }
}
