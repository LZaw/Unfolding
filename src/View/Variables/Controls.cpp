
#include "Controls.hpp"

namespace Controls{
  namespace Keyboard{
    namespace Sequences{
      const QKeySequence colorSelect(Qt::Key::Key_No);
      const QKeySequence computeDrawColors(Qt::Key::Key_No);
      const QKeySequence createNewUnfolding(Qt::Key::Key_No);

      const QKeySequence dropFoldedMesh(Qt::Key::Key_No);
      const QKeySequence dropOriginalMesh(Qt::Key::Key_No);

      const QKeySequence loadFoldedMesh(Qt::CTRL + Qt::SHIFT + Qt::Key_O);
      const QKeySequence loadOriginalMesh(Qt::CTRL + Qt::Key_O);
      const QKeySequence loadUnfoldTree(Qt::CTRL + Qt::SHIFT + Qt::ALT + Qt::Key_O);

      const QKeySequence quit(Qt::CTRL + Qt::Key_Q);

      const QKeySequence runDebug(Qt::Key::Key_No);

      const QKeySequence saveColoredUnfolding(Qt::Key::Key_No);
      const QKeySequence saveFoldedMesh(Qt::CTRL + Qt::Key_S);
      const QKeySequence saveFoldingInstructions(Qt::Key::Key_No);
      const QKeySequence saveOriginalMesh(Qt::Key::Key_No);
      const QKeySequence saveUnfoldTree(Qt::CTRL + Qt::SHIFT + Qt::ALT + Qt::Key_S);
      const QKeySequence saveUnfolding(Qt::CTRL + Qt::SHIFT + Qt::Key_S);
      const QKeySequence showConfigMenu(Qt::Key::Key_No);
      const QKeySequence simplifyOriginalMesh(Qt::Key::Key_No);
      const QKeySequence solveCollisions2D(Qt::Key::Key_No);
      const QKeySequence solveCollisionsGeometric(Qt::Key::Key_No);

      const QKeySequence toggleFullScreen(Qt::Key::Key_No);
      const QKeySequence toggleHighlightRoot(Qt::Key::Key_No);
      const QKeySequence toggleScreenshotLineThickness(Qt::Key::Key_No);
      const QKeySequence toggleShowColorGradient(Qt::Key::Key_No);
      const QKeySequence toggleShowCutTree(Qt::Key::Key_No);
      const QKeySequence toggleShowDebug(Qt::Key::Key_No);
      const QKeySequence toggleShowFoldedMesh(Qt::Key::Key_No);
      const QKeySequence toggleShowOriginalMesh(Qt::Key::Key_No);
      const QKeySequence toggleShowUnfoldTree(Qt::Key::Key_No);
    }
    const Qt::KeyboardModifier previewKeyModifier(Qt::ShiftModifier);

    const Qt::Key chooseNextPossiblePolygonKey(Qt::Key_Right);
    const Qt::Key choosePriorPossiblePolygonKey(Qt::Key_Left);

    const Qt::Key moveVertexKey(Qt::Key_Control);

    const Qt::Key previewKey(Qt::Key_Shift);

    const Qt::Key rerootKey(Qt::Key_R);

    const Qt::Key selectParentKey(Qt::Key_Up);

    bool isPreviewKeyPressed(){
      return (QApplication::keyboardModifiers() & previewKeyModifier) != 0;
    }
  }
  namespace Mouse{
    const Qt::MouseButton selectButton(Qt::LeftButton);

    const Qt::MouseButton moveViewButton(Qt::RightButton);

    bool isSelectButtonPressed(){
      return (QApplication::mouseButtons() & selectButton) != 0;
    }
    bool isMoveViewButtonPressed(){
      return (QApplication::mouseButtons() & moveViewButton) != 0;

    }
  }
}
