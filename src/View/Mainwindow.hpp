
#pragma once

// C++ Headers
#include <memory>

// Project Headers
#include <Model/UnfoldModel.hpp>
#include <View/Widget2D.hpp>
#include <View/Widget3D.hpp>

// QT Headers
#include <QFileDialog>
#include <QMainWindow>
#include <QWidget>

class MainWindow : public QMainWindow{
  Q_OBJECT
public:
  explicit MainWindow(QWidget* parent = nullptr);

private:
  Eigen::MatrixXd m_eigenVectors;
  QWidget*        m_mainWidget;
  std::unique_ptr<
    UnfoldModel>  m_model;
  Widget3D*       m_openGLWidget;
  Widget2D*       m_twoDWidget;

  void computeDrawColors();
  bool computeEigenDecomposition();

  void emitToggledActions(QList<QAction*>& actions);
  void extractColors(const int eigenColumn);

  void initLayout();
  void initMenu();

  QString setupFileDialog(const QString& title, const QString& filter, const QFileDialog::AcceptMode mode);

public slots:
  void on_CollisionSolveAction2D_triggered();
  void on_CollisionSolveActionGeometric_triggered();
  void on_ColorSelectionAction_triggered();
  void on_ComputeDrawColors_triggered();
  void on_CreateNewUnfoldAct_triggered();

  void on_DropAct_triggered();
  void on_DropUnfoldAct_triggered();

  void on_FullScreenAction_triggered();

  void on_LoadOriginalMeshAction_triggered();
  void on_LoadFoldedMeshAction_triggered();
  void on_LoadUnfoldTreeAction_triggered();

  void on_RunDebugAct_triggered();

  void on_SaveColoredUnfolding_triggered();
  void on_SaveOriginalMeshAction_triggered();
  void on_SaveFoldedMeshAction_triggered();
  void on_SaveFoldingInstructionsAction_triggered();
  void on_SaveUnfoldingAction_triggered();
  void on_SaveUnfoldTreeAction_triggered();
  void on_SimplifyAct_triggered();

  void repaintAllWidgets();
};
