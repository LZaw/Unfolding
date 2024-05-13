
#include "Mainwindow.hpp"

// C++ Headers
#include <future>
#include <limits>
#include <thread>
#include <tuple>
#include <utility>

// Project Headers
#include <Datastructures/Matrices/LaplaceMatrix.hpp>
#include <Datastructures/Matrices/MassMatrix.hpp>
#include <Datastructures/Mesh/FaceList.hpp>
#include <Datastructures/Mesh/Mesh.hpp>
#include <Util/DDGUtil.hpp>
#include <View/Dialogs/CollisionSolverProgressDialog.hpp>
#include <View/Dialogs/CollisionSolverSelectDialog.hpp>
#include <View/Variables/Colorpalette.hpp>
#include <View/Variables/Controls.hpp>

// Eigen Headers
#include <Eigen/Dense>
#include <Eigen/Sparse>

// Spectra Headers
#include <Spectra/MatOp/SparseSymShiftSolve.h>
#include <Spectra/SymEigsShiftSolver.h>

// QT Headers
#include <QAction>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMenu>
#include <QMenuBar>
#include <QProgressDialog>
#include <QPushButton>
#include <QRadioButton>
#include <QThread>
#include <QTimeEdit>

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  m_mainWidget(new QWidget()),
  m_model(new UnfoldModel()){
  m_openGLWidget = new Widget3D(m_model.get());
  QSurfaceFormat f = QSurfaceFormat::defaultFormat();
  m_openGLWidget->setFormat(f);
  m_twoDWidget = new Widget2D(m_model.get());

  initMenu();
  initLayout();

  // Connect Widgets
  connect(m_openGLWidget, SIGNAL(requestUpdate2D()), m_twoDWidget, SLOT(update()));
  connect(m_twoDWidget, SIGNAL(requestUpdate3D()), m_openGLWidget, SLOT(update()));
  connect(m_twoDWidget, SIGNAL(updateUnfoldTree(const std::pair<int, int>&, const std::pair<int, int>&)), m_openGLWidget, SLOT(updateUnfoldTree(const std::pair<int, int>&, const std::pair<int, int>&)));

  setWindowTitle(QStringLiteral("Unfolding"));
  setCentralWidget(m_mainWidget);
  show();
  showMaximized();
}

// Private

void MainWindow::computeDrawColors(){
  if(m_model->getModel3D()->getUnfoldMesh() == nullptr){
    return;
  }
  if(!computeEigenDecomposition()){
    return;
  }
  extractColors(m_eigenVectors.cols() - 2);
}

bool MainWindow::computeEigenDecomposition(){
  const Mesh* const mesh = m_model->getModel3D()->getUnfoldMesh();
  // Setup Matrices
  LaplaceMatrix L;
  L.setup(mesh->getVertices(), mesh->getFaces(), LaplaceMatrix::LaplaceMatrixType::COT);
  MassMatrix M;
  M.setup(mesh->getVertices(), mesh->getFaces(), MassMatrix::MassMatrixType::MIXEDVORONOI);
  Eigen::SparseMatrix<double> Mrsqrt(M.rows(), M.cols());
  Mrsqrt.setIdentity();
  Mrsqrt.diagonal() = M.diagonal().array().rsqrt();
  const Eigen::SparseMatrix<double> LB = Mrsqrt * -L * Mrsqrt;

  // Compute Eigenvectors
  const int numEigenVectors = 6;
  const double shift = -std::numeric_limits<float>::epsilon();
  const int ncv = std::min(2 * numEigenVectors, static_cast<int>(LB.rows()));
  Spectra::SparseSymShiftSolve<double> op(LB);
  try{
    Spectra::SymEigsShiftSolver<Spectra::SparseSymShiftSolve<double>> eigs(op, numEigenVectors, ncv, shift);
    eigs.init();
    eigs.compute(Spectra::SortRule::LargestMagn);
    if(eigs.info() != Spectra::CompInfo::Successful){
      return false;
    }
    m_eigenVectors = eigs.eigenvectors();
  }catch(...){
    return false;
  }
  return true;
}

void MainWindow::emitToggledActions(QList<QAction*>& actions){
  for(int i = 0; i < actions.size(); ++i){
    if(actions[i]->menu()){
      QList<QAction*> subMenu = actions[i]->menu()->actions();
      emitToggledActions(subMenu);
    }else{
      if(actions[i]->isCheckable() && actions[i]->isChecked()){
        emit(actions[i]->toggled(true));
      }
      if(actions[i]->isCheckable() && !actions[i]->isChecked()){
        emit(actions[i]->toggled(false));
      }
    }
  }
}

void MainWindow::extractColors(const int eigenColumn){
  const Mesh* const mesh = m_model->getModel3D()->getUnfoldMesh();
  if(mesh == nullptr){
    return;
  }
  Eigen::VectorXd eigenVector = m_eigenVectors.col(eigenColumn);
  double minCoeff = eigenVector.minCoeff();
  eigenVector.array() -= minCoeff;
  double maxCoeff = eigenVector.maxCoeff();
  eigenVector /= maxCoeff;

  // Set Colors
  const Eigen::Vector3f minColor( 64., 128., 255.);
  const Eigen::Vector3f midColor(255.,  16.,  16.);
  const Eigen::Vector3f maxColor(255., 223.,  64.);
  Eigen::MatrixXf colors(mesh->getFaces().size(), 3);
  colors.setZero();
  for(FaceList::const_iterator faceIterator = mesh->getFaces().cbegin(); faceIterator != mesh->getFaces().cend(); ++faceIterator){
    unsigned int i = std::distance(mesh->getFaces().cbegin(), faceIterator);
    for(std::vector<unsigned int>::const_iterator vertexIndexIt = faceIterator->cbegin(); vertexIndexIt != faceIterator->cend(); ++vertexIndexIt){
      const double vertexValue = eigenVector[*vertexIndexIt];
      if(vertexValue < 0.5){
        colors.row(i) += vertexValue * midColor;
        colors.row(i) += (1. - vertexValue) * minColor;
      }else{
        colors.row(i) += vertexValue * maxColor;
        colors.row(i) += (1. - vertexValue) * midColor;
      }
    }
    colors.row(i) /= mesh->getFaces()[i].size();
  }
  m_openGLWidget->setColors(colors);
  m_openGLWidget->updateUnfoldMeshVertices();
  m_model->setFaceColors(colors);
}

void MainWindow::initLayout(){
  QHBoxLayout *mainLayout(new QHBoxLayout(m_mainWidget));
  QPalette pal = m_mainWidget->palette();

  pal.setColor(QPalette::Window, Colorpalette::windowBackgroundColor);
  m_mainWidget->setPalette(pal);
  pal = m_openGLWidget->palette();

  pal.setColor(QPalette::Window, Colorpalette::widgetBackgroundColor);
  m_openGLWidget->setPalette(pal);
  m_twoDWidget->setPalette(pal);
  m_twoDWidget->setMouseTracking(true);
  m_twoDWidget->setAutoFillBackground(true);
  m_mainWidget->setAutoFillBackground(true);
  m_openGLWidget->setAutoFillBackground(true);

  m_openGLWidget->setFocusPolicy(Qt::StrongFocus);
  m_twoDWidget->setFocusPolicy(Qt::StrongFocus);

  mainLayout->setContentsMargins(3, 3, 3, 3);
  mainLayout->addWidget(m_openGLWidget);
  mainLayout->addSpacing(-3);
  mainLayout->addWidget(m_twoDWidget);
  m_mainWidget->setLayout(mainLayout);
}

void MainWindow::initMenu(){
  // File Menu
  QAction* loadOriginalMeshAct(new QAction(tr("&Original Mesh"), this));
  loadOriginalMeshAct->setShortcut(Controls::Keyboard::Sequences::loadOriginalMesh);
  connect(loadOriginalMeshAct, SIGNAL(triggered()), this, SLOT(on_LoadOriginalMeshAction_triggered()));

  QAction* loadFoledMeshdAct(new QAction(tr("&Folded Mesh"), this));
  loadFoledMeshdAct->setShortcut(Controls::Keyboard::Sequences::loadFoldedMesh);
  connect(loadFoledMeshdAct, SIGNAL(triggered()), this, SLOT(on_LoadFoldedMeshAction_triggered()));

  QAction* loadUnfoldTreeAct(new QAction(tr("&Unfoldtree"), this));
  loadUnfoldTreeAct->setShortcut(Controls::Keyboard::Sequences::loadUnfoldTree);
  connect(loadUnfoldTreeAct, SIGNAL(triggered()), this, SLOT(on_LoadUnfoldTreeAction_triggered()));

  QMenu* loadMenu(new QMenu(tr("&Load"), this));
  loadMenu->addAction(loadOriginalMeshAct);
  loadMenu->addAction(loadFoledMeshdAct);
  loadMenu->addAction(loadUnfoldTreeAct);

  QAction* saveOriginalMeshAct(new QAction(tr("&Original Mesh"), this));
  saveOriginalMeshAct->setShortcut(Controls::Keyboard::Sequences::saveOriginalMesh);
  connect(saveOriginalMeshAct, SIGNAL(triggered()), this, SLOT(on_SaveOriginalMeshAction_triggered()));

  QAction* saveFoldedMeshAct(new QAction(tr("&Folded Mesh"), this));
  saveFoldedMeshAct->setShortcut(Controls::Keyboard::Sequences::saveFoldedMesh);
  connect(saveFoldedMeshAct, SIGNAL(triggered()), this, SLOT(on_SaveFoldedMeshAction_triggered()));

  QAction* saveUnfoldtreeAct(new QAction(tr("&Unfoldtree"), this));
  saveUnfoldtreeAct->setShortcut(Controls::Keyboard::Sequences::saveUnfoldTree);
  connect(saveUnfoldtreeAct, SIGNAL(triggered()), this, SLOT(on_SaveUnfoldTreeAction_triggered()));

  QAction* saveUnfoldingAct(new QAction(tr("&Unfolding"), this));
  saveUnfoldingAct->setShortcut(Controls::Keyboard::Sequences::saveUnfolding);
  connect(saveUnfoldingAct, SIGNAL(triggered()), this, SLOT(on_SaveUnfoldingAction_triggered()));

  QAction* saveColoredUnfolding(new QAction(tr("&Colored Unfolding"), this));
  saveColoredUnfolding->setShortcut(Controls::Keyboard::Sequences::saveColoredUnfolding);
  connect(saveColoredUnfolding, SIGNAL(triggered()), this, SLOT(on_SaveColoredUnfolding_triggered()));

  QAction* saveFoldingInstructionsAct(new QAction(tr("&Folding Instructions"), this));
  saveFoldingInstructionsAct->setShortcut(Controls::Keyboard::Sequences::saveFoldingInstructions);
  connect(saveFoldingInstructionsAct, SIGNAL(triggered()), this, SLOT(on_SaveFoldingInstructionsAction_triggered()));

  QMenu* saveMenu(new QMenu(tr("&Save"), this));
  saveMenu->addAction(saveOriginalMeshAct);
  saveMenu->addAction(saveFoldedMeshAct);
  saveMenu->addAction(saveUnfoldtreeAct);
  saveMenu->addAction(saveUnfoldingAct);
  saveMenu->addAction(saveColoredUnfolding);
  saveMenu->addAction(saveFoldingInstructionsAct);

  QAction* quitAct(new QAction(tr("&Quit"), this));
  quitAct->setShortcut(Controls::Keyboard::Sequences::quit);
  connect(quitAct, SIGNAL(triggered()), this, SLOT(close()));

  QMenu* fileMenu = menuBar()->addMenu(tr("&File"));
  fileMenu->addMenu(loadMenu);
  fileMenu->addMenu(saveMenu);
  fileMenu->addAction(quitAct);

  // Edit Menu
  QAction* dropAct(new QAction(tr("&Drop Original Model"), this));
  dropAct->setShortcut(Controls::Keyboard::Sequences::dropOriginalMesh);
  connect(dropAct, SIGNAL(triggered()), this, SLOT(on_DropAct_triggered()));

  QAction* dropUnfoldAct(new QAction(tr("&Drop Folded Model"), this));
  dropUnfoldAct->setShortcut(Controls::Keyboard::Sequences::dropFoldedMesh);
  connect(dropUnfoldAct, SIGNAL(triggered()), this, SLOT(on_DropUnfoldAct_triggered()));

  QMenu* editMenu = menuBar()->addMenu(tr("&Edit"));
  editMenu->addAction(dropAct);
  editMenu->addAction(dropUnfoldAct);

  // View Menu
  QAction* showOriginalMeshAct(new QAction(tr("&Show Original Mesh"), this));
  showOriginalMeshAct->setCheckable(true);
  showOriginalMeshAct->setChecked(true);
  showOriginalMeshAct->setShortcut(Controls::Keyboard::Sequences::toggleShowOriginalMesh);
  connect(showOriginalMeshAct, SIGNAL(toggled(bool)), m_openGLWidget, SLOT(toggleDrawOriginalMesh(bool)));

  QAction* showFoldedMeshAct(new QAction(tr("&Show Folded Mesh"), this));
  showFoldedMeshAct->setCheckable(true);
  showFoldedMeshAct->setChecked(true);
  showFoldedMeshAct->setShortcut(Controls::Keyboard::Sequences::toggleShowFoldedMesh);
  connect(showFoldedMeshAct, SIGNAL(toggled(bool)), m_openGLWidget, SLOT(toggleDrawUnfoldMesh(bool)));

  QAction* showUnfoldTreeAct(new QAction(tr("&Show UnfoldTree"), this));
  showUnfoldTreeAct->setCheckable(true);
  showUnfoldTreeAct->setChecked(true);
  showUnfoldTreeAct->setShortcut(Controls::Keyboard::Sequences::toggleShowUnfoldTree);
  connect(showUnfoldTreeAct, SIGNAL(toggled(bool)), m_openGLWidget, SLOT(toggleDrawUnfoldTree(bool)));
  connect(showUnfoldTreeAct, SIGNAL(toggled(bool)), m_twoDWidget, SLOT(toggleDrawUnfoldTree(bool)));

  QAction* showCutTreeAct(new QAction(tr("&Show CutTree"), this));
  showCutTreeAct->setCheckable(true);
  showCutTreeAct->setChecked(false);
  showCutTreeAct->setShortcut(Controls::Keyboard::Sequences::toggleShowCutTree);
  connect(showCutTreeAct, SIGNAL(toggled(bool)), m_openGLWidget, SLOT(toggleDrawCutTree(bool)));

  QAction* showColorGradientAct(new QAction(tr("&Show ColorGradient"), this));
  showColorGradientAct->setCheckable(true);
  showColorGradientAct->setChecked(false);
  showColorGradientAct->setShortcut(Controls::Keyboard::Sequences::toggleShowColorGradient);
  connect(showColorGradientAct, SIGNAL(toggled(bool)), m_openGLWidget, SLOT(toggleDrawColorGradient(bool)));
  connect(showColorGradientAct, SIGNAL(toggled(bool)), m_twoDWidget, SLOT(toggleDrawColorGradient(bool)));

  QAction* showDebugAct(new QAction(tr("&Show Debug"), this));
  showDebugAct->setCheckable(true);
  showDebugAct->setChecked(false);
  showDebugAct->setShortcut(Controls::Keyboard::Sequences::toggleShowDebug);
  connect(showDebugAct, SIGNAL(toggled(bool)), m_openGLWidget, SLOT(toggleDrawDebugMesh(bool)));
  connect(showDebugAct, SIGNAL(toggled(bool)), m_twoDWidget, SLOT(toggleDrawDebug(bool)));

  QAction* highlightRootAct(new QAction(tr("&Highlight Root"), this));
  highlightRootAct->setCheckable(true);
  highlightRootAct->setChecked(true);
  highlightRootAct->setShortcut(Controls::Keyboard::Sequences::toggleHighlightRoot);
  connect(highlightRootAct, SIGNAL(toggled(bool)), m_openGLWidget, SLOT(toggleHighlightRoot(bool)));
  connect(highlightRootAct, SIGNAL(toggled(bool)), m_twoDWidget, SLOT(toggleHighlightRoot(bool)));

  QAction* toggleScreenshotLinesAct(new QAction(tr("&Toggle Screenshot Lines")));
  toggleScreenshotLinesAct->setCheckable(true);
  toggleScreenshotLinesAct->setChecked(false);
  toggleScreenshotLinesAct->setShortcut(Controls::Keyboard::Sequences::toggleScreenshotLineThickness);
  connect(toggleScreenshotLinesAct, SIGNAL(toggled(bool)), m_openGLWidget, SLOT(toggleScreenshotLines(bool)));
  connect(toggleScreenshotLinesAct, SIGNAL(toggled(bool)), m_twoDWidget, SLOT(toggleScreenshotLines(bool)));

  QMenu* viewMenu = menuBar()->addMenu(tr("&View"));
  viewMenu->addAction(showOriginalMeshAct);
  viewMenu->addAction(showFoldedMeshAct);
  viewMenu->addAction(showUnfoldTreeAct);
  viewMenu->addAction(showCutTreeAct);
  viewMenu->addAction(showColorGradientAct);
  viewMenu->addAction(showDebugAct);
  viewMenu->addAction(highlightRootAct);
  viewMenu->addAction(toggleScreenshotLinesAct);

  // Tools Menu
  QAction* simplifyAct(new QAction(tr("&Generate working mesh"), this));
  simplifyAct->setShortcut(Controls::Keyboard::Sequences::simplifyOriginalMesh);
  connect(simplifyAct, SIGNAL(triggered()), this, SLOT(on_SimplifyAct_triggered()));

  QAction* createNewUnfoldAct(new QAction(tr("&Create new unfolding"), this));
  createNewUnfoldAct->setShortcut(Controls::Keyboard::Sequences::createNewUnfolding);
  connect(createNewUnfoldAct, SIGNAL(triggered()), this, SLOT(on_CreateNewUnfoldAct_triggered()));

  QAction* collisionSolveAct2D(new QAction(tr("&Solve Collisions (2D)"), this));
  collisionSolveAct2D->setShortcut(Controls::Keyboard::Sequences::solveCollisions2D);
  connect(collisionSolveAct2D, SIGNAL(triggered()), this, SLOT(on_CollisionSolveAction2D_triggered()));

  QAction* collisionSolveActGeometric(new QAction(tr("&Solve Collisions (Geometric)"), this));
  collisionSolveActGeometric->setShortcut(Controls::Keyboard::Sequences::solveCollisionsGeometric);
  connect(collisionSolveActGeometric,SIGNAL(triggered()), this, SLOT(on_CollisionSolveActionGeometric_triggered()));

  QMenu* toolMenu = menuBar()->addMenu(tr("&Tools"));
  toolMenu->addAction(simplifyAct);
  toolMenu->addAction(createNewUnfoldAct);
  toolMenu->addAction(collisionSolveAct2D);
  toolMenu->addAction(collisionSolveActGeometric);

  // Extras Menu
  QAction* fullScreenAct(new QAction(tr("&Fullscreen"), this));
  fullScreenAct->setShortcut(Controls::Keyboard::Sequences::toggleFullScreen);
  fullScreenAct->setCheckable(true);
  connect(fullScreenAct, SIGNAL(triggered(bool)), this, SLOT(on_FullScreenAction_triggered()));

  QAction* computeDrawColorsMenuAct(new QAction(tr("&Compute Colorgradient"), this));
  computeDrawColorsMenuAct->setShortcut(Controls::Keyboard::Sequences::computeDrawColors);
  connect(computeDrawColorsMenuAct, SIGNAL(triggered()), this, SLOT(on_ComputeDrawColors_triggered()));

  QAction* colorSelectionAct(new QAction(tr("&Select ColorVector"), this));
  colorSelectionAct->setShortcut(Controls::Keyboard::Sequences::colorSelect);
  connect(colorSelectionAct, SIGNAL(triggered()), this, SLOT(on_ColorSelectionAction_triggered()));

  QAction* runDebugAct(new QAction(tr("&Run Debug"), this));
  runDebugAct->setShortcut(Controls::Keyboard::Sequences::runDebug);
  connect(runDebugAct, SIGNAL(triggered()), this, SLOT(on_RunDebugAct_triggered()));

  QMenu* extraMenu = menuBar()->addMenu(tr("&Extras"));
  extraMenu->addAction(fullScreenAct);
  extraMenu->addAction(computeDrawColorsMenuAct);
  extraMenu->addAction(colorSelectionAct);
  extraMenu->addAction(runDebugAct);

  QList<QAction*> actions = menuBar()->actions();
  emitToggledActions(actions);
}

QString MainWindow::setupFileDialog(const QString& title, const QString& filter, const QFileDialog::AcceptMode mode){
  QString file("");
  QFileDialog diag(this, title, QDir::homePath(), filter);
  diag.setOptions(QFileDialog::DontUseNativeDialog);
  diag.setAcceptMode(mode);
//  diag.setOption(QFileDialog::DontUseNativeDialog);
  if(diag.exec() &&  diag.selectedFiles().size() == 1){
    file = diag.selectedFiles()[0];
  }
  return file;
}

// Protected

void MainWindow::on_CollisionSolveAction2D_triggered(){
  CollisionSolverSelectDialog dialog(this);
  QTimeEdit* timeBox(new QTimeEdit(&dialog));
  QComboBox* chooser(new QComboBox(&dialog));
  dialog.setupSelectDialog(m_model->get2DSolvingMethods(), chooser, timeBox);

  if(dialog.exec()){
    QTime t = timeBox->time();
    int time = (t.hour() * 3600 + t.minute() * 60 + t.second());

    CollisionSolverProgressDialog progressDialog(this);
    progressDialog.setupProgressDialog(time);
    progressDialog.show();

    if(time == 0){
      time = std::numeric_limits<int>::max();
    }

    std::packaged_task<void(const int, const unsigned long)> solveTask(std::bind(&UnfoldModel::solveCollisions2D, m_model.get(), chooser->currentIndex(), time));
    std::future<void> result = solveTask.get_future();
    std::thread solveThread(std::move(solveTask), chooser->currentIndex(), time);
    auto status = result.wait_for(std::chrono::milliseconds(0));
    while(!progressDialog.wasCanceled() && status != std::future_status::ready){
      status = result.wait_for(std::chrono::milliseconds(500));
      progressDialog.setValue(progressDialog.value() + 500);
      progressDialog.repaint();
      m_openGLWidget->updateUnfoldTree();
      m_openGLWidget->repaint();
      m_twoDWidget->repaint();
    }
    if(progressDialog.wasCanceled()){
      m_model->interruptSolveCollisions2D();
    }
    if(status == std::future_status::ready){
      progressDialog.close();
    }
    solveThread.join();
    m_openGLWidget->updateUnfoldTree();
    m_twoDWidget->calibrateViewParameters();
  }
  repaintAllWidgets();
}

void MainWindow::on_CollisionSolveActionGeometric_triggered(){
  CollisionSolverSelectDialog dialog;
  QTimeEdit* timeBox(new QTimeEdit(&dialog));
  QComboBox* chooser(new QComboBox(&dialog));
  dialog.setupSelectDialog(m_model->getGeometricSolvingMethods(), chooser, timeBox);

  if(dialog.exec()){
    QTime t = timeBox->time();
    unsigned long int time = t.hour() * 3600 + t.minute() * 60 + t.second();

    CollisionSolverProgressDialog progressDialog(this);
    progressDialog.setupProgressDialog(time);
    progressDialog.show();

    if(time == 0){
      time = std::numeric_limits<unsigned long int>::max();
    }
    std::packaged_task<void(const int, const unsigned long)> solveTask(std::bind(&UnfoldModel::solveCollisionsGeometric, m_model.get(), chooser->currentIndex(), time));
    std::future<void> result = solveTask.get_future();
    std::thread solveThread(std::move(solveTask), chooser->currentIndex(), time);
    auto status = result.wait_for(std::chrono::milliseconds(0));
    while(!progressDialog.wasCanceled() && status != std::future_status::ready){
      status = result.wait_for(std::chrono::milliseconds(500));
      progressDialog.setValue(progressDialog.value() + 500);
      m_openGLWidget->updateUnfoldMeshVertices();
      m_openGLWidget->updateUnfoldTree();
      repaintAllWidgets();
    }
    if(progressDialog.wasCanceled()){
      m_model->interruptSolveCollisionsGeometric();
    }
    if(status == std::future_status::ready){
      progressDialog.close();
    }
    solveThread.join();
    m_openGLWidget->updateUnfoldMeshVertices();
    m_openGLWidget->updateUnfoldTree();
    m_twoDWidget->calibrateViewParameters();
  }
  repaintAllWidgets();
}

void MainWindow::on_ColorSelectionAction_triggered(){
  QDialog dialog;
  QHBoxLayout* selectLayout(new QHBoxLayout());
  QHBoxLayout* buttonLayout(new QHBoxLayout());
  QVBoxLayout* mainLayout(new QVBoxLayout(&dialog));
  QLabel* textLabel(new QLabel("Colorvector: "));
  QSpinBox* selectionBox(new QSpinBox());
  QPushButton* okButton(new QPushButton(QString("OK")));
  QPushButton* cancelButton(new QPushButton(QString("Cancel")));

  selectionBox->setMinimum(0);
  selectionBox->setMaximum(6);
  selectionBox->setValue(1);

  selectLayout->addWidget(textLabel);
  selectLayout->addWidget(selectionBox);

  okButton->connect(okButton, SIGNAL(released()), &dialog, SLOT(accept()));
  cancelButton->connect(cancelButton, SIGNAL(released()), &dialog, SLOT(reject()));

  buttonLayout->addStretch();
  buttonLayout->addWidget(okButton);
  buttonLayout->addWidget(cancelButton);

  mainLayout->addLayout(selectLayout);
  mainLayout->addLayout(buttonLayout);

  dialog.setLayout(mainLayout);
  dialog.setWindowTitle(QString("Select Colorvector."));
  if(dialog.exec()){
    extractColors(selectionBox->value());
  }
  repaintAllWidgets();
}

void MainWindow::on_ComputeDrawColors_triggered(){
  computeDrawColors();
  m_openGLWidget->updateUnfoldMeshVertices();
  repaintAllWidgets();
}

void MainWindow::on_CreateNewUnfoldAct_triggered(){
  m_model->createNewUnfolding();
  m_twoDWidget->calibrateViewParameters();
  m_twoDWidget->repaint();
  m_openGLWidget->updateUnfoldTree();
  m_openGLWidget->repaint();
}

void MainWindow::on_DropAct_triggered(){
  m_model->drop3D();
  repaintAllWidgets();
}

void MainWindow::on_DropUnfoldAct_triggered(){
  m_model->dropUnfold();
  repaintAllWidgets();
}

void MainWindow::on_FullScreenAction_triggered(){
  if(isFullScreen()){
    showNormal();
  }else{
    showFullScreen();
  }
}

void MainWindow::on_LoadOriginalMeshAction_triggered(){
  QString title("Load a Mesh");
  QString filter("Mesh Files (*.obj *.off *.stl *.wrl *.ply *.mesh)");
  QString file = setupFileDialog(title, filter, QFileDialog::AcceptOpen);
  if(file.length() == 0){
    return;
  }
  m_model->loadOriginalMesh(file.toStdString());
  m_openGLWidget->resetDrawProperties();
  m_model->setFaceColors(Eigen::MatrixXf::Zero(0, 0));
  m_openGLWidget->updateCameraPos();
  m_openGLWidget->bufferOriginalMesh();
  repaintAllWidgets();
}

void MainWindow::on_LoadFoldedMeshAction_triggered(){
  QString title("Load a Folded Mesh");
  QString filter("Mesh Files (*.obj *.off *.stl *.wrl *.ply *.mesh)");
  QString file = setupFileDialog(title, filter, QFileDialog::AcceptOpen);
  if(file.length() == 0){
    return;
  }
  m_model->loadFoldedMesh(file.toStdString());
  computeDrawColors();
  m_openGLWidget->bufferUnfoldMesh();
  m_twoDWidget->calibrateViewParameters();
  repaintAllWidgets();
}

void MainWindow::on_LoadUnfoldTreeAction_triggered(){
  QString filter("Unfoldtrees (*.unf)");
  QString title("Load an Unfoldtree");
  QString file = setupFileDialog(title, filter, QFileDialog::AcceptOpen);
  if(file.length() == 0){
    return;
  }
  m_model->loadUnfoldTree(file.toStdString());
  m_twoDWidget->calibrateViewParameters();
  m_twoDWidget->update();
  m_openGLWidget->bufferUnfoldTree();
  m_openGLWidget->update();
}

void MainWindow::on_RunDebugAct_triggered(){
  // Temporärer Code, nicht commiten.
}

void MainWindow::on_SaveColoredUnfolding_triggered(){
  QString filter("Unfoldings (*.svg)");
  QString title("Save a Colored Unfolding");
  QString file = setupFileDialog(title, filter, QFileDialog::AcceptSave);
  if(file.length() == 0){
    return;
  }
  if(file.split((".")).size() == 1){
    file.append(".svg");
  }
  m_model->saveColoredUnfolding(file.toStdString());
}

void MainWindow::on_SaveOriginalMeshAction_triggered(){
  QString filter("Mesh Files (*.obj *.off *.stl *.wrl *.ply *.mesh)");
  QString title("Save a Mesh");
  QString file = setupFileDialog(title, filter, QFileDialog::AcceptSave);
  if(file.length() == 0){
    return;
  }
  m_model->saveOriginalMesh(file.toStdString());
}

void MainWindow::on_SaveFoldedMeshAction_triggered(){
  QString filter("Mesh Files (*.off)");
  QString title("Save a Folded Mesh");
  QString file = setupFileDialog(title, filter, QFileDialog::AcceptSave);
  if(file.length() == 0){
    return;
  }
  if(file.split((".")).size() == 1){
    file.append(".off");
  }
  m_model->saveFoldedMesh(file.toStdString());
}

void MainWindow::on_SaveFoldingInstructionsAction_triggered(){
  QString filter("Text Files (*.txt)");
  QString title("Save the current unfolding");
  QString file = setupFileDialog(title, filter, QFileDialog::AcceptSave);
  if(file.length() == 0){
    return;
  }
  if(file.split((".")).size() == 1){
    file.append(".txt");
  }
  m_model->saveUnfoldInstructions(file.toStdString());
}

void MainWindow::on_SaveUnfoldingAction_triggered(){
  QString filter("Unfoldings (*.svg)");
  QString title("Save an Unfolding");
  QString file = setupFileDialog(title, filter, QFileDialog::AcceptSave);
  if(file.length() == 0){
    return;
  }
  if(file.split((".")).size() == 1){
    file.append(".svg");
  }
  m_model->saveUnfolding(file.toStdString());
}

void MainWindow::on_SaveUnfoldTreeAction_triggered(){
  QString filter("Unfoldtrees (*.unf)");
  QString title("Save an Unfoldtree");
  QString file = setupFileDialog(title, filter, QFileDialog::AcceptSave);
  if(file.length() == 0){
    return;
  }
  if(file.split((".")).size() == 1){
    file.append(".unf");
  }
  m_model->saveUnfoldTree(file.toStdString());
}

void MainWindow::on_SimplifyAct_triggered(){
  QDialog dialog;
  QHBoxLayout* absoluteLayout(new QHBoxLayout());
  QHBoxLayout* relativeLayout(new QHBoxLayout());
  QHBoxLayout* buttonLayout(new QHBoxLayout());
  QVBoxLayout* mainLayout(new QVBoxLayout(&dialog));
  QRadioButton* absolute(new QRadioButton(QString("Absolute (number): ")));
  QRadioButton* relative(new QRadioButton(QString("Relative (percent): ")));
  QDoubleSpinBox* absoluteSpinBox(new QDoubleSpinBox());
  QDoubleSpinBox* relativeSpinBox(new QDoubleSpinBox());
  QPushButton* okButton(new QPushButton(QString("OK")));
  QPushButton* cancelButton(new QPushButton(QString("Cancel")));

  absolute->connect(absolute, SIGNAL(toggled(bool)), absoluteSpinBox, SLOT(setEnabled(bool)));
  absolute->setChecked(true);
  relative->connect(relative, SIGNAL(toggled(bool)), relativeSpinBox, SLOT(setEnabled(bool)));
  relative->setChecked(false);

  const Mesh* const mesh = m_model->getModel3D()->getOriginalMesh();

  absoluteSpinBox->setMinimum(0);
  absoluteSpinBox->setDecimals(0);
  absoluteSpinBox->setMaximum(mesh->getFaces().size());
  absoluteSpinBox->setValue(mesh->getFaces().size());

  absoluteLayout->addWidget(absolute);
  absoluteLayout->addWidget(absoluteSpinBox);

  relativeSpinBox->setMinimum(0);
  relativeSpinBox->setDecimals(2);
  relativeSpinBox->setMaximum(100);
  relativeSpinBox->setValue(100);
  relativeSpinBox->setEnabled(false);

  relativeLayout->addWidget(relative);
  relativeLayout->addWidget(relativeSpinBox);

  okButton->connect(okButton, SIGNAL(released()), &dialog, SLOT(accept()));
  cancelButton->connect(cancelButton, SIGNAL(released()), &dialog, SLOT(reject()));

  buttonLayout->addStretch();
  buttonLayout->addWidget(okButton);
  buttonLayout->addWidget(cancelButton);

  mainLayout->addLayout(absoluteLayout);
  mainLayout->addLayout(relativeLayout);
  mainLayout->addLayout(buttonLayout);

  dialog.setLayout(mainLayout);
  dialog.setWindowTitle(QString("Set new number of Faces."));
  if(dialog.exec()){
    int numFaces;
    if(relative->isChecked()){
      numFaces = static_cast<int>(mesh->getFaces().size() * relativeSpinBox->value() / 100);
    }else{
      numFaces = static_cast<int>(absoluteSpinBox->value());
    }
    m_twoDWidget->resetSelections();
    m_model->simplifyOriginalMesh(numFaces);
    computeDrawColors();
    m_openGLWidget->bufferUnfoldMesh();
    m_twoDWidget->calibrateViewParameters();
  }
  repaintAllWidgets();
}

void MainWindow::repaintAllWidgets(){
  m_openGLWidget->update();
  m_twoDWidget->update();
}
