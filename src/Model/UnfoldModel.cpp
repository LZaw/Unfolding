
#include "UnfoldModel.hpp"

// C++ Headers
#include <cstdlib>
#include <ctime>
#include <iostream>

// Project Headers
#include <Collisions/Solver/2D/PredictingRandomCollisionSolver.hpp>
#include <Collisions/Solver/2D/RandomCollisionSolver.hpp>
#include <Collisions/Solver/2D/RandomSolver.hpp>
#include <Collisions/Solver/2D/TabuSolver.hpp>
#include <Collisions/Solver/Holistic/Geometric/CollisionSolverHolisticGeometric.hpp>
#include <Unfolding/Unfolder.hpp>
#include <Transformer/CCF/CCFLaplaceTransformer.hpp>
#include <Transformer/CCF/CCFWillmoreTransformer.hpp>
#include <Transformer/CCF/CCFZeroTransformer.hpp>
#include <Transformer/AngleEqualizeTransformer.hpp>
#include <Transformer/ConformalizedMeanCurvatureTransformer.hpp>
#include <Transformer/EdgeNormalAlignTransformer.hpp>
#include <Transformer/MassSpringTransformer.hpp>
#include <Util/FileUtil.hpp>

UnfoldModel::UnfoldModel():
  m_model3D(new Model3D()),
  m_unfolder(new Unfolder(m_model3D->getUnfoldMesh())){
  std::srand(std::time(nullptr));
  m_unfolder->setUnfoldMethod(Unfolder::UnfoldMethod::SteepestEdge);
  m_model2D.reset(new Model2D(m_model3D->getUnfoldMesh()));
  m_model2D->setUnfoldTreeList(&m_unfolder->getUnfoldTreeList());
}

// Public

void UnfoldModel::attachPolygonToNewNeighbor(const unsigned int polygonIndex, const unsigned int newNeighborIndex){
  m_model2D->attachPolygonToNewNeighbor(polygonIndex, newNeighborIndex);
}

void UnfoldModel::createNewUnfolding(const Eigen::VectorXd& weights){
  if(!m_model3D->getUnfoldMesh()->getFaces().empty()){
    m_unfolder->createNewUnfoldTree(weights);
    m_model2D->generatePolygonList();
  }
}

void UnfoldModel::drop3D(){
  dropUnfold();
  m_model3D->dropOriginalMesh();
}

void UnfoldModel::dropUnfold(){
  m_unfolder->resetTree();
  m_model2D->drop();
  m_model3D->dropFoldedMesh();
}

const std::vector<std::string>& UnfoldModel::get2DSolvingMethods() const{
  return m_solvingMethods2D;
}

const std::vector<std::string>& UnfoldModel::getGeometricSolvingMethods() const{
  return m_solvingMethodsGeometric;
}

const Model2D* UnfoldModel::getModel2D() const{
  return m_model2D.get();
}
Model2D* UnfoldModel::getModel2D(){
  return m_model2D.get();
}

const Model3D* UnfoldModel::getModel3D() const{
  return m_model3D.get();
}

Model3D* UnfoldModel::getModel3D(){
  return m_model3D.get();
}

const Unfolder* UnfoldModel::getUnfolder() const{
  return m_unfolder.get();
}

Unfolder* UnfoldModel::getUnfolder(){
  return m_unfolder.get();
}

void UnfoldModel::interruptSolveCollisions2D(){
  m_collisionSolver2D->interruptSolve();
}

void UnfoldModel::interruptSolveCollisionsGeometric(){
  m_collisionSolverGeometric->interruptSolve();
  if(m_collisionSolverGeometric->getSolver()){
    m_collisionSolverGeometric->getSolver()->interruptSolve();
  }
}

void UnfoldModel::loadFoldedMesh(const std::string& path){
  m_model3D->loadFoldedMesh(path);
  loadUnfoldTree(FileUtil::removeFileEnding(path) + ".unf");
}

void UnfoldModel::loadOriginalMesh(const std::string& path){
  m_unfolder->drop();
  m_model2D->drop();
  m_model3D->loadOriginalMesh(path);
}

void UnfoldModel::loadUnfoldTree(const std::string& path){
  m_unfolder->loadUnfoldTree(path);
  m_model2D->generatePolygonList();
}

void UnfoldModel::rerootUnfoldTree(const int newRootIndex, bool generatePolygonList){
  m_unfolder->rerootTree(newRootIndex);
  if(generatePolygonList){
    m_model2D->updatePolygonList();
  }
}

void UnfoldModel::saveColoredUnfolding(const std::string& path) const{
  if(FileUtil::hasFileEnding(path)){
    m_model2D->saveColoredUnfolding(path);
  }else{
    m_model2D->saveColoredUnfolding(path + ".svg");
  }
}

void UnfoldModel::saveFoldedMesh(const std::string& path) const{
  if(FileUtil::hasFileEnding(path)){
    m_model3D->saveFoldedMesh(path);
    saveUnfoldTree(FileUtil::removeFileEnding(path) + ".unf");
  }else{
    m_model3D->saveFoldedMesh(path + ".off");
    saveUnfoldTree(path + ".unf");
  }
}

void UnfoldModel::saveOriginalMesh(const std::string& path) const{
  m_model3D->saveOriginalMesh(path);
}

void UnfoldModel::saveUnfolding(const std::string& path) const{
  if(FileUtil::hasFileEnding(path)){
    m_model2D->saveUnfolding(path);
    m_unfolder->saveFoldingInstructions(FileUtil::removeFileEnding(path) + ".txt");
  }else{
    m_model2D->saveUnfolding(path + ".svg");
    m_unfolder->saveFoldingInstructions(path + ".txt");
  }
}

void UnfoldModel::saveUnfoldInstructions(const std::string& path) const{
  m_unfolder->saveFoldingInstructions(path);
}

void UnfoldModel::saveUnfoldTree(const std::string& path) const{
  m_unfolder->saveUnfoldTree(path);
}

void UnfoldModel::setFaceColors(const Eigen::MatrixXf& faceColors){
  m_model2D->getPolygonListRef().setColors(faceColors);
}

void UnfoldModel::simplifyOriginalMesh(const int numFaces){
  m_model3D->simplifyOriginalMesh(numFaces);
  m_unfolder->createNewUnfoldTree();
  m_model2D->generatePolygonList();
}

int UnfoldModel::solveCollisions2D(const int method, const unsigned long int seconds){
  CollisionDetector& collisionDetector = m_model2D->getCollisionDetector();
  PolygonList& polygonList = m_model2D->getPolygonListRef();
  UnfoldTreeList* unfoldTreeList = &m_unfolder->getUnfoldTreeList();
  if(collisionDetector.getNumberOfCollisions() == 0){
    std::cout << "No collisions detected." << std::endl;
    std::cout << "Iterations done: " << 0 << std::endl;
    std::cout << "Needed " << 0. << " seconds." << std::endl;
    return 0;
  }
  std::cout << "Solving collisions..." << std::endl;
  switch(method){
  case 0:{
    // Random "Solver"
    std::cout << "Random Solver" << std::endl;
    m_collisionSolver2D.reset(new RandomSolver(unfoldTreeList, &polygonList, &collisionDetector));
    break;
  }
  case 1:{
    // Random Collision Solver
    std::cout << "Random Collision Solver" << std::endl;
    m_collisionSolver2D.reset(new RandomCollisionSolver(unfoldTreeList, &polygonList, &collisionDetector));
    break;
  }
  case 2:{
    // Predicting Random Collision Solver
    std::cout << "Predicting Random Solver" << std::endl;
    m_collisionSolver2D.reset(new PredictingRandomCollisionSolver(unfoldTreeList, &polygonList, &collisionDetector));
    break;
  }
  case 3:{
    // Tabu Solver
    std::cout << "Tabu Solver" << std::endl;
    m_collisionSolver2D.reset(new TabuSolver(m_model2D.get(), m_unfolder.get()));
    break;
  }
  default:{
    std::cerr << "Method not supported yet." << std::endl;
    return false;
  }
  };
  std::clock_t startTime = std::clock();
  int solved = m_collisionSolver2D->solve(seconds);
  std::clock_t endTime = std::clock();
  if(solved == 0){
    std::cout << "Successfully solved." << std::endl;
  }else{
    std::cerr << "Could not solve." << std::endl;
  }
  std::cout << "Needed " << ((endTime - startTime) / (CLOCKS_PER_SEC / 1000.)) / 1000. << " seconds." << std::endl;
  m_model2D->updatePolygonList();
  m_collisionSolver2D.reset();
  return solved;
}

int UnfoldModel::solveCollisionsGeometric(const int method, const unsigned long int seconds){
  if(m_model2D->getNumberOfCollisions() == 0){
    std::cout << "No collisions detected." << std::endl;
    std::cout << "Steps taken: " << 0 << std::endl;
    std::cout << "Needed " << 0. << " seconds." << std::endl;
    return 0;
  }
  std::cout << "Solving collisions..." << std::endl;
  std::cout << m_solvingMethodsGeometric[method] << std::endl;
  switch(method){
  case 0:{
    // Conformalized Mean Curvature Flow
    // Set Unfolder to MSTQuot
    m_unfolder->setUnfoldMethod(Unfolder::UnfoldMethod::MSTQuotient);
    // Set Solver
    m_collisionSolverGeometric.reset(new CollisionSolverHolisticGeometric(m_model2D.get(), m_model3D.get(), m_unfolder.get(), new ConformalizedMeanCurvatureTransformer(m_model3D->getUnfoldMesh())));
    break;
  }
  case 1:{
    // Conformal Zero Flow
    // Set Unfolder to MSTQuot
    m_unfolder->setUnfoldMethod(Unfolder::UnfoldMethod::MSTQuotient);
    // Set Solver
    m_collisionSolverGeometric.reset(new CollisionSolverHolisticGeometric(m_model2D.get(), m_model3D.get(), m_unfolder.get(), new CCFZeroTransformer(m_model3D->getUnfoldMesh())));
    break;
  }
  case 2:{
    // Conformal Laplace Flow
    // Set Unfolder to MSTQuot
    m_unfolder->setUnfoldMethod(Unfolder::UnfoldMethod::MSTQuotient);
    // Set Solver
    m_collisionSolverGeometric.reset(new CollisionSolverHolisticGeometric(m_model2D.get(), m_model3D.get(), m_unfolder.get(), new CCFLaplaceTransformer(m_model3D->getUnfoldMesh())));
    break;
  }
  case 3:{
    // Conformal Willmore Flow
    // Set Unfolder to MSTQuot
    m_unfolder->setUnfoldMethod(Unfolder::UnfoldMethod::MSTQuotient);
    // Set Solver
    m_collisionSolverGeometric.reset(new CollisionSolverHolisticGeometric(m_model2D.get(), m_model3D.get(), m_unfolder.get(), new CCFWillmoreTransformer(m_model3D->getUnfoldMesh())));
    break;
  }
  case 4:{
    // Angle Equalize Flow Solver
    // Set Unfolder to MSTQuot
    m_unfolder->setUnfoldMethod(Unfolder::UnfoldMethod::MSTQuotient);
    // Set Solver
    m_collisionSolverGeometric.reset(new CollisionSolverHolisticGeometric(m_model2D.get(), m_model3D.get(), m_unfolder.get(), new AngleEqualizeTransformer(m_model3D->getUnfoldMesh())));
    break;
  }
  case 5:{
    // Edge Normal Equalize Flow Solver
    // Set Unfolder to MSTQuot
    m_unfolder->setUnfoldMethod(Unfolder::UnfoldMethod::MSTQuotient);
    // Set Solver
    m_collisionSolverGeometric.reset(new CollisionSolverHolisticGeometric(m_model2D.get(), m_model3D.get(), m_unfolder.get(), new EdgeNormalAlignTransformer(m_model3D->getUnfoldMesh())));
    break;
  }
  case 6:{
    // Mass Spring Flow Solver
    // Set Unfolder to MSTQuot
    m_unfolder->setUnfoldMethod(Unfolder::UnfoldMethod::MSTQuotient);
    // Set Solver
    m_collisionSolverGeometric.reset(new CollisionSolverHolisticGeometric(m_model2D.get(), m_model3D.get(), m_unfolder.get(), new MassSpringTransformer(m_model3D->getUnfoldMesh())));
    break;
  }
  default:{
    std::cerr << "Method not supported yet." << std::endl;
    return false;
  }
  }
  std::clock_t startTime = std::clock();
  int solved = m_collisionSolverGeometric->solve(seconds);
  std::clock_t endTime = std::clock();
  switch(solved){
    case 0:{
      std::cout << "Successfully solved." << std::endl;
      break;
    }
    case 1:{
      std::cout << "Approximative result." << std::endl;
      break;
    }
    default:{
      std::cerr << "Could not solve." << std::endl;
    }
  }
  std::cout << "Needed " << ((endTime - startTime) / (CLOCKS_PER_SEC / 1000.)) / 1000. << " seconds." << std::endl;
  m_model2D->updatePolygonList();
  m_collisionSolverGeometric.reset();

  // Reset Unfolder
  m_unfolder->setUnfoldMethod(Unfolder::UnfoldMethod::SteepestEdge);
  return solved;
}
