
#include "CollisionSolverHolisticGeometric.hpp"

// C++ Headers
#include <algorithm>
#include <array>
#include <ctime>
#include <iostream>
#include <limits>
#include <utility>

// Project Headers
#include <Collisions/Solver/2D/TabuSolver.hpp>

CollisionSolverHolisticGeometric::CollisionSolverHolisticGeometric(Model2D* model2D, Model3D* model3D, Unfolder* unfolder, GenericTransformer* transformer):
  CollisionSolverHolisticBase(model2D, model3D, unfolder),
  m_transformer(std::move(transformer)){
}

// Public

int CollisionSolverHolisticGeometric::solve(const unsigned long seconds){
  std::cout << "Number of collisions: " << m_collisionDetector->getNumberOfCollisions() << std::endl;
  m_interruptSolving = false;
  m_startTime = std::clock();

  m_initialFaceAreas = m_mesh->calculateFaceAreas();

  // Transform
  if(!transformMesh(seconds) || m_interruptSolving){
    abort(true);
    return -1;
  }

  // Unfold
  if(!unfoldMesh(seconds) || m_interruptSolving){
    abort(true);
    return -1;
  }

  // Inverse Transform
  if(!invTransformMesh(seconds) || m_interruptSolving){
    abort(false);
    return 1;
  }
  m_mesh->updateMeanCurvature();
  return 0;
}

// Protected

void CollisionSolverHolisticGeometric::abort(bool restore){
  if(m_collisionSolver){
    m_collisionSolver->interruptSolve();
  }
  if((restore) && !m_timeline.empty()){
    m_mesh->setVertices(m_timeline[0]);
    m_mesh->calculateNormals();
  }else{
    m_mesh->calculateNormals();
  }
  m_mesh->updateMeanCurvature();
  m_unfoldTreeList->updateTransformations();
  m_model2D->generatePolygonList();
}

void CollisionSolverHolisticGeometric::cacheValidState(){
  m_lastVertices = m_mesh->getVertices();
  m_unfoldTreeList->cacheCurrentTree();
}

bool CollisionSolverHolisticGeometric::invTransformMesh(const unsigned long seconds){
  std::cout << "Inverse Transforming" << std::endl;
  while(!m_interruptSolving &&
        !m_timeline.empty()){
    cacheValidState();
    m_mesh->setVertices(m_timeline.back());
    m_timeline.pop_back();
    m_mesh->calculateFaceNormals();
    m_mesh->updateAllTransformations();
    m_unfoldTreeList->updateTransformations();
    m_model2D->generatePolygonList();

    unsigned long remainingTime = seconds - getRuntime();
    if(remainingTime > seconds){
      restoreLastValidState();
      return false;
    }
    if(!isTimeLeft(seconds) || (m_collisionSolver->solve(remainingTime) != 0)){
      restoreLastValidState();
      return false;
    }
  }
  return true;
}

bool CollisionSolverHolisticGeometric::isTransformMeshConverged(const std::array<std::pair<double, double>, 2>& oldEnergies, const double currentEnergy, const double accumulatedStepSize){
  if(accumulatedStepSize == 0.){
    return false;
  }
  for(std::array<std::pair<double, double>, 2>::const_iterator it = oldEnergies.cbegin(); it != oldEnergies.cend(); ++it){
    if((currentEnergy - it->second) / (accumulatedStepSize - it->first) < -0.1){
      return false;
    }
  }
  return true;
}

void CollisionSolverHolisticGeometric::restoreLastValidState(){
  m_mesh->setVertices(m_lastVertices);
  m_mesh->calculateFaceNormals();
  m_mesh->updateAllTransformations();
  m_unfoldTreeList->restoreCachedTree();
  m_model2D->generatePolygonList();
}

bool CollisionSolverHolisticGeometric::transformMesh(const unsigned long seconds){
  std::cout << "Transforming" << std::endl;
  m_timeline.clear();
  m_timeline.reserve(200);
  m_timeline.push_back(m_mesh->getVertices());

  double stepSize = m_transformer->getMinimalStepSize();
  double accumulatedStepSize = 0.;

  double currentEnergy = m_transformer->getCurrentEnergy();
  // Accumulated StepSize, Energy
  std::array<std::pair<double, double>, 2> oldEnergies;
  oldEnergies[0] = {accumulatedStepSize, std::numeric_limits<double>::max()};
  oldEnergies[1] = {accumulatedStepSize, std::numeric_limits<double>::max()};

  while(!m_interruptSolving &&
        isTimeLeft(seconds) &&
        !isTransformMeshConverged(oldEnergies, currentEnergy, accumulatedStepSize)){
    oldEnergies[0] = std::move(oldEnergies[1]);
    oldEnergies[1] = {accumulatedStepSize, currentEnergy};

    if(!m_transformer->performStep(stepSize)){
      return false;
    }

    m_timeline.push_back(m_mesh->getVertices());
    accumulatedStepSize += stepSize;
    currentEnergy = m_transformer->getCurrentEnergy();

    stepSize *= 2.;
    stepSize = std::min(m_transformer->getMaximalStepSize(), stepSize);
  }
  std::cout << "Number of transformations: " << m_timeline.size() - 1 << std::endl;
  m_mesh->calculateFaceNormals();
  m_mesh->updateAllTransformations();
  return true;
}

bool CollisionSolverHolisticGeometric::unfoldMesh(const unsigned long seconds){
  std::cout << "Unfolding Transformed Mesh" << std::endl;
  const Eigen::VectorXd faceAreas = m_mesh->calculateFaceAreas();
  Eigen::MatrixXd weights(faceAreas.rows(), 2);
  weights.col(0) = m_initialFaceAreas.array() / faceAreas.array();
  weights.col(1) = faceAreas.array() / m_initialFaceAreas.array();
  const Eigen::VectorXd weightVector = weights.rowwise().maxCoeff();
  m_unfolder->createNewUnfoldTree(weightVector);
  m_model2D->generatePolygonList();
  m_collisionDetector->detectAllCollisions();
  m_collisionSolver.reset(new TabuSolver(m_model2D, m_unfolder));
  if(m_collisionDetector->getNumberOfCollisions() == 0){
    return true;
  }
  unsigned long remainingTime = seconds - getRuntime();
  if(remainingTime > seconds){
    return false;
  }
  return (m_collisionSolver->solve(remainingTime) == 0);
}
