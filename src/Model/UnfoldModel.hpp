
#pragma once

// C++ Headers
#include <memory>
#include <string>

// Project Headers
#include <Collisions/Solver/Holistic/Geometric/CollisionSolverHolisticGeometric.hpp>
#include <Model/Model2D.hpp>
#include <Model/Model3D.hpp>
#include <Unfolding/Unfolder.hpp>

// Eigen Headers
#include <Eigen/Dense>

class UnfoldModel{
public:
  UnfoldModel();

  void attachPolygonToNewNeighbor(const unsigned int polygonIndex, const unsigned int newNeighborIndex);

  void createNewUnfolding(const Eigen::VectorXd& weights = Eigen::VectorXd::Zero(1));

  void drop3D();
  void dropUnfold();

  const std::vector<std::string>& get2DSolvingMethods() const;
  const std::vector<std::string>& getGeometricSolvingMethods() const;
  const Model2D* getModel2D() const;
  Model2D* getModel2D();
  const Model3D* getModel3D() const;
  Model3D* getModel3D();
  const Unfolder* getUnfolder() const;
  Unfolder* getUnfolder();

  void interruptSolveCollisions2D();
  void interruptSolveCollisionsGeometric();

  void loadFoldedMesh(const std::string& path);
  void loadOriginalMesh(const std::string& path);
  void loadUnfoldTree(const std::string& path);

  void rerootUnfoldTree(const int newRootIndex, bool generatePolygonList = true);

  void saveColoredUnfolding(const std::string& path) const;
  void saveFoldedMesh(const std::string& path) const;
  void saveOriginalMesh(const std::string& path) const;
  void saveUnfolding(const std::string& path) const;
  void saveUnfoldInstructions(const std::string& path) const;
  void saveUnfoldTree(const std::string& path) const;
  void setFaceColors(const Eigen::MatrixXf& faceColors);
  void simplifyOriginalMesh(const int numFaces);
  int solveCollisions2D(const int method, const unsigned long int seconds);
  int solveCollisionsGeometric(const int method, const unsigned long int seconds);

private:
  std::unique_ptr<
    CollisionSolver2DBase>                  m_collisionSolver2D;
  std::unique_ptr<
    CollisionSolverHolisticGeometric>       m_collisionSolverGeometric;
  std::unique_ptr<Model2D>                  m_model2D;
  std::unique_ptr<Model3D>                  m_model3D;
  const std::vector<
    std::string>                            m_solvingMethods2D = {"Random Solver", "Random Collision Solver", "Predicting Random Collision Solver", "Tabu Solver"};
  const std::vector<
    std::string>                            m_solvingMethodsGeometric = {"Conformalized Mean Curvature Flow Solver", "CCF Zero Flow Solver", "CCF Laplace Flow Solver", "CCF Willmore Flow Solver", "Angle Equalize Flow Solver", "Edge Normal Equalize Flow Solver", "Mass Spring Flow Solver"};
  std::unique_ptr<
    Unfolder>                               m_unfolder;
};
