
#include "CCFTransformer.hpp"

// C++ Headers
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <thread>
#include <utility>

// Project Headers
#include <Datastructures/Mesh/FaceList.hpp>
#include <Util/DDGUtil.hpp>

CCFTransformer::CCFTransformer(UnfoldMesh* const m):
  GenericTransformer(m){
}

// Public

double CCFTransformer::getMaximalStepSize() const{
  return m_maximalStepSize;
}

double CCFTransformer::getMinimalStepSize() const{
  double minimalStepSize = m_maximalStepSize / std::log(m_mesh->getFaces().size() / m_mesh->getAverageDualValence());
  return std::min(m_maximalStepSize, minimalStepSize);
}

double CCFTransformer::getOptimalStepSize() const{
  return 0.1;
}

bool CCFTransformer::performStep(const double stepSize){
  // Setup Rho
  setupRho(stepSize);
  // Setup Eigenproblem
  setupEigenproblem();
  // Solve Eigenproblem
  if(!solveSmallestEigenproblem()){
    return false;
  }
  // Restore Positions
  if(!restorePositions()){
    return false;
  }
  // Copy Vertices
  copyVerticesAndUpdateMesh();
  // Calculate Energy
  calculateEnergy();

  return true;
}

void CCFTransformer::updateProperties(){
  // Update Properties
  // Assuming there are two threads per physical core.
  Eigen::setNbThreads(std::thread::hardware_concurrency() / 2);
  int nV = m_mesh->getVertices().rows();
  m_eigenMatrix.resize(4 * nV, 4 * nV);
  m_lambda.resize(4 * nV);
  m_omega.resize(nV, 4);
  m_rho.resize(nV);
  m_sigma = 0.;
  m_doFilter = true;
  m_maximalStepSize = 1. - std::numeric_limits<float>::epsilon();
  updateSolvers();
  calculateEnergy();
}

// Private

void CCFTransformer::buildAndProjectOutConstraints(){
  Eigen::MatrixXd constraints(m_mesh->getVertices().rows(), 7);

  // Filter
  if(m_doFilter){
    filter();
  }

  // Total Curvature
  constraints.col(0).setOnes();

  // Exactness
  // TODO: Implementieren
  constraints.block(0, 1, m_mesh->getVertices().rows(), 3).setZero();

  // Sphere Inversions
  const Eigen::MatrixXd& VN = m_mesh->getVertexNormals();
  constraints.block(0, 4, m_mesh->getVertices().rows(), 3) = Eigen::MatrixXd(VN);

  orthornormalize(constraints);

  projectOutConstraints(constraints);
}

void CCFTransformer::buildPoissonProblem(){
  // Build Omega Matrix
  m_omega.setZero();
  for(FaceList::const_iterator faceIterator = m_mesh->getFaces().cbegin(); faceIterator != m_mesh->getFaces().cend(); ++faceIterator){
    const std::vector<unsigned int>& currentFace = *faceIterator;
    for(size_t j = 0; j < currentFace.size(); j++){
      int i0 = currentFace[j];
      int i1 = currentFace[(j + 1) % 3];
      int i2 = currentFace[(j + 2) % 3];
      Eigen::Vector3d e0 = m_mesh->getVertices().row(i1) - m_mesh->getVertices().row(i0);
      Eigen::Vector3d e1 = m_mesh->getVertices().row(i2) - m_mesh->getVertices().row(i0);
      double cotHalf = e0.dot(e1) / (2. * e0.cross(e1).norm());

      // Omega
      if(i1 > i2){
        std::swap(i1, i2);
      }
      DerivedQuaterniond lambda1(m_lambda[4 * i1 + 0], m_lambda[4 * i1 + 1], m_lambda[4 * i1 + 2], m_lambda[4 * i1 + 3]);
      DerivedQuaterniond lambda2(m_lambda[4 * i2 + 0], m_lambda[4 * i2 + 1], m_lambda[4 * i2 + 2], m_lambda[4 * i2 + 3]);
      const Eigen::Vector3d edge = m_mesh->getVertices().row(i2) - m_mesh->getVertices().row(i1);
      DerivedQuaterniond quaterionicEdge(0, edge[0], edge[1], edge[2]);
      DerivedQuaterniond eTilde = lambda1.conjugate() * 1./3. * quaterionicEdge * lambda1 +
                                  lambda1.conjugate() * 1./6. * quaterionicEdge * lambda2 +
                                  lambda2.conjugate() * 1./6. * quaterionicEdge * lambda1 +
                                  lambda2.conjugate() * 1./3. * quaterionicEdge * lambda2;
      m_omega(i1, 0) += cotHalf * eTilde.w();
      m_omega(i1, 1) += cotHalf * eTilde.x();
      m_omega(i1, 2) += cotHalf * eTilde.y();
      m_omega(i1, 3) += cotHalf * eTilde.z();
      m_omega(i2, 0) -= cotHalf * eTilde.w();
      m_omega(i2, 1) -= cotHalf * eTilde.x();
      m_omega(i2, 2) -= cotHalf * eTilde.y();
      m_omega(i2, 3) -= cotHalf * eTilde.z();
    }
  }
  double mean = m_omega.mean();
  m_omega.array() -= mean;
}

void CCFTransformer::copyVerticesAndUpdateMesh(){
  // Vertices müssen geshifted werden, um die Orientierung zu erhalten.
  Eigen::MatrixXd& vertices = m_mesh->getVertices();
  vertices.col(0) = m_newVertices.col(3);
  vertices.col(1) = m_newVertices.col(1);
  vertices.col(2) = m_newVertices.col(2);
  m_mesh->shiftAndNormalize();
  m_mesh->updateMeanCurvature();
}

void CCFTransformer::mapQuaternionToBlockTriplets(const DerivedQuaterniond& q, int i, int j, std::vector<Eigen::Triplet<double>>& triplets){
  triplets.push_back(Eigen::Triplet<double>(i * 4 + 0, j * 4 + 0, +q.w()));
  triplets.push_back(Eigen::Triplet<double>(i * 4 + 0, j * 4 + 1, -q.x()));
  triplets.push_back(Eigen::Triplet<double>(i * 4 + 0, j * 4 + 2, -q.y()));
  triplets.push_back(Eigen::Triplet<double>(i * 4 + 0, j * 4 + 3, -q.z()));

  triplets.push_back(Eigen::Triplet<double>(i * 4 + 1, j * 4 + 0, +q.x()));
  triplets.push_back(Eigen::Triplet<double>(i * 4 + 1, j * 4 + 1, +q.w()));
  triplets.push_back(Eigen::Triplet<double>(i * 4 + 1, j * 4 + 2, -q.z()));
  triplets.push_back(Eigen::Triplet<double>(i * 4 + 1, j * 4 + 3, +q.y()));

  triplets.push_back(Eigen::Triplet<double>(i * 4 + 2, j * 4 + 0, +q.y()));
  triplets.push_back(Eigen::Triplet<double>(i * 4 + 2, j * 4 + 1, +q.z()));
  triplets.push_back(Eigen::Triplet<double>(i * 4 + 2, j * 4 + 2, +q.w()));
  triplets.push_back(Eigen::Triplet<double>(i * 4 + 2, j * 4 + 3, -q.x()));

  triplets.push_back(Eigen::Triplet<double>(i * 4 + 3, j * 4 + 0, +q.z()));
  triplets.push_back(Eigen::Triplet<double>(i * 4 + 3, j * 4 + 1, -q.y()));
  triplets.push_back(Eigen::Triplet<double>(i * 4 + 3, j * 4 + 2, +q.x()));
  triplets.push_back(Eigen::Triplet<double>(i * 4 + 3, j * 4 + 3, +q.w()));
}

void CCFTransformer::orthornormalize(Eigen::MatrixXd& Matrix){
  for(int i = 0; i < Matrix.cols(); ++i){
    for(int j = 0; j < i; ++j){
      Matrix.col(i) -= Matrix.col(i).transpose() * m_mesh->getVertexMassMatrix() * Matrix.col(j) * Matrix.col(j);
    }
    Matrix.col(i).normalize();
  }
}

void CCFTransformer::projectOutConstraints(const Eigen::MatrixXd& C){
  // Project constraints out
  const Eigen::SparseMatrix<double>& M = m_mesh->getVertexMassMatrix();
  Eigen::VectorXd copy = Eigen::VectorXd(m_rho);
  Eigen::VectorXd factors = C.transpose() * M * m_rho;
  for(int i = 0; i < C.cols(); ++i){
    m_rho -= C.col(i) * factors[i];
  }
}

bool CCFTransformer::restorePositions(){
  // Build Poisson Problem
  buildPoissonProblem();
  // Solve Poisson Problem
  m_newVertices = m_positionSolver.solve(-m_omega);
  return true;
}

void CCFTransformer::setupEigenproblem(){
  m_mesh->calculateFaceAreas();
  const Eigen::VectorXd& faceAreas = m_mesh->getFaceAreas();
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(m_mesh->getFaces().size() * 10 * 16);
  for(FaceList::const_iterator faceIterator = m_mesh->getFaces().cbegin(); faceIterator != m_mesh->getFaces().cend(); ++faceIterator){
    const std::vector<unsigned int>& currentFace = *faceIterator;
    double faceArea = faceAreas[std::distance(m_mesh->getFaces().cbegin(), faceIterator)];
    double a = -1. / (4. * faceArea);
    double b = faceArea / 9.;
    std::array<DerivedQuaterniond, 3> edges;
    for(int i = 0; i < 3; ++i){
      std::pair<int, int> indices = {currentFace[(i + 2) % 3], currentFace[(i + 1) % 3]};
      Eigen::Vector3d edge = m_mesh->getVertices().row(indices.first) - m_mesh->getVertices().row(indices.second);
      edges[i] = DerivedQuaterniond(0, edge[0], edge[1], edge[2]);
    }
    for(int localIndexI = 0; localIndexI < 3; ++localIndexI){
      size_t globalIndexI = currentFace[localIndexI];
      for(int localIndexJ = 0; localIndexJ < 3; ++localIndexJ){
        size_t globalIndexJ = currentFace[localIndexJ];
        DerivedQuaterniond q = edges[localIndexI] * edges[localIndexJ] * a + (edges[localIndexJ] * m_rho[globalIndexI] - edges[localIndexI] * m_rho[globalIndexJ]) / 6. + b * m_rho[globalIndexI] * m_rho[globalIndexJ];
        mapQuaternionToBlockTriplets(q, globalIndexI, globalIndexJ, triplets);
      }
    }
  }
  m_eigenMatrix.setZero();
  m_eigenMatrix.setFromTriplets(triplets.begin(), triplets.end());
}

void CCFTransformer::setupRho(double stepSize){
  // Step Size
  m_rho = -m_mesh->getMeanCurvature() * stepSize;

  buildAndProjectOutConstraints();
}

bool CCFTransformer::solveSmallestEigenproblem(){
  m_eigenSolver.factorize(m_eigenMatrix);
  if(m_eigenSolver.info() != Eigen::Success){
    std::cerr << "Failed to factorize Eigenproblem" << std::endl;
    return false;
  }
  Eigen::VectorXd b(m_lambda.rows());
  m_lambda.setZero();
  b.setOnes().normalize();
  double absDotProduct = 0.;
  int maxIterations = 10;
  int iterations = 0;
  // Inverse Power Method
  while(1. - absDotProduct > std::numeric_limits<double>::epsilon() && ++iterations <= maxIterations){
    m_lambda = m_eigenSolver.solve(b);
    m_lambda.normalize();
    absDotProduct = std::fabs(m_lambda.dot(b));
    b = m_lambda;
  }
  return true;
}

void CCFTransformer::updateSolvers(){
  m_cotMatrix = m_mesh->getCotMatrix();

  // Prepare FilterSolver
  updateFilterSolver();

  // Analyze Eigenpattern
  setupEigenproblem();
  m_eigenSolver.analyzePattern(m_eigenMatrix);

  // Analyze Laplace pattern
  m_positionSolver.analyzePattern(m_mesh->getCotMatrix());
  m_positionSolver.factorize(-m_mesh->getCotMatrix());

  if(m_positionSolver.info() != Eigen::Success){
    Eigen::SparseMatrix<double> I(m_cotMatrix.rows(), m_cotMatrix.rows());
    I.setIdentity();
    m_positionSolver.factorize(-(m_cotMatrix + I * std::numeric_limits<double>::epsilon()));
    if(m_positionSolver.info() != Eigen::Success){
      m_positionSolver.factorize(-(m_cotMatrix + I * std::numeric_limits<float>::epsilon()));
      if(m_positionSolver.info() != Eigen::Success){
        std::cerr << "Failed to factorize laplace matrix." << std::endl;
      }
    }
  }
}

// Protected
