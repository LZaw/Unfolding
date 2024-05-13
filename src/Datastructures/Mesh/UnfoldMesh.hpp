
#pragma once

// Project Headers
#include <Datastructures/Matrices/LaplaceMatrix.hpp>
#include <Datastructures/Matrices/MassMatrix.hpp>
#include <Datastructures/Mesh/DualGraph.hpp>
#include <Datastructures/Mesh/Mesh.hpp>


class OriginalMesh;
class UnfoldMesh: public Mesh{
public:
  UnfoldMesh();

  const Eigen::VectorXd& calculateAngleDeficit();
  const Eigen::VectorXd& calculateFaceAreas();
  int calculateGenus();
  const Eigen::VectorXd& calculateMeanCurvature();
  void calculateSumPointsPerFace();
  void createDualGraph(bool withTransformations = true);

  void drop();

  double getAverageDualValence() const;
  const LaplaceMatrix& getCotMatrix() const;
  const DualGraph& getDualGraph() const;
  const Eigen::VectorXd& getFaceAreas() const;
  const std::vector<std::vector<unsigned int>>& getFacesPerVertex() const;
  int getGenus() const;
  const Eigen::VectorXd& getMeanCurvature() const;
  unsigned int getSumPointsPerFace() const;
  const MassMatrix& getVertexMassMatrix() const;

  void initializeMembers();

  void normalizeSurfaceArea();

  void planarize();

  void setupCotMatrix();
  void setupFacesPerVertex();
  void setupVertexMassMatrix(const MassMatrix::MassMatrixType type = MassMatrix::MassMatrixType::MIXEDVORONOI);

  void updateAllTransformations();
  void updateMeanCurvature(bool updateCotMatrix = false);

protected:
  void postReadInitialization() override;

private:
  Eigen::VectorXd                 m_angleDeficit;
  LaplaceMatrix                   m_cotMatrix;
  DualGraph                       m_dualGraph;
  Eigen::VectorXd                 m_faceAreas;
  std::vector<
    std::vector<
      unsigned int>>              m_facesPerVertex;
  Eigen::VectorXd                 m_gaussianCurvature;
  int                             m_genus = -1;
  Eigen::VectorXd                 m_meanCurvature;
  unsigned int                    m_sumPointsPerFace = 0;
  
  MassMatrix                      m_vertexMassMatrix;
  

  void updateFaceNormal(const unsigned int faceID);
  void updateFaceNormalsAroundVertex(const unsigned int vertexID);
};
