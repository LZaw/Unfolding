
#include "OriginalMesh.hpp"

// C++ Headers
#include <iostream>
#include <numeric>

// Project Headers
#include <Datastructures/Mesh/UnfoldMesh.hpp>

// IGL Headers
#include <igl/decimate.h>

OriginalMesh::OriginalMesh():
  Mesh(){
}

// Public

void OriginalMesh::drop(){
  Mesh::drop();
}

void OriginalMesh::simplify(UnfoldMesh& targetMesh, const unsigned int numFaces){
  std::cout << "Simplifying." << std::endl;
  targetMesh.drop();
  if(numFaces == m_faces.size()){
    targetMesh.setVertices(m_vertices);
    targetMesh.setFaces(m_faces);
  }else{
    const auto& shortestEdgeFunction =
      [](const int e, const Eigen::MatrixXd& V, const Eigen::MatrixXi& /*F*/,
        const Eigen::MatrixXi& E, const Eigen::VectorXi& /*EMAP*/, const Eigen::MatrixXi& /*EF*/,
        const Eigen::MatrixXi& /*EI*/, double& cost, Eigen::RowVectorXd& p){
        cost = (V.row(E(e, 0)) - V.row(E(e, 1))).norm();
        p = .5 * (V.row(E(e, 0)) + V.row(E(e, 1)));
      };
    Eigen::VectorXi J, I;
    Eigen::MatrixXi F(m_faces.size(), 3);
    for(size_t i = 0; i < m_faces.size(); i++){
      for(size_t j = 0; j < m_faces[i].size(); j++){
        F(i, j) = m_faces[i][j];
      }
    }
    Eigen::MatrixXd targetV;
    Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> targetF;
    int numThisFaces = m_faces.size();
    igl::decimate(m_vertices, F, shortestEdgeFunction, igl::max_faces_stopping_condition(numThisFaces, numThisFaces, numFaces), targetV, targetF, J, I);
    targetMesh.setVertices(targetV);
    FaceList targetFaces(targetF.rows());
    for(Eigen::Index i = 0; i < targetF.rows(); i++){
      targetFaces[i].resize(targetF.cols());
      for(Eigen::Index j = 0; j < targetF.cols(); j++){
        targetFaces[i][j] = targetF(i, j);
      }
    }
    targetMesh.setFaces(targetFaces);
    targetMesh.calculateSumPointsPerFace();
    targetMesh.setupFacesPerVertex();
    targetMesh.calculateNormals();
    targetMesh.calculateGenus();
    targetMesh.createDualGraph();
    std::cout << "Mesh simplified! New number of faces: " << targetMesh.getFaces().size() << std::endl;
  }
}

// Protected

void OriginalMesh::postReadInitialization(){
  shiftBoundingBoxCenterToOrigin();
}
