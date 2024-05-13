#define EIGEN_NO_DEBUG 1
#define EIGEN_NO_STATIC_ASSERT 1

// Terminal Version

// C++ Headers
#include <iostream>
#include <string>

// Project Headers
#include <Model/UnfoldModel.hpp>

// Eigen Headers

int main(int argc, char* argv[]){
  if(argc < 5){
    std::cout << "Usage: ./Unfolding inputModel resolvingMethod abortTime pathToUnfolding [pathToApproximativeModel]" << std::endl;
    return 0;
  }
  const std::string inputModel = std::string(argv[1]);
  const std::string method = std::string(argv[2]);
  const unsigned long abortTime = std::stoul(argv[3]);
  const std::string unfoldingPath = std::string(argv[4]);
  UnfoldModel model;
  model.loadFoldedMesh(inputModel);
  int methodID = -1;
  bool solve2D = false;
  if(method == "TU"){
    methodID = 4;
    solve2D = true;
  }
  if(method == "AEF"){
    methodID = 4;
    solve2D = false;
  }
  if(method == "cMCF"){
    methodID = 0;
    solve2D = false;
  }
  if(method == "CCF0"){
    methodID = 1;
    solve2D = false;
  }
  if(method == "CCF1"){
    methodID = 2;
    solve2D = false;
  }
  if(method == "CCF2"){
    methodID = 3;
    solve2D = false;
  }
  if(method == "ENAF"){
    methodID = 5;
    solve2D = false;
  }
  if(method == "IIF"){
    methodID = 6;
    solve2D = false;
  }
  int success = -1;
  if(solve2D){
    success = model.solveCollisions2D(methodID, abortTime);
  }else{
    success = model.solveCollisionsGeometric(methodID, abortTime);
  }
  if(success == -1){
    std::cout << "Failed to unfold." << std::endl;
    return 0;
  }
  model.saveUnfolding(unfoldingPath);
  if(argc == 6){
    model.saveFoldedMesh(std::string(argv[5]));
  }
  return 0;
}
