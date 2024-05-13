
#include "FaceList.hpp"

// C++ Header
#include <algorithm>
#include <cstdlib>
#include <numeric>

FaceList::FaceList():
  std::vector<std::vector<unsigned int>>(){
}

FaceList::FaceList(size_type size):
  std::vector<std::vector<unsigned int>>(size){
}
