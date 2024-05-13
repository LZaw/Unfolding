
#pragma once

// C++ Headers
#include <unordered_map>
#include <vector>

class FaceList : public std::vector<std::vector<unsigned int>>{
public:
  FaceList();
  FaceList(size_type size);
};