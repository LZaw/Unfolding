
#include "PolygonUtil.hpp"

// C++ Headers
#include <cmath>
#include <limits>

bool PolygonUtil::lineLineIntersection(const Eigen::Vector2d& l10, const Eigen::Vector2d& l11, const Eigen::Vector2d& l20, const Eigen::Vector2d& l21){
  const std::array<double, 3> sideLine{l10[0] - l11[0], l11[1] - l10[1], l11[0] * l10[1] - l10[0] * l11[1]};
  std::array<double, 2> intersectionCondition{sideLine[1] * l20[0] + sideLine[0] * l20[1] + sideLine[2], sideLine[1] * l21[0] + sideLine[0] * l21[1] + sideLine[2]};

  if((intersectionCondition[0] > 0 && intersectionCondition[1] > 0) ||
     (intersectionCondition[0] < 0 && intersectionCondition[1] < 0)){
    return false;
  }
  const std::array<double, 3> rayLine{l20[0] - l21[0], l21[1] - l20[1], l21[0] * l20[1] - l20[0] * l21[1]};

  intersectionCondition = {rayLine[1] * l10[0] + rayLine[0] * l10[1] + rayLine[2], rayLine[1] * l11[0] + rayLine[0] * l11[1] + rayLine[2]};
  if((intersectionCondition[0] > 0 && intersectionCondition[1] > 0) ||
     (intersectionCondition[0] < 0 && intersectionCondition[1] < 0)){
    return false;
  }
  if(std::fabs((sideLine[0] * rayLine[1]) - (rayLine[0] * sideLine[1])) < std::numeric_limits<float>::epsilon()){
    return false;
  }
  return true;
}
