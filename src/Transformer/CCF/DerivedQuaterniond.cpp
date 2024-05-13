
#include "DerivedQuaterniond.hpp"

DerivedQuaterniond operator*(double s, const DerivedQuaterniond& q){
  return q * s;
}
