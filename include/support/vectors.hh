#ifndef PREZ_SUPPORT_VECTORS_HH
#define PREZ_SUPPORT_VECTORS_HH
/** @file vectors.hh */
#include <argos3/core/utility/math/vector3.h>
#include <cstdint>
#include <string>

namespace prez {
  inline double_t Distance(argos::CVector3& A, argos::CVector3& B) {
    return std::sqrt(
      std::pow(A.GetX() - B.GetX(), 2) +
      std::pow(A.GetY() - B.GetY(), 2) +
      std::pow(A.GetZ() - B.GetZ(), 2)
    );
  }

  inline std::string ToString(argos::CVector3& v) {
    return "(" + std::to_string(v.GetX()) + "," + std::to_string(v.GetY()) + "," + std::to_string(v.GetZ()) + ")";
  }
};
#endif//PREZ_SUPPORT_VECTORS_HH