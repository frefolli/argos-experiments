#ifndef PREZ_SUPPORT_VECTORS_HH
#define PREZ_SUPPORT_VECTORS_HH
/** @file vectors.hh */
#include <argos3/core/utility/math/vector3.h>
#include <cstdint>
#include <string>

namespace prez {
  inline std::string ToString(argos::CVector3& v) {
    return "(" + std::to_string(v.GetX()) + "," + std::to_string(v.GetY()) + "," + std::to_string(v.GetZ()) + ")";
  }
};
#endif//PREZ_SUPPORT_VECTORS_HH
