#ifndef PREZ_SUPPORT_TARGET_HH
#define PREZ_SUPPORT_TARGET_HH
#include <argos3/core/utility/math/vector3.h>
#include <cstdint>

namespace prez {
  struct Target {
    argos::CVector3 position;
    uint32_t force;
  };

  std::vector<Target>* GetTargetList();
}
#endif//PREZ_SUPPORT_TARGET_HH
