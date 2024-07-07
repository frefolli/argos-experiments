#ifndef PREZ_SUPPORT_TARGETS_HH
#define PREZ_SUPPORT_TARGETS_HH
#include <argos3/core/utility/math/vector3.h>
#include <cstdint>
#include <ostream>

namespace prez {
  struct Target {
    argos::CVector3 position;
    uint32_t force;//number of drones that ideally should be assigned to him
  };

  std::vector<Target>* GetTargetList();
}

std::ostream& operator<<(std::ostream& out, prez::Target&);
#endif//PREZ_SUPPORT_TARGETS_HH
