#ifndef PREZ_SUPPORT_SQUADRONS_HH
#define PREZ_SUPPORT_SQUADRONS_HH
#include <argos3/core/utility/math/vector3.h>
#include <cstdint>
#include <ostream>

namespace prez {
  struct Squadron {
    argos::CVector3 position;
    uint32_t force;
  };

  std::vector<Squadron>* GetSquadronList();
}

std::ostream& operator<<(std::ostream& out, prez::Squadron&);
#endif//PREZ_SUPPORT_SQUADRONS_HH
