#include <argos3/core/utility/string_utilities.h>
#include <support/squadrons.hh>
std::vector<prez::Squadron>* _STATIC_SQUADRON_LIST = nullptr;

std::vector<prez::Squadron>* prez::GetSquadronList() {
  if (_STATIC_SQUADRON_LIST == nullptr) {
    _STATIC_SQUADRON_LIST = new std::vector<prez::Squadron>();
  }
  return _STATIC_SQUADRON_LIST;
}

std::ostream& operator<<(std::ostream& out, prez::Squadron& squadron) {
  return out << "Squadron(position: " << argos::ToString(squadron.position) << ", force: " << squadron.force << ")";
}
