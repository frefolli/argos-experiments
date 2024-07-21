#include <argos3/core/utility/string_utilities.h>
#include <support/targets.hh>
std::vector<prez::Target>* _STATIC_TARGET_LIST = nullptr;

std::vector<prez::Target>* prez::GetTargetList() {
  if (_STATIC_TARGET_LIST == nullptr) {
    _STATIC_TARGET_LIST = new std::vector<prez::Target>();
  }
  return _STATIC_TARGET_LIST;
}

std::ostream& operator<<(std::ostream& out, prez::Target& target) {
  return out << "Target(position: " << argos::ToString(target.position) << ", force: " << target.force << ")";
}
