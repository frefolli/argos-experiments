#include <support/target.hh>
std::vector<prez::Target>* _STATIC_TARGET_LIST = nullptr;

std::vector<prez::Target>* prez::GetTargetList() {
  if (_STATIC_TARGET_LIST == nullptr) {
    _STATIC_TARGET_LIST = new std::vector<prez::Target>();
  }
  return _STATIC_TARGET_LIST;
}
