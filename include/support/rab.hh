#ifndef RAB_HH
#define RAB_HH
/** @file rab.hh */
namespace prez {
  /** This is a trick commonly used in the worst places possible (ex: Linux drivers)
   * It uses the implicit and constexpr std::iota of enums definitions to set sequential values.
   * */
  enum RABKey {
    ID = 0,
    TARGET,
    TASK_ALLOCATOR_STATE,
    TASK_EXECUTOR_STATE
  };
}
#endif//RAB_HH
