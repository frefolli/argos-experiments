#ifndef TASK_ALLOCATOR_HH
#define TASK_ALLOCATOR_HH
/** @file task_allocator.hh */
namespace prez {
  class TaskAllocator {
    public:
      virtual void Round() {}
      virtual void Reset() {}
  };
}
#endif//TASK_ALLOCATOR_HH
