#ifndef TASK_EXECUTOR_HH
#define TASK_EXECUTOR_HH
/** @file task_executor.hh */
namespace prez {
  class TaskExecutor {
    public:
      virtual void Round() {}
      virtual void Reset() {}
  };
}
#endif//TASK_EXECUTOR_HH
