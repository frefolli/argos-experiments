#ifndef TASK_ALLOCATORS_RandomChoice_HH
#define TASK_ALLOCATORS_RandomChoice_HH
/** @file RandomChoice.hh */
#include <limits>
#include <support/task_allocator.hh>
#include <support/targets.hh>
#include <support/rab.hh>
#include <support/task.hh>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/core/utility/math/rng.h>

namespace prez::task_allocators
{
  class RandomChoice : public TaskAllocator
  {

  public:
    enum State
    {
      START,
      IDLE
    } state = START;

    Task *task;
    argos::CRandom::CRNG *random_number_generator;
    argos::CCI_PositioningSensor *positioning_sensor;
    argos::CCI_RangeAndBearingSensor *range_and_bearing_sensor; // not used here but every task_allocators should have it

    void Start()
    {
      std::vector<prez::Target> *targets = prez::GetTargetList();
      argos::CRange<uint32_t> target_range(0, targets->size());
      task->target = random_number_generator->Uniform(target_range);

      state = State::IDLE;
    }

    void Idle() {}

    void Round()
    {
      switch (state)
      {
      case State::START:
        Start();
        break;
      case State::IDLE:
        Idle();
        break;
      }
    }

    void Reset()
    {
      state = START;
    }
  };
}
#endif // TASK_ALLOCATORS_RandomChoice_HH
