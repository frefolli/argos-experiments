#ifndef TASK_ALLOCATORS_NN2_HH
#define TASK_ALLOCATORS_NN2_HH
/** @file nn2.hh */
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
  class NN2 : public TaskAllocator
  {
    static constexpr uint32_t MAX_REVIEWING_SESSIONS = 20;

  public:
    enum State
    {
      START,
      REVIEWING,
      IDLE
    } state = START;
    uint32_t reviewing_sessions = 0;

    std::unordered_map<uint32_t, double_t> distances_from_targets;
    double_t max_distance_from_targets;
    double_t min_distance_from_targets = std::numeric_limits<int>::max();
    double_t nearest_target;

    Task *task;
    argos::CRandom::CRNG *random_number_generator;
    argos::CCI_PositioningSensor *positioning_sensor;
    argos::CCI_RangeAndBearingSensor *range_and_bearing_sensor;

    void Start()
    {
      const argos::CCI_RangeAndBearingSensor::TReadings neighbours = range_and_bearing_sensor->GetReadings();
      uint32_t const n = argos::CSimulator::GetInstance().GetSpace().GetEntitiesByType("eye-bot").size();
      assert(n == neighbours.size() + 1); // we are able to reach all other drones with our rab

      std::vector<prez::Target> *targets = prez::GetTargetList();
      if (targets->empty())
      {
        std::cerr << "Warning: targets is empty" << std::endl;
      }
      else
      {
        max_distance_from_targets = 0.0f;
        argos::CVector3 position = positioning_sensor->GetReading().Position;
        for (uint32_t index = 0; index < targets->size(); ++index)
        {
          distances_from_targets[index] = (position - targets->at(index).position).Length();
          max_distance_from_targets = std::max(max_distance_from_targets, distances_from_targets[index]);
          if (distances_from_targets[index] < min_distance_from_targets)
          {
            min_distance_from_targets = distances_from_targets[index];
            nearest_target = index;
          }
        }
        task->target = nearest_target; // my target is the nearest to me
        state = State::REVIEWING;
        ++reviewing_sessions;
      }
    }

    void Review()
    {
      std::vector<prez::Target> *targets = prez::GetTargetList();
      /** formations is a X:Y map. It has to be thought as:
       * at this time we sense Y drones belonging to the X target */
      std::unordered_map<uint32_t, uint32_t> formations;
      /** with rab we sense all other drones in the arena */
      const argos::CCI_RangeAndBearingSensor::TReadings neighbours = range_and_bearing_sensor->GetReadings();
      for (argos::CCI_RangeAndBearingSensor::SPacket neighbour : neighbours)
      {
        ++formations[neighbour.Data[prez::RABKey::TARGET]];
      }
      /** we belong to the target "target" */
      ++formations[task->target];

      // if the current target has enough drones assigend to it...we relocate
      if (formations[task->target] > targets->at(task->target).force)
      {
        argos::CRange<uint32_t> target_range(0, targets->size());
        uint32_t temp = task->target;
        for (; temp == task->target;) // we want to avoid to join the current squadron
        {
          temp = random_number_generator->Uniform(target_range);
        }
        task->target = temp;
      }

      ++reviewing_sessions;
      if (reviewing_sessions > MAX_REVIEWING_SESSIONS)
      {
        state = State::IDLE;
      }
      else
      {
        ++reviewing_sessions;
      }
    }

    void Idle() {}

    void Round()
    {
      switch (state)
      {
      case State::START:
        Start();
        break;
      case State::REVIEWING:
        Review();
        break;
      case State::IDLE:
        Idle();
        break;
      }
    }

    void Reset()
    {
      state = START;
      reviewing_sessions = 0;
      distances_from_targets.clear();
      max_distance_from_targets = 0.0f;
    }
  };
}
#endif // TASK_ALLOCATORS_NN2_HH
